#include "state_controller.hpp"

using std::placeholders::_1;

StateController::StateController() : Node("state_controller"), mission_finished(false)
{
    handshake_sub_ = this->create_subscription<lart_msgs::msg::ASStatus>("/acu_origin/system_status/critical_as", 10, std::bind(&StateController::handshakeCallback, this, _1));

    mission_finished_sub_ = this->create_subscription<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as", 10, std::bind(&StateController::missionFinishedCallback, this, _1));

    go_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>("GoSignal", 10, std::bind(&StateController::goSignalCallback, this, _1));

    emergency_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>("/acu_origin/system_status/critical_as/ebs_brk", 10, std::bind(&StateController::ebsStatusCallback, this, _1));

    standstill_sub_ = this->create_subscription<std_msgs::msg::Bool>("Standstill", 10, std::bind(&StateController::standstillCallback, this, _1));

    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10);

    acu_state_sub_ = this->create_subscription<lart_msgs::msg::State>("/acu_origin/system_status/critical_as/state", 10, std::bind(&StateController::acuStateCallback, this, _1));

    handshake_publisher_ = this->create_publisher<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as", 10);

    state_msg.data = lart_msgs::msg::State::OFF; // initialize state as off
}

std::string StateController::compute_sha256(const std::string &data)
{
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(data.c_str()), data.size(), hash);

    std::stringstream ss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
    }
    return ss.str();
}

void StateController::publish_handshake()
{
    lart_msgs::msg::ASStatus handshake_msg;

    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto unix_time = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();

    std_msgs::msg::String hash_msg;
    hash_msg.data = compute_sha256(std::to_string(unix_time));

    handshake_msg.timestamp = unix_time;
    handshake_msg.hash = hash_msg.data;
    handshake_publisher_->publish(handshake_msg);
}

void StateController::acuStateCallback(const lart_msgs::msg::State::SharedPtr msg)
{
    if (msg->data == lart_msgs::msg::State::EMERGENCY)
    {
        state_msg.data = lart_msgs::msg::State::EMERGENCY;
        RCLCPP_INFO(this->get_logger(), "Emergency state activated");
        state_publisher_->publish(state_msg);
    }
}

void StateController::handshakeCallback(const lart_msgs::msg::ASStatus::SharedPtr msg)
{
    if (msg->hash.empty() || msg->timestamp == 0)
    {
        handshake_recived = 1;
        ready_state();
    }
    else
    {
        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto unix_time = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();

        std_msgs::msg::String verify_recived_msg;
        verify_recived_msg.data = compute_sha256(std::to_string(unix_time));
        if (msg->hash.compare(verify_recived_msg.data) == 0 && msg->timestamp == unix_time)
        {
            handshake_recived = 1;
            ready_state();
        }
        else
        {
            handshake_recived = 0;
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_publish).count() >= 1)
            {
                publish_handshake();
                last_publish = std::chrono::steady_clock::now();
            }
        }
    }
}

void StateController::ready_state()
{
    if (state_msg.data == lart_msgs::msg::State::OFF)
    {
        state_msg.data = lart_msgs::msg::State::READY;
        ready_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Ready state activated");
        state_publisher_->publish(state_msg);
    }
    else if (state_msg.data == lart_msgs::msg::State::EMERGENCY)
    {
        throw EmergencyException("Emergency state detected");
    }
    else if (!valid_state(state_msg))
    {
        throw BadStateException("Bad state detected");
    }
}

void StateController::ebsStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        state_msg.data = lart_msgs::msg::State::EMERGENCY;
        RCLCPP_INFO(this->get_logger(), "Emergency Brake system activated");
        state_publisher_->publish(state_msg);
    }
}

void StateController::goSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && state_msg.data == lart_msgs::msg::State::READY)
    {
        auto now = std::chrono::steady_clock::now();
        auto ready_duration = std::chrono::duration_cast<std::chrono::seconds>(now - ready_start_time_).count();

        if (ready_duration >= 5)
        {
            state_msg.data = lart_msgs::msg::State::DRIVING;
            RCLCPP_INFO(this->get_logger(), "Driving state activated");
            state_publisher_->publish(state_msg);
        }
    }
}

void StateController::missionFinishedCallback(const lart_msgs::msg::ASStatus::SharedPtr msg)
{
    if (msg->state.data == lart_msgs::msg::State::FINISH)
    {
        mission_finished = true;
    }
    else
    {
        mission_finished = false;
    }
}

void StateController::standstillCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && mission_finished)
    {
        state_msg.data = lart_msgs::msg::State::FINISH;
        RCLCPP_INFO(this->get_logger(), "Mission finished, state changed to Finish");
        state_publisher_->publish(state_msg);
    }
}

bool StateController::valid_state(lart_msgs::msg::State msg)
{
    return (msg.data == lart_msgs::msg::State::OFF || msg.data == lart_msgs::msg::State::READY || msg.data == lart_msgs::msg::State::DRIVING || msg.data == lart_msgs::msg::State::EMERGENCY || msg.data == lart_msgs::msg::State::FINISH);
}

int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<StateController>());
        rclcpp::shutdown();
    }
    catch (const EmergencyException &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Emergency Exception: %s", e.what());
        return EXIT_FAILURE;
    }
    catch (const BadStateException &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Bad State Exception: %s", e.what());
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        return EXIT_FAILURE;
    }

    return 0;
}
