#include <functional>
#include <memory>
#include <chrono>
#include "exceptions.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"


using std::placeholders::_1;



class StateController : public rclcpp::Node
{

public:
  StateController() : Node("state_controller"), systems_ready(false), mission_finished(false)
  {
    
    systems_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "SystemsReady", 10, std::bind(&StateController::systemsReadyCallback, this, _1));

    mission_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "Mission", 10, std::bind(&StateController::missionCallback, this, _1));

    go_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "GoSignal", 10, std::bind(&StateController::goSignalCallback, this, _1));

    emergency_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "Emergency_Brake", 10, std::bind(&StateController::ebsStatusCallback, this, _1));

    standstill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "Standstill", 10, std::bind(&StateController::standstillCallback, this, _1));

    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("state", 10);
    handshake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("Handshake", 10);

    state_msg.data= lart_msgs::msg::State::OFF;
  }

private:
  lart_msgs::msg::State state_msg;
  
  bool systems_ready=false;
  bool mission_finished=false;

  void systemsReadyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    systems_ready = msg->data;
    if (systems_ready)
    {
      ready_state();
    }
  }

  void ready_state()
  {
      if (state_msg.data  == lart_msgs::msg::State::OFF)
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
      else if(!valid_state(state_msg))
      {
        throw BadStateException("Bad state detected");
      }  
  }
  
  void ebsStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      state_msg.data = lart_msgs::msg::State::EMERGENCY;
      RCLCPP_INFO(this->get_logger(), "Emergency Brake system activated");
      state_publisher_->publish(state_msg);
    }
  }


  void goSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)
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

  void missionCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data){
      mission_finished=true;
    }
  }

  void standstillCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if(msg->data && mission_finished){
      state_msg.data = lart_msgs::msg::State::FINISH;
      RCLCPP_INFO(this->get_logger(), "Mission finished");
      state_publisher_->publish(state_msg);
    }
  }

  bool valid_state(lart_msgs::msg::State msg)
  {
    return (msg.data == lart_msgs::msg::State::OFF || msg.data == lart_msgs::msg::State::READY || msg.data == lart_msgs::msg::State::DRIVING || msg.data == lart_msgs::msg::State::EMERGENCY || msg.data == lart_msgs::msg::State::FINISH);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_brake_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr systems_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go_signal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr standstill_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_sub_;

  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr handshake_publisher_;

  std::chrono::steady_clock::time_point ready_start_time_;
};

int main(int argc, char *argv[])
{
  try{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateController>());
    rclcpp::shutdown();
  }catch(const EmergencyException& e){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Emergency Exception: %s", e.what());
    return EXIT_FAILURE;
  }catch(const BadStateException& e){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Bad State Exception: %s", e.what());
    return EXIT_FAILURE;
  }catch(const std::exception& e){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    return EXIT_FAILURE;
  }
  
  return 0;
}
