#include <functional>
#include <memory>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "exceptions.hpp"
#include "openssl/sha.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/as_status.hpp"


using std::placeholders::_1;


class StateController : public rclcpp::Node
{

public:
  StateController() : Node("state_controller"), mission_finished(false)
  {
    
    handshake_sub_ = this->create_subscription<lart_msgs::msg::ASStatus>("/acu_origin/system_status/critical_as/", 10, std::bind(&StateController::handshakeCallback, this, _1));

    mission_sub_ = this->create_subscription<lart_msgs::msg::Mission>("/acu_origin/system_status/critical_as/", 10, std::bind(&StateController::missionCallback, this, _1));//mission finished

    go_signal_sub_ = this->create_subscription<std_msgs::msg::Bool>("GoSignal", 10, std::bind(&StateController::goSignalCallback, this, _1));//ready to drive button, need to know the topic and type of message

    emergency_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>("/acu_origin/system_status/critical_as/ebs_brk", 10, std::bind(&StateController::ebsStatusCallback, this, _1));//need to know the type of message

    standstill_sub_ = this->create_subscription<std_msgs::msg::Bool>("Standstill", 10, std::bind(&StateController::standstillCallback, this, _1));// velocity = 0 need to sub spac probably

    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10);

    acu_state_sub_ = this->create_subscription<lart_msgs::msg::State>("/acu_origin/system_status/critical_as/state", 10, std::bind(&StateController::acuStateCallback, this, _1));

    handshake_publisher_ = this->create_publisher<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as/", 10);

    state_msg.data= lart_msgs::msg::State::OFF;//initialize state as off
  }

private:
  lart_msgs::msg::State state_msg;

  bool mission_finished=false;

  int handshake_recived = 0;
  

  std::string compute_sha256(const std::string &data) {
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(data.c_str()), data.size(), hash);

    std::stringstream ss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
    }
    return ss.str();
  }

  void publish_handshake(){
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

  void acuStateCallback(const lart_msgs::msg::State::SharedPtr msg)
  {
    if (msg->data == lart_msgs::msg::State::EMERGENCY)
    {
      state_msg.data = lart_msgs::msg::State::EMERGENCY;
      RCLCPP_INFO(this->get_logger(), "Emergency state activated");
      state_publisher_->publish(state_msg);
    }else if (msg->data == lart_msgs::msg::State::DRIVING){
      state_msg.data = lart_msgs::msg::State::DRIVING;
      RCLCPP_INFO(this->get_logger(), "Driving state activated");
      state_publisher_->publish(state_msg);
    }
  }

  void handshakeCallback(const lart_msgs::msg::ASStatus::SharedPtr msg)
  {
    if(msg->hash.empty() || msg->timestamp == 0){
      handshake_recived=1;
      ready_state();
    }
    else{
      auto now = std::chrono::system_clock::now();
      auto epoch = now.time_since_epoch();
      auto unix_time = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
    
      std_msgs::msg::String verify_recived_msg;
      verify_recived_msg.data = compute_sha256(std::to_string(unix_time));
      if(msg->hash.compare(verify_recived_msg.data) == 0 && msg->timestamp == unix_time){
        handshake_recived=1;
        ready_state();
      }
      else{
        handshake_recived=0;
        publish_handshake();
      }
    }
  }

  void ready_state()
  {
      if (state_msg.data  == lart_msgs::msg::State::OFF)
      {
        state_msg.data = lart_msgs::msg::State::READY;
        ready_start_time_ = std::chrono::steady_clock::now();//start timer
        RCLCPP_INFO(this->get_logger(), "Ready state activated");
        state_publisher_->publish(state_msg);
      }
      else if (state_msg.data == lart_msgs::msg::State::EMERGENCY)
      {
        throw EmergencyException("Emergency state detected");//emergency exception
      }
      else if(!valid_state(state_msg))
      {
        throw BadStateException("Bad state detected");//bad state exception
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


  void goSignalCallback(const std_msgs::msg::Bool::SharedPtr msg)//if the state has been ready for at least 5 seconds and R2D button has been pressed, change to driving state
  {
    if (msg->data && state_msg.data == lart_msgs::msg::State::READY)
    {
      auto now = std::chrono::steady_clock::now();//get current time
      auto ready_duration = std::chrono::duration_cast<std::chrono::seconds>(now - ready_start_time_).count();//check if 5 seconds have passed

      if (ready_duration >= 5)
      {
        state_msg.data = lart_msgs::msg::State::DRIVING;
        RCLCPP_INFO(this->get_logger(), "Driving state activated");
        state_publisher_->publish(state_msg);
      }
    }
  }

  void missionCallback(const lart_msgs::msg::Mission::SharedPtr msg)
  {
    if(msg->data==lart_msgs::msg::Mission::FINISHED){
      mission_finished=true;
    }
  }

  void standstillCallback(const std_msgs::msg::Bool::SharedPtr msg)//car velocity = 0 and all the laps have been completed
  {
    if(msg->data && mission_finished){
      state_msg.data = lart_msgs::msg::State::FINISH;
      RCLCPP_INFO(this->get_logger(), "Mission finished");
      state_publisher_->publish(state_msg);
    }
  }

  bool valid_state(lart_msgs::msg::State msg)//check if the state is valid for bad state exception
  {
    return (msg.data == lart_msgs::msg::State::OFF || msg.data == lart_msgs::msg::State::READY || msg.data == lart_msgs::msg::State::DRIVING || msg.data == lart_msgs::msg::State::EMERGENCY || msg.data == lart_msgs::msg::State::FINISH);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_brake_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go_signal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr standstill_sub_;
  rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<lart_msgs::msg::ASStatus>::SharedPtr handshake_sub_;
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr acu_state_sub_;

  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<lart_msgs::msg::ASStatus>::SharedPtr handshake_publisher_;

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
