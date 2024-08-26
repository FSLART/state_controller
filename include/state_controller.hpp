#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

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

class StateController : public rclcpp::Node
{
public:
  StateController();

private:
  // Function declarations
  std::string compute_sha256(const std::string &data);
  void publish_handshake();
  void acuStateCallback(const lart_msgs::msg::State::SharedPtr msg);
  void handshakeCallback(const lart_msgs::msg::ASStatus::SharedPtr msg);
  void ready_state();
  void ebsStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void goSignalCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void missionFinishedCallback(const lart_msgs::msg::ASStatus::SharedPtr msg);
  void standstillCallback(const std_msgs::msg::Bool::SharedPtr msg);
  bool valid_state(lart_msgs::msg::State msg);

  // Member variables
  lart_msgs::msg::State state_msg;
  bool mission_finished;
  int handshake_recived;
  std::chrono::steady_clock::time_point last_publish;
  std::chrono::steady_clock::time_point ready_start_time_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_brake_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go_signal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr standstill_sub_;
  rclcpp::Subscription<lart_msgs::msg::ASStatus>::SharedPtr mission_finished_sub_;
  rclcpp::Subscription<lart_msgs::msg::ASStatus>::SharedPtr handshake_sub_;
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr acu_state_sub_;

  // Publishers
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<lart_msgs::msg::ASStatus>::SharedPtr handshake_publisher_;
};

#endif
