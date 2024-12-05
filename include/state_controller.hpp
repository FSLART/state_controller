#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP

#include <functional>
#include <memory>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "linux/can.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <thread>
#include <unistd.h> 
#include <array>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/as_status.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "./Can-Header-Map/CAN_asdb.h"
#include "./Can-Header-Map/CANOPEN_maxondb.h"
#include "lart_common.h"

#define T24E_CAN_INTERFACE "can0"
#define REMOTE_NODE_ID 0x05
#define MAX_ACTUATOR_POS 492200


//#define RES_READY_CAN_ID 0x0B//see real id

class StateController : public rclcpp::Node{
public:
  StateController();

private:
  // Function declarations
  void read_can_frame();
  void missionFinishedCallback(const lart_msgs::msg::State::SharedPtr msg);
  bool valid_state(lart_msgs::msg::State msg);
  void send_can_frames();
  void send_can_frame(struct can_frame frame);
  void handle_can_frame(struct can_frame frame);
  void spacCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);
  void emergencyCallback(const lart_msgs::msg::State::SharedPtr msg);
  void inspectionSteeringAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void maxon_activation();
  void sendPosToMaxon(float angle);
  void resetMaxon();


  // class variables
  int s=-1;//socket descriptor

  lart_msgs::msg::State state_msg;//save the current state
  lart_msgs::msg::DynamicsCMD dynamics_msg;//save the current dynamics command
  bool mission_finished;
  std::chrono::steady_clock::time_point ready_change;//time the state was changed to ready
  bool res_ready;
  bool relative_zero_set; // flag to check if relative zero is set
  long relative_maxon_zero; // relative zero value

  //id 0x185
  uint32_t statusword1;
  uint32_t mode;
  uint32_t error_code;

  //id 0x285
  uint32_t target_position;
  uint32_t target_speed;

  //id 0x385
  uint32_t statusword2;
  uint32_t actual_position;
  uint32_t actual_moment;

  //id 0x485
  uint32_t statusword3;
  uint32_t actual_speed;
  uint32_t actual_pwm_duty;

  std::mutex state_mutex;
  std::mutex socket_mutex;

  // mission controller subscription
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr mission_finished_sub_;

  //spac subscription
  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr spac_sub_;

  // emergency stop subscription
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr emergency_sub_;

  // state publisher to pc
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;

  //mission_publisher
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_publisher_;

  //inspection steering angle publisher
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr inspection_steering_angle_sub_;
};

#endif
