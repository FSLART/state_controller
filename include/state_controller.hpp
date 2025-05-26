/**
 * @file state_controller.hpp
 * @author André Lopes (2230779@my.ipleiria.pt)
 * @brief A ros node that controls the autonomous state of the vehicle and is a can bridge between the acu and the autonomous pc
 * @version 0.1
 * 
 */
#ifndef STATE_CONTROLLER_HPP
#define STATE_CONTROLLER_HPP
#define __LART_AXANATO_VCU_GATEWAY__

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
#include "lart_msgs/msg/dynamics.hpp"
#include "./Can-Header-Map/CAN_asdb.h"
#include "./Can-Header-Map/CANOPEN_db.h"
#include "lart_common.h"

#define T24E_CAN_INTERFACE "can0"
#define MAX_ACTUATOR_POS 492200 //assuming a maximum steering wheel angle of 105 degrees
#define RES_CAN_ID 0x191


//#define RES_READY_CAN_ID 0x0B//see real id

class StateController : public rclcpp::Node{
public:
  StateController();

private:
  // Function declarations
  void read_can_frame();
  void missionFinishedCallback(const lart_msgs::msg::State::SharedPtr msg);
  /**
   * @brief A function that verifies if the received state is valid(used when the acu would set the autonomous state)
   * 
   * @param msg 
   * @return true 
   * @return false 
   */
  bool valid_state(lart_msgs::msg::State msg);
  /**
   * @brief This function will repeatedly send the state of the autonomous system to the acu
   * 
   */
  void send_can_frames();
  void send_can_frame(struct can_frame frame);
  /**
   * @brief Each switch case on this function will handle a specific can id and update the corresponding variables, this are either feedbask from maxon, information from the IMU or messages from the acu
   * 
   * @param frame 
   */
  void handle_can_frame(struct can_frame frame);
  /**
   * @brief Here the state_controller receives the rpm and steering angle from SPAC and sends the maxon position directly and the rpm to the acu
   * 
   * @param msg 
   */
  void spacCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);
  /**
   * @brief Receives a emergency message from any other node and sets the state to emergency
   * 
   * @param msg 
   */
  void emergencyCallback(const lart_msgs::msg::State::SharedPtr msg);
  /**
   * @brief Inspection mission is currently publishing the dynamics command onto a different topic than SPAC, this does basically the same as spacCallback but for the inspection mission
   * 
   * @param msg 
   */
  void inspectionSteeringAngleCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);
  /**
   * @brief To activate the maxon some specific values must be sent to it, this function does that
   * in order the values and ids are:
   * 
   * id 0x00: 
   *  - 0x00 0x05 -> turn on maxon
   *  - 0x80 0x05 -> enter pre op mode
   *  - 0x01 0x005  -> enter op mode
   * 
   * id 0x205:
   *  - 0x06 0x00 -> set maxon to profile position mode
   *  - 0x0F 0x00 -> set maxon to profile velocity mode
   * 
   * If all went well the maxon is activated and the led is solid green
   * 
   * The DLC of the can frame NEEDS to be 2, any other way IT WILL NOT WORK!!!
   * 
   */
  void maxon_activation();
  /**
   * @brief This receives the steering angle in radians and verifies if the position is "safe" to send to the maxon
   * to send the position maxon is a bit picky, first you need to say that you are sending a position, then you need to send the position in 6 bytes(more on the next lines), the position is in encoder ticks
   * to say that you are sending a position you send 0x0F 0x00 to the id 0x205, internaly this toggles a "new position bit" on the controller
   * 
   * The possition needs to be sent on a can frame with DLC of 6, the first two bytes are 0x3F 0x00, the next 4 bytes are the position in little endian, the id for the position is 0x405
   * 0x3f means 'start immediately', first we were sending 0x1F and the movement was not smooth at all, the maxon was wainting to reach the previous position before moving to the new one
   *
   * @param angle 
   */
  void sendPosToMaxon(float angle);
  /**
   * @brief This function resets the maxon, it sends a can frame with the id 0x00 and the data 0x81 0x05, this resets the communication protocol of the maxon controller(EPOS 4)
   * 
   */
  void resetMaxon();
  /**
  * @brief This function uses a liner regression to get and aproximate steering ratio for given angle
  */
  //float steeringRatio(float angle);

  /**
   * @brief This function sets the state to emergency
   * 
   */
  void setEmergency();
  
  /**
   * @brief This function sends the current state 
   */
  void sendState();
  


  // class variables
  int s=-1;//socket descriptor

  lart_msgs::msg::State state_msg;//save the current state
  lart_msgs::msg::DynamicsCMD dynamics_msg;//save the current dynamics command
  bool mission_finished;
  std::chrono::steady_clock::time_point ready_change;//time the state was changed to ready
  bool res_ready;
  bool relative_zero_set; // flag to check if relative zero is set
  long relative_maxon_zero; // relative zero value (maxon encoder position)
  bool maxon_activated;
  uint16_t current_rpm = 0;//save the current speed
  lart_msgs::msg::Mission mission;//save the mission


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

  //spac publisher
  rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr spac_publisher;

  // emergency stop subscription
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr emergency_sub_;

  // state publisher to pc
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;

  //mission_publisher
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_publisher_;

  //inspection steering angle publisher
  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr inspection_steering_angle_sub_;
};

#endif
