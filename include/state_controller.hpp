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
#include <boost/process.hpp>
#include <ctime>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/as_status.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "./Can-Header-Map/CAN_asdb.h"
#include "./Can-Header-Map/CANOPEN_db.h"
#include "lart_common.h"

#define T24E_CAN_INTERFACE "can0"
#define MAX_ACTUATOR_POS 492000//492200 //assuming a maximum steering wheel angle of 105 degrees
#define RES_CAN_ID 0x191

#define DINAMICS_STEERING_ID 0x446 //id for the steering angle from SPAC
#define DBC_MESSAGES 0x613 //temporary
#define DBC_IMU 0x501 //id for the imu data
#define IMU_GPS_POSE 0x1235 //id for the imu gps pose
#define IMU_TURN_RATE 0x1236 //id for the imu turn rate
#define IMU_ACCELERATION 0x1237 //id for the imu acceleration


#define RECORD_BAG "ros2 bag record -s mcap -o "
#define BAG_DIRECTORY "/home/lart-tasha/Documents/bags/"
#define BAG_TOPICS "/acu_origin/dynamics /mapping/cones /mapping/cones_markers /pc_origin/dynamics /pc_origin/system_status/critical_as/mission /pc_origin/system_status/critical_as/state /planned_path_topic /rviz_path_topic /target_marker_topic /zed/depth/camera_info /zed/left/camera_info /imu/angular_velocity /zed/left/image_raw/compressed /zed/depth/image_raw /ekf/state /gnss_pose /ekf/stats /ekf/map /ekf/cone_markers /tf /tf_static /spac/path_marker"

namespace bp = boost::process;

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

  /**
   * @brief This function sets the state to emergency
   * 
   */
  void setEmergency();
  
  /**
   * @brief This function sends the current state 
   */
  void sendState();

  /**
   * @brief This function gets the current cone count, total of cones in the map and lap count from the ekf slam
   * 
   */
  void ekfStatsCallback(const lart_msgs::msg::SlamStats::SharedPtr msg);
  
  std::string stateToString(int state);

  void startRecordBagProcess();

  void sendImuCanMessages();

  void angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  void accelerationsCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  void check_maxon_timeout();



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
  long maxon_start_position;
  bool maxon_start_position_set = false; // flag to check if the maxon start position is set
  uint16_t last_valid_rpm = 0; // last valid rpm received from spac
  bool bag_recording = false; // flag to check if the bag is being recorded
  int bag_process;
  boost::process::child bag_process_; // Member variable to store the process
  bool dynamics_available = false; // flag to check if the dynamics message is available
  bool last_maxon_position_set = false; // flag to check if the last maxon position is set
  std::chrono::steady_clock::time_point maxon_message_time;//time the state was changed to ready


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


  // dbc variables
  float last_target_angle = 0.0f;
  float last_actual_angle = 0.0f;
  float last_target_speed = 0.0f;
  float last_actual_speed = 0.0f;
  uint16_t last_lap_count = 0;
  uint16_t last_total_cone_count = 0;
  uint16_t last_current_cone_count = 0;

  float last_angular_velocity_z = 0.0f;
  float last_acceleration_x = 0.0f;
  float last_acceleration_y = 0.0f;


  // mission controller subscription
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr mission_finished_sub_;

  //spac subscription
  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr spac_sub_;

  // emergency stop subscription
  rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr emergency_sub_;

  // ekf stats subscriber
  rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr ekf_stats_sub_;

  //inspection steering angle publisher
  rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr inspection_steering_angle_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_angular_velocity_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_acceleration_sub_;

  //spac publisher
  rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr spac_publisher;

  // state publisher to pc
  rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher_;

  //mission_publisher
  rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_publisher_;

  // imu gps pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_gps_pose_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_turn_rate_publisher_;

};

#endif
