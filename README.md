# T24e State Controller
This ROS2 package written in c++ has the function of controlling the autonomous state of the vehicle and acting as a Can Bus bridge between the autonomous computer (Jetson) the ACU and the steering wheel controller (Maxon).

# Inputs
- Actual RPM from  ACU
- Maxon Feedback
- RPM and Steering angle from SPAC
- Autonomous State from ACU
- Autonomous Mission from ACU

# Outputs
- Maxon encoder position
- RPM to ACU
- Autonomous State to Jetson
- Autonomous Mission to Jetson



# How to Run
```sh
source install/setup.bash

ros2 run state_controller state_controller
```