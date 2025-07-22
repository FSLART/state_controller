#include "state_controller.hpp"

using std::placeholders::_1;

StateController::StateController() : Node("state_controller"){
    mission_finished_sub_ = this->create_subscription<lart_msgs::msg::State>("/pc_origin/system_status/critical_as", 10, std::bind(&StateController::missionFinishedCallback, this, _1));

    emergency_sub_ = this->create_subscription<lart_msgs::msg::State>("/pc_origin/emergency", 10, std::bind(&StateController::emergencyCallback, this, _1));//decide the topic name

    spac_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/pc_origin/dynamics", 10, std::bind(&StateController::spacCallback, this, _1));//change for right topic name

    spac_publisher = this->create_publisher<lart_msgs::msg::Dynamics>("/acu_origin/dynamics", 10);
    
    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10);

    mission_publisher_ = this->create_publisher<lart_msgs::msg::Mission>("/acu_origin/system_status/critical_as/mission", 10);

    inspection_steering_angle_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/cmd", 10, std::bind(&StateController::inspectionSteeringAngleCallback, this, _1));

    ekf_stats_sub_ = this->create_subscription<lart_msgs::msg::SlamStats>("/ekf/stats", 10, std::bind(&StateController::ekfStatsCallback, this, _1));
    
    imu_gps_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_pose", 10);

    imu_turn_rate_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/angular_velocity", 10);



    state_msg.data = lart_msgs::msg::State::OFF; // initialize state as off

    mission.data = lart_msgs::msg::Mission::MANUAL; // initialize mission as manual //CHANGE BACK TO MANUAL
    
    mission_finished = false;

    res_ready = false;

    relative_zero_set = false;

    relative_maxon_zero = 0;

    // create a socket
	if((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
		return;
	}

    //define can interface
    struct ifreq ifr;
    strcpy(ifr.ifr_name, T24E_CAN_INTERFACE);
    ioctl(this->s, SIOCGIFINDEX, &ifr);
    
    // bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(this->s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));
        exit(1);
    }

    // create a thread to read CAN frames
    std::thread read_can_thread(&StateController::read_can_frame, this);
    read_can_thread.detach();

    std::thread send_can_thread(&StateController::send_can_frames, this);
    send_can_thread.detach();

    std::thread send_imu_can_messages_thread(&StateController::sendImuCanMessages, this);
    send_imu_can_messages_thread.detach();

    resetMaxon();

    maxon_activation();
    
    //initialize CAN open for res and maxon 
    /*NEW!!*/
    // struct can_frame frame;

    // frame.can_dlc = 2;

    // frame.can_id = 0x00;//id for initialization
    // frame.data[0] = 0x00; 
    // frame.data[1] = 0x00; //activate maxon and RES
    // send_can_frame(frame);

    // frame.data[0]=0x01;//op mode
    // frame.data[1]=0x00;
    // send_can_frame(frame);

    rclcpp::on_shutdown([this]() {
        if (this->bag_recording){
            ::kill(this->bag_process_.id(), SIGINT); // Terminate the bag recording process
            this->bag_recording = false; // Reset the bag recording flag
        }
        resetMaxon();
    });
}

void StateController::maxon_activation(){
    while(!maxon_activated){
        RCLCPP_INFO(this->get_logger(), "maxon activation");

        struct can_frame frame;

        frame.can_dlc = 2;

        frame.can_id = 0x00;//id for initialization
        frame.data[0] = 0x00; 
        frame.data[1] = 0x00; //activate maxon and RES
        send_can_frame(frame);
        //usleep(5000); //sleep for 200ms for maxon to change modes
        RCLCPP_INFO(this->get_logger(), "maxon initiated");

        frame.data[0]=0x80;//pre op mode
        frame.data[1]=NODE_ID_STEERING;
        send_can_frame(frame);
        //usleep(5000);
        RCLCPP_INFO(this->get_logger(), "maxon in pre op mode");

        frame.data[0]=0x01;//op mode
        frame.data[1]=0x00;
        send_can_frame(frame);
        //usleep(5000);
        RCLCPP_INFO(this->get_logger(), "maxon in op mode");

        frame.can_id = 0x305;
        frame.can_dlc = 8;
        int velocity = 5000; //set the velocity to 0
        int acceleration = 7000; //set the acceleration to 0
        for (int i = 0; i < 4; ++i) {
            frame.data[i] = (velocity >> (8 * i)) & 0xFF; // Extract each byte
        }

        for (int i = 0; i < 4; ++i) {
            frame.data[4 + i] = (acceleration >> (8 * i)) & 0xFF; // Extract each byte
        }
        send_can_frame(frame);

        memset(frame.data, 0, 8); //set the velocity and acceleration to 0

        int deceleration = 2000;
        (void) deceleration;

        frame.can_dlc = 2;
        frame.can_id = 0x205;
        frame.data[0]=0x06;
        frame.data[1]=0x00;
        // for (int i = 0; i < 4; ++i) {
        //     frame.data[2 + i] = (deceleration >> (8 * i)) & 0xFF; // Extract each byte
        // }

	    //for (int i=0; i<10;i++){
            send_can_frame(frame);
            // }
        usleep(5000);
        RCLCPP_INFO(this->get_logger(), "sent 0x06 to 0x205");

        frame.can_dlc = 2;
        frame.data[0]=0x0F;
        frame.data[1]=0x00;
        //for (int i=0; i<10;i++){
            send_can_frame(frame);
        //}
        RCLCPP_INFO(this->get_logger(), "sent 0x0F to 0x205");
        usleep(70000);
    }
     RCLCPP_INFO(this->get_logger(), "maxon activated");
}

void StateController::resetMaxon(){
    struct can_frame frame;
    frame.can_id = 0x00;//id for resetMaxon();
    frame.can_dlc = 2;
    frame.data[0] = 0x81; //reset the maxon
    frame.data[1] = NODE_ID_STEERING;
    send_can_frame(frame);
}

void StateController::sendPosToMaxon(float angle){
    // std::cout<<"Sending position to maxon: "<<angle<<std::endl;
    //receives the angle and calculates the position with the offset
    float ratio = STEERING_ANGLE_TO_RATIO(angle);

    // long raw_pos= RAD_ST_ANGLE_TO_ACTUATOR_POS(angle);

    long raw_pos;

    if(this->mission.data == lart_msgs::msg::Mission::INSPECTION){
        raw_pos= RAD_ST_ANGLE_TO_ACTUATOR_POS(angle);
        RCLCPP_INFO(this->get_logger(), "Using inspection mission, raw_pos: %ld", raw_pos);
    }else{
        raw_pos= RAD_ST_TO_MAXON_POS_WITH_RATIO(angle,ratio);
    }

    if(raw_pos>MAX_ACTUATOR_POS || raw_pos<-MAX_ACTUATOR_POS){
        RCLCPP_WARN(this->get_logger(), "Position out of range: %ld", raw_pos);
        raw_pos=MAX_ACTUATOR_POS;
    }
    long pos = relative_maxon_zero + raw_pos;
    
    struct can_frame frame;

    frame.can_id = 0x205;
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    frame.data[1] = 0x00;
    this->send_can_frame(frame);
    usleep(1300);

    frame.can_id = 0x405;//pc to maxon position id
    frame.can_dlc = 6;

    frame.data[0] = 0x3F; //With 0x3F the maxon starts moving immediately to that position, does not wait to reach the previous position
    frame.data[1] = 0x00;
    for (int i = 0; i < 4; ++i) {
        frame.data[2 + i] = (pos >> (8 * i)) & 0xFF; // Extract each byte
    }
    this->send_can_frame(frame);

    //print to see the values in the array for debugging purposes
    //for (int i = 0; i < frame.can_dlc; i++) {
    //    std::cout << "0x" << std::hex << std::uppercase << static_cast<int>(frame.data[i]) << " ";
    //}
    //std::cout << std::endl;

    //std::cout<<"relative_maxon_zero: "<<std::endl;
    //std::cout<<std::dec<<relative_maxon_zero<<std::endl;
    //std::cout<<std::hex<<pos<<std::endl;//position in hex
    //std::cout<<std::dec<<pos<<std::endl;//position in encoder ticks
}

void StateController::inspectionSteeringAngleCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){//to test the maxon with the jetson
    float angle = msg->steering_angle;
    RCLCPP_INFO(this->get_logger(), "Received steering angle: %f", angle);
    uint16_t rpm = msg->rpm;

    sendPosToMaxon(msg->steering_angle);
    uint8_t rpm_array[2];
    struct can_frame frame;
    frame.can_id = CAN_TOJAL_TEST;//to be defined
    frame.can_dlc = 2;
    MAP_ENCODE_TOJAL_RPM(rpm_array, rpm);
    memcpy(frame.data,rpm_array,2);
    send_can_frame(frame);
}


void StateController::spacCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    // Handle spac callback
    if(this->state_msg.data != lart_msgs::msg::State::DRIVING){
        return;
    }
    uint8_t rpm_array[2];

    this->last_target_angle = - RAD_TO_DEG(msg->steering_angle);
    this->last_target_speed = RPM_TO_MS(msg->rpm) * 3.6;
    
    //send steering position to maxon
    sendPosToMaxon(msg->steering_angle);

    //send RPM to can
    uint16_t rpm= msg->rpm;
    struct can_frame frame;
    frame.can_id = CAN_TOJAL_TEST;//to be defined
    frame.can_dlc = 2;
    MAP_ENCODE_TOJAL_RPM(rpm_array, rpm);
    memcpy(frame.data,rpm_array,2);
    send_can_frame(frame);
}

void StateController::ekfStatsCallback(const lart_msgs::msg::SlamStats::SharedPtr msg){
    // Handle EKF stats callback
    this->last_current_cone_count = msg->cones_count_current;
    this->last_total_cone_count = msg->cones_count_all;
    this->last_lap_count = msg->lap_count;

    struct can_frame frame;
    frame.can_id = DBC_MESSAGES;
    frame.can_dlc = 8;
    frame.data[0] = this->last_target_angle * 2;
    frame.data[1] = this->last_target_speed;
    frame.data[2] = this->last_actual_angle * 2;
    frame.data[3] = this->last_actual_speed;
    frame.data[4] = this->last_lap_count;
    frame.data[5] = this->last_current_cone_count;
    frame.data[6] = this->last_total_cone_count;

    (void) frame; // TO REMOVE!!! Just because of the warning

    // this->send_can_frame(frame);
}

void StateController::missionFinishedCallback(const lart_msgs::msg::State::SharedPtr msg){
    //Handle mission finished callback from mission controller
    //print
    RCLCPP_INFO(this->get_logger(), "Mission finished callback received with state: %d", msg->data);
    {
        std::lock_guard<std::mutex> guard(this->state_mutex);
        this->state_msg.data = lart_msgs::msg::State::FINISH;
    
    }
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (this->bag_recording) {
            ::kill(this->bag_process_.id(), SIGTERM);
            this->bag_recording = false;
        }
    }).detach();
    // if (msg->data == lart_msgs::msg::State::FINISH){
        // this->mission_finished = true;


        // if(current_rpm == 0){
        //     {
        //         std::lock_guard<std::mutex> guard(this->state_mutex);
        //         this->state_msg.data = lart_msgs::msg::State::FINISH;
        //     }
        // }else{
        //     this->setEmergency();
        // }
    // }
}

void StateController::emergencyCallback(const lart_msgs::msg::State::SharedPtr msg){
    //handle emergency from pc pipeline
    if (msg->data == lart_msgs::msg::State::EMERGENCY){
        this->setEmergency();
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            if (this->bag_recording) {
                ::kill(this->bag_process_.id(), SIGTERM);
                this->bag_recording = false;
            }
        }).detach();
    }
}

void StateController::setEmergency(){
    //set the state to emergency
    {
        std::lock_guard<std::mutex> guard(this->state_mutex);
        this->state_msg.data = lart_msgs::msg::State::EMERGENCY;
    }
    // state_publisher_->publish(this->state_msg);
    // struct can_frame frame;
    // frame.can_id = CAN_AS_STATUS;
    // frame.can_dlc = 1;
    // memset(frame.data, 0, frame.can_dlc);
    // {
    //     std::lock_guard<std::mutex> guard(this->state_mutex);
    //     // MAP_ENCODE_AS_STATE(frame.data, this->state_msg.data);
    //     frame.data[0]=this->state_msg.data;
    //     this->send_can_frame(frame);
    // }
}

// Send frame with state every 200ms
void StateController::send_can_frames(){
    while(rclcpp::ok()){
        {
            this->sendState();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void StateController::sendImuCanMessages(){
    //send imu gps pose
    while (rclcpp::ok()){
        int16_t lon_accel = (int16_t) this->last_acceleration_x * 512;
        int16_t lat_accel = (int16_t) this->last_acceleration_y * 512;
        int16_t yaw_angular_velocity = (int16_t) this->last_angular_velocity_z * 128;
        struct can_frame frame;
        frame.can_id = DBC_IMU;
        frame.can_dlc = 6;
        frame.data[0] = lon_accel & 0xFF;        // Low byte
        frame.data[1] = (lon_accel >> 8) & 0xFF; // High byte
        frame.data[2] = lat_accel & 0xFF;        // Low byte  
        frame.data[3] = (lat_accel >> 8) & 0xFF; // High byte
        frame.data[4] = yaw_angular_velocity & 0xFF;        // Low byte
        frame.data[5] = (yaw_angular_velocity >> 8) & 0xFF; // High byte
        // this->send_can_frame(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void StateController::sendState(){
    struct can_frame frame;
    frame.can_id = 0x503;
    frame.can_dlc = 1;
    memset(frame.data, 0, frame.can_dlc);
    {
        std::lock_guard<std::mutex> guard(this->state_mutex);
        this->state_publisher_->publish(this->state_msg);
        // MAP_ENCODE_AS_STATE(frame.data, this->state_msg.data);
        frame.data[0]=this->state_msg.data;
        for (int i = 0; i <10 ; i++){
            this->send_can_frame(frame);
        }
    }
}

void StateController::send_can_frame(struct can_frame frame){
    std::lock_guard<std::mutex> guard(this->socket_mutex);   
    if(write(this->s, &frame, sizeof(frame)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
    }
}

// Handle CAN frame
void StateController::handle_can_frame(struct can_frame frame){
    switch (frame.can_id){
        case 0x51://mission from ACU Id
            frame.can_id = 0x61; //Mission to ACU Id
            send_can_frame(frame);
            this->mission.data =frame.data[0]; //save the mission
            break;

            /*NEW!!*/
        case 0x71:{
            uint32_t ignition_status = frame.data[0];
            if (ignition_status == 1 && !this->bag_recording){
                //starting the bag when receiving the ignition

                if (!this->relative_zero_set){
                    this->relative_maxon_zero = this->maxon_start_position; //set the relative zero to the initial position in case no message from the dynamics was received until ignition
                    this->relative_zero_set = true;
                }

                this->startRecordBagProcess();
                //this->maxon_activation(); //To be tested

                this->mission_publisher_->publish(this->mission); // send the mission to the mission controller
            }
            break;
        }

        // case IMU_TURN_RATE:{
        //     // ----- Decode gyrX -----
        //     int16_t raw_gyrX = (int16_t)((frame.data[0] << 8) | frame.data[1]);
        //     double gyrX = raw_gyrX * 0.001953125;

        //     // ----- Decode gyrY -----
        //     int16_t raw_gyrY = (int16_t)((frame.data[2] << 8) | frame.data[3]);
        //     double gyrY = raw_gyrY * 0.001953125;

        //     // ----- Decode gyrZ -----
        //     int16_t raw_gyrZ = (int16_t)((frame.data[4] << 8) | frame.data[5]);
        //     double gyrZ = raw_gyrZ * 0.001953125;

        //     this->last_angular_velocity_z = gyrZ * 57.295779513082; // Save the last angular velocity for later use

        //     geometry_msgs::msg::Vector3Stamped msg;
        //     msg.header.stamp = this->get_clock()->now();
        //     msg.header.frame_id = "base_footprint";
        //     msg.vector.x = gyrX;
        //     msg.vector.y = gyrY;
        //     msg.vector.z = gyrZ;

        //     RCLCPP_WARN(this->get_logger(), "IMU Turn Rate - X: %f, Y: %f, Z: %f", gyrX, gyrY, gyrZ);
        //     this->imu_turn_rate_publisher_->publish(msg);
        //     break;
        // }

        // case IMU_ACCELERATION:{
        //      // -------- Decode accX --------
        //     int16_t raw_accX = (int16_t)((frame.data[0] << 8) | frame.data[1]); // Big endian
        //     double accX = raw_accX * 0.00390625;

        //     this->last_acceleration_x = accX; // Save the last acceleration for later use

        //     // -------- Decode accY --------
        //     int16_t raw_accY = (int16_t)((frame.data[2] << 8) | frame.data[3]);
        //     double accY = raw_accY * 0.00390625;

        //     this->last_acceleration_y = accY; // Save the last acceleration for later use

        //     // -------- Decode accZ --------
        //     int16_t raw_accZ = (int16_t)((frame.data[4] << 8) | frame.data[5]);
        //     double accZ = raw_accZ * 0.00390625;

            

        //     break;
        // }

        // case IMU_GPS_POSE:{
        //     float raw_lat =(frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];

        //     float raw_lon = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
        
        //     geometry_msgs::msg::PoseStamped msg;

        //     msg.header.stamp = this->get_clock()->now();
        //     msg.header.frame_id = "base_footprint";

        //     // publishing Lat/Long/Altitude as x,y z
        //     msg.pose.position.x = raw_lat * 5.9604644775e-08;
        //     msg.pose.position.y = raw_lon *  1.1920928955e-07;
        //     msg.pose.position.z = 0.0;
        //     // publishing Orientation as w,x,y,z
        //     msg.pose.orientation.w = 1.0;
        //     msg.pose.orientation.x = 0.0;
        //     msg.pose.orientation.y = 0.0;
        //     msg.pose.orientation.z = 0.0;

        //     RCLCPP_WARN(this->get_logger(), "IMU GPS Pose - Lat: %f, Lon: %f", msg.pose.position.x, msg.pose.position.y);

        //     this->imu_gps_pose_publisher_->publish(msg);

        //     break;
        // }


        case CAN_TOJAL_SEND_RPM:{ //RPM from VCU to PC
            // Handle ACU RPM frame
            uint16_t rpm = MAP_DECODE_TOJAL_RPM(frame.data);
            // if (rpm >5000){
            //     rpm = this->last_valid_rpm;
            // }else {
            //     this->last_valid_rpm = rpm;
            // }
            this->last_actual_speed = RPM_TO_MS(rpm) * 3.6; //convert to km/h for dbc purposes
            lart_msgs::msg::Dynamics spac_msg;
            spac_msg.rpm = rpm;
            spac_publisher->publish(spac_msg);
            current_rpm = rpm; //save the current speed
            break;
        }
        case DINAMICS_STEERING_ID:{ //To be tested
            int16_t raw_angle = (int16_t)((frame.data[1] << 8) | frame.data[0]); // Combine the two bytes to get the signed angle
            float angle = raw_angle * 0.1f;
            RCLCPP_WARN(this->get_logger(), "Steering angle: %f", angle);
            this->last_actual_angle = angle; //save the last actual angle
            angle = DEG_TO_RAD(angle); // Convert to radians
            RCLCPP_WARN(this->get_logger(), "relative zero set: %d", relative_zero_set ? 1 : 0);
            RCLCPP_WARN(this->get_logger(), "relative maxon zero: %ld", relative_maxon_zero);
            RCLCPP_WARN(this->get_logger(), "maxon start position: %ld", maxon_start_position);
            RCLCPP_WARN(this->get_logger(), "maxon activated: %d", maxon_activated);

            if(!relative_zero_set && maxon_activated && maxon_start_position_set){
                int raw_angle_pos = RAD_SW_ANGLE_TO_ACTUATOR_POS(angle); //calculate the position of the maxon in encoder ticks
                relative_maxon_zero = maxon_start_position + raw_angle_pos; //set the relative zero to the first position of the maxon when the system is turned on
                relative_zero_set = true;
                for (int k = 0; k < 5; k++) {
                    struct can_frame frame;
                    frame.can_id = 0x205;
                    frame.can_dlc = 2;
                    frame.data[0] = 0x0F;
                    frame.data[1] = 0x00;
                    this->send_can_frame(frame);
                    usleep(1300);
                    
                    frame.can_id = 0x405;//pc to maxon position id
                    frame.can_dlc = 6;
                    
                    frame.data[0] = 0x3F; //With 0x3F the maxon starts moving immediately to that position, does not wait to reach the previous position
                    frame.data[1] = 0x00;
                    for (int i = 0; i < 4; i++) {
                        // if (k == 0){
                            frame.data[2 + i] = ((relative_maxon_zero) >> (8 * i)) & 0xFF; // Extract each byte
                        // }else{
                        //     frame.data[2 + i] = (relative_maxon_zero >> (8 * i)) & 0xFF; // Extract each byte
                        // }
                    }
                    this->send_can_frame(frame);
                    usleep(1300);
                }
            }
            break;
        }

        case PDO_TXONE(NODE_ID_STEERING):
            //maxon feedback
            statusword1 = MAP_DECODE_PDO_TXONE_MAXON_STATUSWORD(frame.data);
            mode = MAP_DECODE_PDO_TXONE_MAXON_MODES_OF_OPERATION(frame.data);
            error_code = MAP_DECODE_PDO_TXONE_MAXON_ERROR_CODE(frame.data);
            //std::cout<<"statusword: "<<statusword1<<std::endl;
            //std::cout<<"mode: "<<mode<<std::endl;
            //std::cout<<"error_code(actual current): "<<error_code<<std::endl;
            // Handle maxon feedback
            /*if(error_code==0 && maxon_activated){//error value is actually the current being pulled by the motor, when maxon is in error state, the value is 0
                RCLCPP_ERROR(this->get_logger(), "Error code: %d", error_code);
                maxon_activated = false;
                struct can_frame frame;
                frame.can_dlc = 8;
                frame.can_id = 0x00;//id for reset
                for (int i = 0; i < frame.can_dlc; i++){
                    frame.data[i] = 0;
                }
                frame.data[0] = 0x81; //reset the maxon
                frame.data[1] = 0x05;
                send_can_frame(frame);
                maxon_activation();
            }*/
    
            break;


        case PDO_TXTWO(NODE_ID_STEERING):{
            //first two bytes, little endian
            uint16_t error_code_maxon= frame.data[1] << 8 | frame.data[0];
            if(error_code_maxon != 0){
                RCLCPP_ERROR(this->get_logger(), "Maxon error code: %d", error_code_maxon);
                this->resetMaxon();
                this->maxon_activation();
            }
            break;
        }
        
        
        case 0x385:
            //maxon feedback
	        //RCLCPP_INFO(this->get_logger(), "I AM HERE, id 385");
            statusword2 = MAP_DECODE_PDO_TXTHREE_MAXON_STATUSWORD(frame.data);
            actual_position = MAP_DECODE_PDO_TXTHREE_MAXON_ACTUAL_POSITION(frame.data);
            actual_moment = MAP_DECODE_PDO_TXTHREE_MAXON_ACTUAL_MOMENT(frame.data);
            if (actual_position ==0){
                // this->setEmergency();
                // RCLCPP_ERROR(this->get_logger(), "Maxon position is 0, emergency state set");
                // return;
                break;
            }

            // if(!relative_zero_set){ //while the eletronics department does not have the steering wheel angle sensor, the relative zero is set to the first position of the maxon THE WHEELS MUST BE STRAIGHT
            //     relative_maxon_zero = actual_position;
            //     relative_zero_set = true;
            // }
            /* To test*/
            if(!maxon_start_position_set){
                maxon_start_position = actual_position; //save the start position of the maxon
                maxon_start_position_set = true;
            } //For the steering wheel angle sensor, the maxon start position is the first position of the maxon when the system is turned on


            // Handle maxon feedback
            //std::cout<<"statusword: "<<statusword2<<std::endl;
            //RCLCPP_INFO(this->get_logger(), "actual_position: ", actual_position);
            //std::cout<<"actual_moment: "<<actual_moment<<std::endl;
            break;

        //information not needed for now
        /*case PDO_TXFOUR(NODE_ID_STEERING):
            //maxon feedback
            statusword3 = MAP_DECODE_PDO_TXFOUR_MAXON_STATUSWORD(frame.data);
            actual_speed = MAP_DECODE_PDO_TXFOUR_MAXON_ACTUAL_SPEED(frame.data);
            actual_pwm_duty = MAP_DECODE_PDO_TXFOUR_MAXON_ACTUAL_PWM_DUTY(frame.data);
            // Handle maxon feedback
            std::cout<<"statusword: "<<statusword3<<std::endl;
            std::cout<<"actual_speed: "<<actual_speed<<std::endl;
            std::cout<<"actual_pwm_duty: "<<actual_pwm_duty<<std::endl;

            break;*/
        
        case RES_CAN_ID:{
            // Receive the go signal
            uint8_t res_response = frame.data[0];
            if(res_response == 0x05 || res_response == 0x07){ //received the res ready signal
                if((std::chrono::steady_clock::now() - ready_change) >= std::chrono::seconds(5) && this->state_msg.data == lart_msgs::msg::State::READY){
                    {
                        std::lock_guard<std::mutex> guard(this->state_mutex);
                        this->state_msg.data = lart_msgs::msg::State::DRIVING;
                    }
                }
            }
            if(res_response == 0x00){//received the res emergency signal
                this->setEmergency();
                std::thread([this]() {
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    if (this->bag_recording) {
                        ::kill(this->bag_process_.id(), SIGTERM);
                        this->bag_recording = false;
                    }
                }).detach();
                //resetMaxon();
            }
            break;
        }  

        case 0x513:{

            // Handle ACU state frame
            // uint32_t status = MAP_DECODE_AS_STATE(frame.data);
            //this->mission.data = MAP_DECODE_AS_MISSION(frame.data); // save the mission
            //this->mission_publisher_->publish(this->mission); // send the mission to the mission controller
            // std::cout << stateToString(this->state_msg.data) << std::endl;
            uint32_t status = frame.data[0];
            // std::cout<<status<<std::endl;
            
            if(status == lart_msgs::msg::State::READY && (this->state_msg.data != lart_msgs::msg::State::READY && this->state_msg.data != lart_msgs::msg::State::DRIVING)){
                RCLCPP_INFO(this->get_logger(), "State changed to READY");
                ready_change = std::chrono::steady_clock::now(); //save the time the state was changed to ready
                {
                    std::lock_guard<std::mutex> guard(this->state_mutex);
                    this->state_msg.data = lart_msgs::msg::State::READY;
                }
                
                // std::cout<<this->maxon_activated<<std::endl;
                
                // this->maxon_activation();

                 // Start the bag recording process
            }
            /*NEW!!*/
            if(status == lart_msgs::msg::State::OFF && this->state_msg.data == lart_msgs::msg::State::EMERGENCY){
                RCLCPP_INFO(this->get_logger(), "State changed to OFF");
                {
                    std::lock_guard<std::mutex> guard(this->state_mutex);
                    this->state_msg.data = lart_msgs::msg::State::OFF;
                }
                // this->resetMaxon();
            }
            break;
        }
        case 0x708:
            if (frame.data[0]==0x05){
                this->maxon_activated = true;
            }
            break;

    }
}


void StateController::read_can_frame(){
    while(rclcpp::ok()) {
		struct can_frame frame;
		int nbytes = read(this->s, &frame, sizeof(frame));
		if(nbytes < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to read CAN frame: %s", strerror(errno));
			return;
		}
		handle_can_frame(frame);// Send the received can frame to a function that handles it
		//RCLCPP_INFO(this->get_logger(), "RECEIVED A CAN FRAME");
	}
}

bool StateController::valid_state(lart_msgs::msg::State msg){
    return (msg.data == lart_msgs::msg::State::OFF || msg.data == lart_msgs::msg::State::READY || msg.data == lart_msgs::msg::State::DRIVING || msg.data == lart_msgs::msg::State::EMERGENCY || msg.data == lart_msgs::msg::State::FINISH);
}


void StateController::startRecordBagProcess() {
    try {
        // Get the current date
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream bag_command;
        bag_command << RECORD_BAG << BAG_DIRECTORY 
            <<"bags_" << std::setw(2) << std::setfill('0') << tm.tm_mday 
            << "_" << std::setw(2) << std::setfill('0') << tm.tm_mon + 1 
            <<"/bag_"<< std::setw(2) << std::setfill('0') << tm.tm_hour << "_"
            << std::setw(2) << std::setfill('0') << tm.tm_min << "_"
            << std::setw(2) << std::setfill('0') << tm.tm_sec  << " " 
            << BAG_TOPICS;

        // Start the process using Boost.Process
        this->bag_process_ = bp::child("/bin/bash",  "-c" ,bag_command.str());

        this->bag_recording = true; // Set the flag to true when the process starts
    } catch (const std::exception &e) {
        std::cerr << "Failed to start process: " << e.what() << std::endl;
    }
}

std::string StateController::stateToString(int state) {
    switch (state) {
      case lart_msgs::msg::State::OFF: return "OFF";
      case lart_msgs::msg::State::READY: return "READY";
      case lart_msgs::msg::State::DRIVING: return "DRIVING";
      case lart_msgs::msg::State::EMERGENCY: return "EMERGENCY";
      case lart_msgs::msg::State::FINISH: return "FINISH";
      default: return "UNKNOWN";
    }
}

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<StateController>());
        rclcpp::shutdown();


    return 0;
}
