#include "state_controller.hpp"

using std::placeholders::_1;

StateController::StateController() : Node("state_controller"){
    mission_finished_sub_ = this->create_subscription<lart_msgs::msg::State>("/pc_origin/system_status/critical_as", 10, std::bind(&StateController::missionFinishedCallback, this, _1));

    emergency_sub_ = this->create_subscription<lart_msgs::msg::State>("/pc_origin/emergency", 10, std::bind(&StateController::emergencyCallback, this, _1));//decide the topic name

    spac_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/pc_origin/dynamics", 10, std::bind(&StateController::spacCallback, this, _1));//change for right topic name

    spac_publisher = this->create_publisher<lart_msgs::msg::Dynamics>("/acu_origin/dynamics", 10);
    
    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10);

    mission_publisher_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);

    inspection_steering_angle_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/cmd", 10, std::bind(&StateController::inspectionSteeringAngleCallback, this, _1));

    state_msg.data = lart_msgs::msg::State::OFF; // initialize state as off
    
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
    //activate the actuator
    resetMaxon();
    usleep(1000000);
    maxon_activation();

    // create a thread to read CAN frames
	std::thread read_can_thread(&StateController::read_can_frame, this);
	read_can_thread.detach();

    std::thread send_can_thread(&StateController::send_can_frames, this);
    send_can_thread.detach();

}

void StateController::maxon_activation(){
    while(!maxon_activated){
        std::cout<<"maxon activation"<<std::endl << std::flush;

        struct can_frame frame;

        frame.can_dlc = 2;

        frame.can_id = 0x00;//id for initialization
        frame.data[0] = 0x00; //turn on the maxon
        frame.data[1] = 0x05;
        send_can_frame(frame);
        usleep(1000000); //sleep for 200ms for maxon to change modes
        std::cout<<"maxon initiated"<<std::endl << std::flush;

        frame.data[0]=0x80;//pre op mode
        frame.data[1]=0x05;
        send_can_frame(frame);
        usleep(1000000);
        std::cout<<"maxon in pre op mode"<<std::endl << std::flush;

        frame.data[0]=0x01;//op mode
        frame.data[1]=0x05;
        send_can_frame(frame);
        usleep(1000000);
        std::cout<<"maxon in op mode"<<std::endl << std::flush;

        frame.can_id = 0x205;
        frame.data[0]=0x06;
        frame.data[1]=0x00;
        send_can_frame(frame);
        usleep(1000000);
        std::cout<<"sent 0x06 to 0x205"<<std::endl << std::flush;


        frame.data[0]=0x0F;
        frame.data[1]=0x00;
        send_can_frame(frame);
        std::cout<<"sent 0x0F to 0x205"<<std::endl << std::flush;
    }
    std::cout<<"maxon activated"<<std::endl << std::flush;
}

void StateController::resetMaxon(){
    struct can_frame frame;
    frame.can_id = 0x00;//id for reset
    frame.can_dlc = 2;
    frame.data[0] = 0x81; //reset the maxon
    frame.data[1] = 0x05;
    send_can_frame(frame);
}

void StateController::inspectionSteeringAngleCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){//to test the maxon with the jetson
    // Handle inspection steering angle callback
    float angle = msg->steering_angle;
    std::cout<<"angle: "<<angle<<std::endl;
    uint16_t rpm = msg->rpm;

    sendPosToMaxon(angle);
    std::cout<<"RPM: "<<rpm<<std::endl;
    /*struct can_frame frame;
    frame.can_id = ACU_RPM_ID;//to be defined
    frame.can_dlc = 2;
    frame.data[0] = rpm & 0xFF;
    frame.data[1] = (rpm >> 8) & 0xFF;
    send_can_frame(frame);*/
}

void StateController::sendPosToMaxon(float angle){
    //receives the angle and calculates the position with the offset
    long raw_pos= RAD_ST_ANGLE_TO_ACTUATOR_POS(angle);
    if(raw_pos>MAX_ACTUATOR_POS || raw_pos<-MAX_ACTUATOR_POS){
        RCLCPP_ERROR(this->get_logger(), "Position out of range: %ld", raw_pos);
        return;
    }
    long pos = relative_maxon_zero + raw_pos;
    
    struct can_frame frame;
    frame.can_id = 0x205;
    frame.can_dlc = 2;
    frame.data[0] = 0x0F;
    this->send_can_frame(frame);
    usleep(1000);

    frame.can_id = 0x405;//pc to maxon position id
    frame.can_dlc = 6;

    frame.data[0] = 0x3F;
    frame.data[1] = 0x00;
    for (int i = 0; i < 4; ++i) {
        frame.data[2 + i] = (pos >> (8 * i)) & 0xFF; // Extract each byte
    }
    this->send_can_frame(frame);

    //print to see the values in the array for debugging purposes
    for (int i = 0; i < frame.can_dlc; i++) {
        std::cout << "0x" << std::hex << std::uppercase << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::endl;

    std::cout<<"relative_maxon_zero: "<<std::endl;
    std::cout<<std::dec<<relative_maxon_zero<<std::endl;
    std::cout<<std::hex<<pos<<std::endl;//position in hex
    std::cout<<std::dec<<pos<<std::endl;//position in encoder ticks
}

void StateController::spacCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    // Handle spac callback
    
    //send steering position to maxon
    sendPosToMaxon(msg->steering_angle);

    //send RPM to can
    uint16_t rpm= msg->rpm;
    struct can_frame frame;
    frame.can_id = ACU_RPM_ID;//to be defined
    frame.can_dlc = 2;
    frame.data[0] = rpm & 0xFF;
    frame.data[1] = (rpm >> 8) & 0xFF;
    send_can_frame(frame);
}

void StateController::missionFinishedCallback(const lart_msgs::msg::State::SharedPtr msg){
    //Handle mission finished callback from mission controller
    if (msg->data == lart_msgs::msg::State::FINISH){
        this->mission_finished = true;
    }
    else{
        this->mission_finished = false;
    }
}

void StateController::emergencyCallback(const lart_msgs::msg::State::SharedPtr msg){
    //handle emergency from pc pipeline
    if (msg->data == lart_msgs::msg::State::EMERGENCY){
        {
            std::lock_guard<std::mutex> guard(this->state_mutex);
            this->state_msg.data = lart_msgs::msg::State::EMERGENCY;
        }
        state_publisher_->publish(this->state_msg);
        struct can_frame frame;
        (void) frame;//to be removed
    }
}

// Send frame with state every 200ms
void StateController::send_can_frames(){
    struct can_frame frame;
    frame.can_id = CAN_AS_STATUS;
    frame.can_dlc = 8;
    for (int i = 0; i < frame.can_dlc; i++){
        frame.data[i] = 0;
    }
    while(rclcpp::ok()){
        {
            std::lock_guard<std::mutex> guard(this->state_mutex);
            if(res_ready && (std::chrono::steady_clock::now() - ready_change) > std::chrono::seconds(5) && state_msg.data ==lart_msgs::msg::State::READY){
                this->state_msg.data = lart_msgs::msg::State::DRIVING;
                this->state_publisher_->publish(this->state_msg);
                MAP_ENCODE_AS_STATE(frame.data, state_msg.data);
                this->send_can_frame(frame);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
        case ACU_RPM_ID:{
            // Handle ACU RPM frame
            uint16_t rpm = frame.data[0] | (frame.data[1] << 8);
            lart_msgs::msg::Dynamics spac_msg;
            spac_msg.rpm = rpm;
            spac_publisher->publish(spac_msg);
            break;
        }

        case PDO_TXONE_MAXON():
            //maxon feedback
            statusword1 = MAP_DECODE_PDO_TXONE_STATUSWORD(frame.data);
            mode = MAP_DECODE_PDO_TXONE_MODES_OF_OPERATION(frame.data);
            error_code = MAP_DECODE_PDO_TXONE_ERROR_CODE(frame.data);
            //std::cout<<"statusword: "<<statusword1<<std::endl;
            //std::cout<<"mode: "<<mode<<std::endl;
            std::cout<<"error_code(actual current): "<<error_code<<std::endl;
            // Handle maxon feedback
            if(error_code==0 && maxon_activated){//error value is actually the current being pulled by the motor, when maxon is in error state, the value is 0
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
            }
    
            break;
        //not being used
        /*case PDO_TXTWO_MAXON():
            //maxon feedback
            target_position = MAP_DECODE_PDO_TXTWO_TARGET_POSITION(frame.data);
            target_speed = MAP_DECODE_PDO_TXTWO_TARGET_SPEED(frame.data);
            // Handle maxon feedback
            std::cout<<"target_position: "<<target_position<<std::endl;
            std::cout<<"target_speed: "<<target_speed<<std::endl;

            break;*/
        
        
        case PDO_TXTHREE_MAXON():
            //maxon feedback
            statusword2 = MAP_DECODE_PDO_TXTHREE_STATUSWORD(frame.data);
            actual_position = MAP_DECODE_PDO_TXTHREE_ACTUAL_POSITION(frame.data);
            maxon_activated = true;
            actual_moment = MAP_DECODE_PDO_TXTHREE_ACTUAL_MOMENT(frame.data);
            if(!relative_zero_set){
                relative_maxon_zero = actual_position;
                relative_zero_set = true;
            }
            // Handle maxon feedback
            //std::cout<<"statusword: "<<statusword2<<std::endl;
            std::cout<<"actual_position: "<<actual_position<<std::endl;
            //std::cout<<"actual_moment: "<<actual_moment<<std::endl;
            break;
        //information not needed for now
        /*case PDO_TXFOUR_MAXON():
            //maxon feedback
            statusword3 = MAP_DECODE_PDO_TXFOUR_STATUSWORD(frame.data);
            actual_speed = MAP_DECODE_PDO_TXFOUR_ACTUAL_SPEED(frame.data);
            actual_pwm_duty = MAP_DECODE_PDO_TXFOUR_ACTUAL_PWM_DUTY(frame.data);
            // Handle maxon feedback
            std::cout<<"statusword: "<<statusword3<<std::endl;
            std::cout<<"actual_speed: "<<actual_speed<<std::endl;
            std::cout<<"actual_pwm_duty: "<<actual_pwm_duty<<std::endl;

            break;*/
        case CAN_AS_STATUS:
            // Handle ACU state frame
            uint32_t status = MAP_DECODE_AS_STATE(frame.data);
            uint32_t mission = MAP_DECODE_AS_MISSION(frame.data);//publish mission to computer
            (void) mission;//void to be removed

            if(status == lart_msgs::msg::State::READY && state_msg.data != lart_msgs::msg::State::READY){
                ready_change = std::chrono::steady_clock::now(); //save the time the state was changed to ready
                {
                    std::lock_guard<std::mutex> guard(this->state_mutex);
                    this->state_msg.data = lart_msgs::msg::State::READY;

                }
            }
            break;
        /*case RES_READY_CAN_ID:
            // Handle car ready frame, not to implement yet, test first without verifications
            break;*/
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
		handle_can_frame(frame);    
	}
}

bool StateController::valid_state(lart_msgs::msg::State msg){
    return (msg.data == lart_msgs::msg::State::OFF || msg.data == lart_msgs::msg::State::READY || msg.data == lart_msgs::msg::State::DRIVING || msg.data == lart_msgs::msg::State::EMERGENCY || msg.data == lart_msgs::msg::State::FINISH);
}

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<StateController>());
        rclcpp::shutdown();


    return 0;
}
