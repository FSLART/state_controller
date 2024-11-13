#include "state_controller.hpp"

using std::placeholders::_1;

StateController::StateController() : Node("state_controller"){
    mission_finished_sub_ = this->create_subscription<lart_msgs::msg::ASStatus>("/pc_origin/system_status/critical_as", 10, std::bind(&StateController::missionFinishedCallback, this, _1));

    emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>("/pc_origin/emergency", 10, std::bind(&StateController::emergencyCallback, this, _1));//decide the topic name

    spac_sub_ = this->create_subscription<lart_msgs::msg::DynamicsCMD>("spacccccc", 10, std::bind(&StateController::spacCallback, this, _1));//change for right topic name

    state_publisher_ = this->create_publisher<lart_msgs::msg::State>("/pc_origin/system_status/critical_as/state", 10);

    mission_publisher_ = this->create_publisher<lart_msgs::msg::Mission>("/pc_origin/system_status/critical_as/mission", 10);

    inspection_steering_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>("steering_cmd", 10, std::bind(&StateController::inspectionSteeringAngleCallback, this, _1));

    state_msg.data = lart_msgs::msg::State::OFF; // initialize state as off
    
    mission_finished = false;

    res_ready = false;

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
        return;
    }


    // create a thread to read CAN frames
	std::thread read_can_thread(&StateController::read_can_frame, this);
	read_can_thread.detach();

    std::thread send_can_thread(&StateController::send_can_frames, this);
    send_can_thread.detach();

    maxon_activation();

    
    
}

void StateController::maxon_activation(){
    std::cout<<"maxon activation"<<std::endl << std::flush;

    struct can_frame frame;
    frame.can_id = 0;
    frame.can_dlc = 8;

    frame.data[0] = 0x00; //turn on the maxon
    frame.data[1] = 0x05;
    send_can_frame(frame);
    

    frame.data[0]=0x80;//pre op mode
    frame.data[1]=0x05;
    send_can_frame(frame);
    

    frame.data[0]=0x01;//op mode
    frame.data[1]=0x05;
    send_can_frame(frame);

    frame.can_id = 0x205;
    frame.data[0]=0x06;
    frame.data[1]=0x05;
    send_can_frame(frame);

    frame.data[0]=0x0F;
    frame.data[1]=0x05;
    send_can_frame(frame);

}

void StateController::inspectionSteeringAngleCallback(const std_msgs::msg::Float64::SharedPtr msg){//to test the maxon with the jetson
    // Handle inspection steering angle callback
    float angle = msg->data;

    struct can_frame frame;
    frame.can_id = 0x405;
    frame.can_dlc = 8;
    MAP_ENCODE_AS_STEERING_ANGLE_ACTUAL(frame.data, angle);
    this->send_can_frame(frame);
    std::cout<<"angle: "<<angle<<std::endl;
}

void StateController::spacCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    struct can_frame frame;
    frame.can_id = CAN_AS_DYNAMICS_ONE;
    frame.can_dlc = 8;
    

    // Handle spac callback
}

void StateController::missionFinishedCallback(const lart_msgs::msg::ASStatus::SharedPtr msg){
    // Handle mission finished callback from mission controller
}

void StateController::emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg){
    //hand emergency from pc pipeline
}

// Send frame
void StateController::send_can_frames(){
    struct can_frame frame;
    frame.can_id = CAN_AS_STATUS;
    frame.can_dlc = 8;
    while(rclcpp::ok()){
        {
            std::lock_guard<std::mutex> guard(this->state_mutex);
            if(res_ready && std::chrono::steady_clock::now() - ready_change > std::chrono::seconds(5)&& state_msg.data ==lart_msgs::msg::State::READY){
                this->state_msg.data = lart_msgs::msg::State::DRIVING;
                this->state_publisher_->publish(this->state_msg);
                MAP_ENCODE_AS_STATE(frame.data, state_msg.data);
                this->send_can_frame(frame);
            }else{
                this->res_ready = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void StateController::send_can_frame(struct can_frame frame){
    if(write(this->s, &frame, sizeof(frame)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
    }
}

// Handle CAN frame
void StateController::handle_can_frame(struct can_frame frame){
    switch (frame.can_id){
        case (0x185):
            //maxon feedback
            {uint32_t statusword1 = MAP_DECODE_PDO_TXONE_STATUSWORD(frame.data);
            uint32_t mode = MAP_DECODE_PDO_TXONE_MODES_OF_OPERATION(frame.data);
            uint32_t error_code = MAP_DECODE_PDO_TXONE_ERROR_CODE(frame.data);
            // Handle maxon feedback
            std::cout<<"statusword: "<<statusword1<<std::endl;
            std::cout<<"mode: "<<mode<<std::endl;
            std::cout<<"error_code: "<<error_code<<std::endl;}

            break;
        case (0x285):
            //maxon feedback
            {uint32_t target_position = MAP_DECODE_PDO_TXTWO_TARGET_POSITION(frame.data);
            uint32_t target_speed = MAP_DECODE_PDO_TXTWO_TARGET_SPEED(frame.data);
            // Handle maxon feedback
            std::cout<<"target_position: "<<target_position<<std::endl;
            std::cout<<"target_speed: "<<target_speed<<std::endl;}

            break;
        case (0x385):
            //maxon feedback
            {uint32_t statusword2 = MAP_DECODE_PDO_TXTHREE_STATUSWORD(frame.data);
            uint32_t actual_position = MAP_DECODE_PDO_TXTHREE_ACTUAL_POSITION(frame.data);
            uint32_t actual_moment = MAP_DECODE_PDO_TXTHREE_ACTUAL_MOMENT(frame.data);
            // Handle maxon feedback
            std::cout<<"statusword: "<<statusword2<<std::endl;
            std::cout<<"actual_position: "<<actual_position<<std::endl;
            std::cout<<"actual_moment: "<<actual_moment<<std::endl;}
            break;
        case (0x485):
            //maxon feedback
            {uint32_t statusword3 = MAP_DECODE_PDO_TXFOUR_STATUSWORD(frame.data);
            uint32_t actual_speed = MAP_DECODE_PDO_TXFOUR_ACTUAL_SPEED(frame.data);
            uint32_t actual_pwm_duty = MAP_DECODE_PDO_TXFOUR_ACTUAL_PWM_DUTY(frame.data);
            // Handle maxon feedback
            std::cout<<"statusword: "<<statusword3<<std::endl;
            std::cout<<"actual_speed: "<<actual_speed<<std::endl;
            std::cout<<"actual_pwm_duty: "<<actual_pwm_duty<<std::endl;}

            break;
        case CAN_AS_STATUS:
            {uint32_t status = MAP_DECODE_AS_STATE(frame.data);
            uint32_t mission = MAP_DECODE_AS_MISSION(frame.data);//publish mission to computer

            if(status == lart_msgs::msg::State::READY && state_msg.data != lart_msgs::msg::State::READY){
                ready_change = std::chrono::steady_clock::now(); //save the time the state was changed to ready
                this->state_msg.data = lart_msgs::msg::State::READY;
            }}
            // Handle ACU state frame
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
