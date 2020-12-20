#include <sstream>
#include <roboticarts_hardware_interface/roboticarts_hardware_interface.h>


using namespace hardware_interface;


namespace roboticarts_hardware_interface
{
    MiniAtomHardwareInterface::MiniAtomHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &MiniAtomHardwareInterface::update, this);
    
    }

    MiniAtomHardwareInterface::~MiniAtomHardwareInterface() {

    }

    void MiniAtomHardwareInterface::init() {
        // Get joint names
        nh_.getParam("hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {


             // Create joint state interface: used for read()
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             joint_state_interface_.registerHandle(jointStateHandle);

            //  Create position joint interface: used for write()
            JointHandle jointPositiontHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositiontHandle);

            //  Create velocity joint interface: used for write()
            JointHandle jointVelocitytHandle(jointStateHandle, &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(jointVelocitytHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&velocity_joint_interface_);

        // Init real hardware 
        motor_velocity_setpoint.data.resize(num_joints_);
        motor_pub_setpoint = nh_.advertise<std_msgs::Float32MultiArray>("teensy_motor_control/velocity_setpoint", 1000);

        motor_state.position.resize(num_joints_);
        motor_state.velocity.resize(num_joints_);
        motor_state.effort.resize(num_joints_);
        motor_sub_state = nh_.subscribe("teensy_motor_control/motor_state", 1, &MiniAtomHardwareInterface::motorStateCallback, this);
    }

    void MiniAtomHardwareInterface::motorStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

        motor_state.position[0] = msg->position[0];
        motor_state.position[1] = msg->position[1];
        motor_state.position[2] = msg->position[2];
        motor_state.position[3] = msg->position[3];
        
        motor_state.velocity[0] = msg->velocity[0];
        motor_state.velocity[1] = msg->velocity[1];
        motor_state.velocity[2] = msg->velocity[2];
        motor_state.velocity[3] = msg->velocity[3];

    }


    void MiniAtomHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write();
    }

    void MiniAtomHardwareInterface::read() {

        joint_position_[0] = motor_state.position[0];
        joint_position_[1] = motor_state.position[1];
        joint_position_[2] = motor_state.position[2];
        joint_position_[3] = motor_state.position[3];

        joint_velocity_[0] = motor_state.velocity[0];
        joint_velocity_[1] = motor_state.velocity[1];
        joint_velocity_[2] = motor_state.velocity[2];
        joint_velocity_[3] = motor_state.velocity[3];

    }

    void MiniAtomHardwareInterface::write() {

        motor_velocity_setpoint.data[0] = joint_velocity_command_[0];
        motor_velocity_setpoint.data[1] = joint_velocity_command_[1];
        motor_velocity_setpoint.data[2] = joint_velocity_command_[2];
        motor_velocity_setpoint.data[3] = joint_velocity_command_[3];

        motor_pub_setpoint.publish(motor_velocity_setpoint);
    }
}