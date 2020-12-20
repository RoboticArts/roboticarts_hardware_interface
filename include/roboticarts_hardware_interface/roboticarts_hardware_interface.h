#ifndef ROS_CONTROL_ROBOTICARTS_HARDWARE_INTERFACE_H
#define ROS_CONTROL_ROBOTICARTS_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <roboticarts_hardware_interface/roboticarts_hardware.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

using namespace hardware_interface;


namespace roboticarts_hardware_interface
{

    class MiniAtomHardwareInterface: public roboticarts_hardware_interface::MiniAtomHardware
    {
        public:
            MiniAtomHardwareInterface(ros::NodeHandle& nh);
            ~MiniAtomHardwareInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write();

        protected:
            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            PositionJointInterface positionJointInterface;
            VelocityJointInterface velocityJointInterface;
            EffortJointInterface effortJointInterface;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
            double p_error_, v_error_, e_error_;
        
        private:

            ros::Publisher motor_pub_setpoint;
            std_msgs::Float32MultiArray motor_velocity_setpoint;
            
            ros::Subscriber motor_sub_state;
            sensor_msgs::JointState motor_state;
            void motorStateCallback(const sensor_msgs::JointState::ConstPtr& msg);


    };

}

#endif

