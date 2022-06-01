#include "swift_uarm/SQNCR.h"
#include "swift_uarm/utils.h"
#include <iostream>
#include <chrono>
#include <math.h> 

#include <geometry_msgs/PoseWithCovarianceStamped.h> //robot pose from slam
#include <geometry_msgs/PoseStamped.h> //part to pick
#include <std_msgs/Float64MultiArray.h> // joints state

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



using namespace std;

/* @brief Constructor */
SQNCR::SQNCR(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters  */

    // pose topic
    private_nh.param<std::string>("joint_vel_pub_topic", _joint_vel_pub_topic, "NONE");
    private_nh.param<std::string>("joint_pose_pub_topic", _joint_pose_pub_topic, "NONE");
    private_nh.param<std::string>("vacuum_state_topic", _vacuum_state_topic, "NONE");
    private_nh.param<std::string>("odom_pub_topic", _odom_pub_topic, "NONE");
    private_nh.param<string>("base_vel_pub_topic", _base_vel_pub_topic, "NONE");
    
    // subscribers params
    private_nh.param<std::string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<std::string>("odom_sub_topic", _odom_sub_topic, "NONE");
    private_nh.param<std::string>("joint_state_sub_topic", _joint_state_sub_topic, "NONE");
    private_nh.param<std::string>("ee_target_pose_pub_sub_topic", _ee_target_pose_sub_topic, "NONE");
    private_nh.param<std::string>("ee_pln_topic", _ee_pln_topic, "NONE");
    private_nh.param<std::string>("ee_pose_err_pub_sub_topic", __ee_pose_err_sub_topic, "NONE");
    private_nh.param<string>("joint_err_pub_sub_topic", _joint_err_topic, "NONE");
    
    // frames
    private_nh.param<std::string>("frame_id", _frame_id, "NONE");
    private_nh.param<std::string>("arm_frame_id", _arm_frame_id, "NONE");
    private_nh.param<std::string>("base_frame_id", _base_frame_id, "NONE");

    // constants
    private_nh.param<float>("gain", _K, 10.0);
    private_nh.param<float>("x", _X, 0.0);
    private_nh.param<float>("y", _Y, 0.0);
    private_nh.param<float>("z", _Z, 0.0);
    private_nh.param<float>("damping", _damping, 0.01);
    private_nh.param<bool>("mobile_base", _is_mobile_base, 1);
    private_nh.param<float>("dist_err_threshold", _dist_err_threshold, 1);
    private_nh.param<float>("seq_freq", _seq_freq, 1);
    private_nh.param<string>("aruco_pose_topic", _aruco_pose_topic, "NONE");
    private_nh.param<bool>("slam", _is_slam, 0);

    // subscribers
    ee_err_sub = node.subscribe(__ee_pose_err_sub_topic, 1, &SQNCR::eeposeerrCallback, this);
    vacuum_sub = node.subscribe(_vacuum_state_topic, 1, &SQNCR::vacuumCallback, this);
    joint_err_sub = node.subscribe(_joint_err_topic, 1, &SQNCR::jointerrCallback, this);
    aruco_pose_sub = node.subscribe(_aruco_pose_topic, 1, &SQNCR::arucoPoseCallback, this);
    
    
    // publishers
    joints_pose_pub = node.advertise<std_msgs::Float64MultiArray>(_joint_pose_pub_topic, 1);
    goal_pub_ctrl = node.advertise<geometry_msgs::PoseStamped>(_ee_target_pose_sub_topic, 1);
    goal_pub_pln = node.advertise<geometry_msgs::PoseStamped>(_ee_pln_topic, 1);
    base_velocity_pub = node.advertise<geometry_msgs::Twist>(_base_vel_pub_topic, 1);
    sequencer = node.createTimer(ros::Duration(1/_seq_freq), &SQNCR::taskSequencer, this); // timer for task sequencer

    //initialising values
     is_aruco_pose = false;


    _is_vacuum_gripper = false; _is_aruco_picked = false; _switched_joint = false; _moving_z = false; _is_ee_error = false; 
    _aruco_placing = false; _aruco_placed = false; _is_j_error = false; _is_done = false; 
    ee_error.setZero(); joint_error.setZero();
}


void SQNCR::arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    if (!is_aruco_pose) {
        aruco_pose << msg->pose.position.x, msg->pose.position.y, 0.155;
        ROS_INFO("aruco_pose: %f %f %f", aruco_pose(0), aruco_pose(1), aruco_pose(2));
    }
    is_aruco_pose = true;

}

/*EE error callback*/
void SQNCR::eeposeerrCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    // check data size

    ee_error(0) = msg->data[0];
    ee_error(1) = msg->data[1];
    ee_error(2) = msg->data[2];
    _is_ee_error = true;
   
    ROS_INFO("ee_error: %f %f %f %f", ee_error.norm(), ee_error(0), ee_error(1), ee_error(2));


}

void SQNCR::vacuumCallback(const std_msgs::Bool::ConstPtr& msg)
{   
    // check data size

    _is_vacuum_gripper = msg->data;
   
    ROS_INFO("vacuum state: %d",msg->data);


}

/*joint error callback*/
void SQNCR::jointerrCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    // check data size
    
    joint_error(0) = msg->data[0];
    joint_error(1) = msg->data[1];
    joint_error(2) = msg->data[2];
    _is_j_error = true;

    ROS_INFO("joint_error: %f %f %f %f", joint_error.norm(), joint_error(0), joint_error(1), joint_error(2));


}


/* @brief task sequencer */
void SQNCR::taskSequencer(const ros::TimerEvent& event)
{
    std_msgs::Float64MultiArray joint_position; // joint position for placing on the back
    geometry_msgs::PoseStamped pose_msg; //pose message
    geometry_msgs::Twist base_velocity;

    double err = ee_error.norm();


    if ((is_aruco_pose && !_is_ee_error) || (is_aruco_pose && err > _dist_err_threshold && err <= 1.5  && !_is_aruco_picked  && !_moving_z && !_switched_joint)){
        //move near aruco
        ROS_INFO("moving to aruco sequencer");
        pose_msg.header.stamp = ros::Time::now();
        double x,y; x = aruco_pose(0); y = aruco_pose(1);
        pose_msg.pose.position.x = aruco_pose(0);
        pose_msg.pose.position.y = aruco_pose(1);
        pose_msg.pose.position.z = 0.14;

        goal_pub_ctrl.publish(pose_msg);
        _is_aruco_picked = false;
        // cout<<"republishing"<<endl;

    }
    // if aruco is too far use motion planning
    else if (_is_ee_error && err > 1.5 && !_is_aruco_picked && !_moving_z && !_switched_joint){
        //move near aruco
        ROS_INFO("moving to aruco using motion planning");
        pose_msg.header.stamp = ros::Time::now();
        double x,y; x = aruco_pose(0); y = aruco_pose(1);
        pose_msg.pose.position.x = aruco_pose(0)-0.5;
        pose_msg.pose.position.y = aruco_pose(1)-0.5;
        pose_msg.pose.position.z = 0.14;

        goal_pub_pln.publish(pose_msg);
        _is_aruco_picked = false;
        // cout<<"republishing"<<endl;

    }
    else if(is_aruco_pose && _is_ee_error && err <= _dist_err_threshold && !_is_aruco_picked && !_switched_joint) {
        //move to aruco
        ROS_INFO("moving to aruco Z");

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -100;
        pose_msg.pose.position.y = -100;
        pose_msg.pose.position.z = 0.138;//+aruco_pose(2);

        goal_pub_ctrl.publish(pose_msg);
        _moving_z = true;
        _is_aruco_picked = true;

    }else if(is_aruco_pose && _is_ee_error && err <= _dist_err_threshold && _is_aruco_picked && !_switched_joint) {
        //to activate vacuum 
        ROS_INFO("picking aruco");

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -100;
        pose_msg.pose.position.y = -100;
        pose_msg.pose.position.z = -100;

        goal_pub_ctrl.publish(pose_msg);
        _switched_joint = true;

    }else if(is_aruco_pose && _is_j_error && joint_error.norm() >= 0.25 && _is_aruco_picked && !_aruco_placing && _switched_joint) {
        //moving to safe position
        ROS_INFO("picking aruco upward");

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -100;
        pose_msg.pose.position.y = -100;
        pose_msg.pose.position.z = -100;
        goal_pub_ctrl.publish(pose_msg);

        joint_position.data.resize(4);
        joint_position.data[0] = 0;
        joint_position.data[1] = 0;
        joint_position.data[2] = 0;
        joint_position.data[3] = 0;
        joints_pose_pub.publish(joint_position);
        // _aruco_placing = true;

    }else if(is_aruco_pose && _is_j_error && joint_error.norm() <= 0.25 && _is_aruco_picked  && !_aruco_placing && _switched_joint) {
        ROS_INFO("placing aruco on back 0");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -200;
        pose_msg.pose.position.y = -200;
        pose_msg.pose.position.z = -200;//+aruco_pose(2);
        goal_pub_ctrl.publish(pose_msg);

        joint_position.data.resize(4);
        joint_position.data[0] = 1.445;
        joint_position.data[1] = -0.556;
        joint_position.data[2] = 0;
        joint_position.data[3] = 0;
        joints_pose_pub.publish(joint_position);

        _is_aruco_picked = true;
        _aruco_placing = true;
        
    }else if(is_aruco_pose &&  _is_j_error && joint_error.norm() >= 0.25 && _aruco_placing && _switched_joint) {
        ROS_INFO("placing aruco on back 1");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -200;
        pose_msg.pose.position.y = -200;
        pose_msg.pose.position.z = -200;//+aruco_pose(2);
        goal_pub_ctrl.publish(pose_msg);

        joint_position.data.resize(4);
        joint_position.data[0] = 1.445;
        joint_position.data[1] = -0.556;
        joint_position.data[2] = 0;
        joint_position.data[3] = 0;
        joints_pose_pub.publish(joint_position);

        _is_aruco_picked = true;
        
    }else if(is_aruco_pose &&  _is_j_error && joint_error.norm() >= 0.25 && _aruco_placing && _switched_joint) {
        ROS_INFO("placing aruco on back 2");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -200;
        pose_msg.pose.position.y = -200;
        pose_msg.pose.position.z = -200;//+aruco_pose(2);
        goal_pub_ctrl.publish(pose_msg);

        joint_position.data.resize(4);
        joint_position.data[0] = 1.445;
        joint_position.data[1] = -0.556;
        joint_position.data[2] = 0;
        joint_position.data[3] = 0;
        joints_pose_pub.publish(joint_position);

        _is_aruco_picked = true;
        
    }else if(is_aruco_pose && _is_j_error  && joint_error.norm() <= 0.25 && !_aruco_placed && _switched_joint) {
        ROS_INFO("turning off vacuum");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -300;
        pose_msg.pose.position.y = -300;
        pose_msg.pose.position.z = -300;//+aruco_pose(2);
        goal_pub_ctrl.publish(pose_msg);
        if(!_is_vacuum_gripper){
        _aruco_placed = true;
        is_aruco_pose = false;}
        ros::Duration(0.5).sleep();


        
    } else if(!_is_vacuum_gripper && _aruco_placed && !_is_done) {
        ROS_INFO("stoping arm");
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = -400;
        pose_msg.pose.position.y = -400;
        pose_msg.pose.position.z = -400;//+aruco_pose(2);
        goal_pub_ctrl.publish(pose_msg);
        joint_position.data.resize(4);
        joint_position.data[0] = 0;
        joint_position.data[1] = 0;
        joint_position.data[2] = 0;
        joint_position.data[3] = 0;
        joints_pose_pub.publish(joint_position);
        _is_done = true;
        
    }    
      
    
}
