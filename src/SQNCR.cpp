#include "swift_uarm/SQNCR.h"
#include "swift_uarm/utils.h"
#include "swift_uarm/TASK.h"
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
    private_nh.param<std::string>("odom_pub_topic", _odom_pub_topic, "NONE");
    
    // subscribers params
    private_nh.param<std::string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<std::string>("joint_state_sub_topic", _joint_state_sub_topic, "NONE");
    private_nh.param<std::string>("ee_target_pose_pub_sub_topic", _ee_target_pose_sub_topic, "NONE");
    private_nh.param<std::string>("ee_pose_err_pub_sub_topic", __ee_pose_err_sub_topic, "NONE");
    
    // frames
    private_nh.param<std::string>("frame_id", _frame_id, "NONE");
    private_nh.param<std::string>("arm_frame_id", _arm_frame_id, "NONE");
    private_nh.param<std::string>("base_frame_id", _base_frame_id, "NONE");

    // constants
    private_nh.param<float>("link_1", _link_1, 0.0);
    private_nh.param<float>("link_2", _link_2, 0.0);
    private_nh.param<float>("base_offset_x", _base_offset_x, 0.0);
    private_nh.param<float>("base_offset_z", _base_offset_z, 0.0);
    private_nh.param<float>("vacuum_offset_x", _vacuum_offset_x, 0.0);
    private_nh.param<float>("vacuum_offset_z", _vacuum_offset_z, 0.0);
    private_nh.param<float>("gain", _K, 10.0);
    private_nh.param<float>("x", _X, 0.0);
    private_nh.param<float>("y", _Y, 0.0);
    private_nh.param<float>("z", _Z, 0.0);
    private_nh.param<float>("damping", _damping, 0.01);
    private_nh.param<bool>("mobile_base", _is_mobile_base, 1);
    private_nh.param<float>("dist_err_threshold", _dist_err_threshold, 1);

    // subscribers
    // pose_sub = node.subscribe(_pose_sub_topic, 1, &SQNCR::poseCallback, this);
    ee_sub = node.subscribe(__ee_pose_err_sub_topic, 3, &SQNCR::eeposeCallback, this);
    
    
    // publishers
    joints_vel_pub = node.advertise<std_msgs::Float64MultiArray>(_joint_vel_pub_topic, 1);
    goal_pub = node.advertise<geometry_msgs::PoseStamped>(_ee_target_pose_sub_topic, 1);
    sequencer = node.createTimer(ros::Duration(0.2), &SQNCR::taskSequencer, this); // timer for task sequencer

    //initialising values

    // _prev_quat = Eigen::Quaternion<float>::Zero();
    // _prev_trans = Eigen::Matrix<float>::Zero();
    is_pose_start = true;

    // Initialization transformation listener
    // mTfBuffer.reset(new tf2_ros::Buffer);
    // mTfListener.reset(new tf2_ros::TransformListener(*mTfBuffer));
    robot_pose.setIdentity();//initialize robot pose


    // link lengths in meters
    // _link_1 = 0.456;   _link_2 = 0.142;  _link_3 = 0.;  _link_4 = 0.142; _link_5 = 0.1588 +0.56;
    joint_values.setZero(4);

    r2b <<  0, -1, 0, 0.037,
            1, 0, 0, 0,
            0, 0, 1, 0.147,
            0, 0, 0, 1;
    ee_pose.setZero(); ee_target = {_X, _Y, _Z};
    is_joints_read = false; _is_vacuum_gripper = false; _is_joint_vel_stopped = false;
    ee_error = 10; 
}

/* @brief base_link to arm transformation */
bool SQNCR::getArm2BaseTransform()
{
    ROS_INFO("Getting static TF from '%s' to '%s'", _arm_frame_id.c_str(), _base_frame_id.c_str());

    mArm2BaseTransfValid = false;
    static bool first_error = true;

    // ----> Static transforms
    // Sensor to Base link
    try {
        // Save the transformation
        ROS_INFO("Getting static TF from '%s' to '%s'", _arm_frame_id.c_str(), _base_frame_id.c_str());
        geometry_msgs::TransformStamped a2b = mTfBuffer->lookupTransform(_base_frame_id, _arm_frame_id, ros::Time(0), ros::Duration(0.1));

        // Get the TF2 transformation
        tf2::fromMsg(a2b.transform, mArm2BaseTransf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mArm2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);
        ROS_INFO("%.3f", roll);
    } catch (tf2::TransformException& ex) {
        ROS_INFO("%s", ex.what());
        if (!first_error) {
            first_error = false;
        }

        mArm2BaseTransf.setIdentity();
        return false;
    }
    // <---- Static transforms
    mArm2BaseTransfValid = true;
    return true;
}

/* @brief pose callback */
void SQNCR::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{   
    // ROS_INFO("poseCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose.setRotation(q);
    robot_pose.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    is_pose_start = false;
    cout<<"robot_pose: "<<endl<<robot_pose.getOrigin().x()<<" "<<robot_pose.getOrigin().y()<<" "<<robot_pose.getOrigin().z()<<endl;

}

/*controller callback*/
void SQNCR::eeposeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    // ROS_INFO("eeposeCallback");
    ee_error = msg->data[0];

}

/* @brief task sequencer */
void SQNCR::taskSequencer(const ros::TimerEvent& event)
{
    std_msgs::Float64MultiArray joint_velocity; // joint velocity
    geometry_msgs::PoseStamped pose_msg; //pose message
    // double dist_error = hypot(hypot(ee_target[0]-ee_pose[0], ee_target[1]-ee_pose[1]), ee_target[2]-ee_pose[2]); 
    cout<<"ee_error: "<<ee_error<<endl;
    if (ee_error>_dist_err_threshold){
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = ee_target[0];
            pose_msg.pose.position.y = ee_target[1];
            pose_msg.pose.position.z = ee_target[2];
            goal_pub.publish(pose_msg);
            _is_joint_vel_stopped = false;
            // cout<<"republishing"<<endl;
        }else if(ee_error<=_dist_err_threshold && !_is_joint_vel_stopped) {
            joint_velocity.data.resize(3);
            joint_velocity.data[0] = 0;
            joint_velocity.data[1] = 0;
            joint_velocity.data[2] = 0;
            cout<<"stopping"<<endl;
            joints_vel_pub.publish(joint_velocity);
            _is_joint_vel_stopped = true;
        }
}
