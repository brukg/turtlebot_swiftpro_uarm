
#ifndef UARM_H
#define UARM_H
#include "swift_uarm/ROBOT_TASK.h"
#include <string>

//ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <eigen3/Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //


class CNTRL
{
    public:
        CNTRL(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~CNTRL() {};

    private:
        // void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
        void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void vacuumServiceOn();
        void vacuumServiceOff();
        void controlCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    
        
        ros::Subscriber pose_sub, joints_sub, ee_sub; //subscriber robot pose, joint state, end effector pose(target)
        ros::Publisher goal_pub, ee_pose_err_pub, joint_err_pub, ee_pose_pub; //publishes poses
        ros::Publisher joints_vel_pub, base_velocity_pub;  //publishes velocities
        ros::ServiceClient vacuum_service_on, vacuum_service_off; //vacuum service

        Eigen::Matrix4f prev_transformation; //cumulative transformation until the previous time instance
        bool is_initial, is_pose_start, is_joints_read; //boolean to tell if this is 1st iteration of the algo and start of imu and pose reading
        Eigen::Quaternionf  _prev_quat; 
        Eigen::Vector3f     _prev_trans; 
        double _prev_time_stamp; //time stamp of the previous point cloud

        /*---------sub parameters----------*/
        std::string _point_cloud_sub_topic; //point cloud ros topic to subscribe to
        std::string _pose_sub_topic, _odom_sub_topic; //pose ros topic to which to subscribe
        std::string _joint_state_sub_topic, _joint_vel_pub_topic, _ee_pose_err_topic, _ee_pose_topic, _joint_err_topic, _base_vel_pub_topic;//joints ros topic to which to subscribe
        std::string _ee_target_pose_sub_topic; //ee target pose topic to which to subscribe

        /*---------pub parameters----------*/
        std::string _pose_pub_topic; //pose ros topic to which to publish
        std::string _odom_pub_topic; //odom ros topic to which to publish
        std::string _point_cloud_pub_topic, _vacuume_service_topic_on, _vacuume_service_topic_off;
        

        /* frames*/
        std::string _frame_id, _base_frame_id, _arm_frame_id; //frame ids
        tf2::Transform mArm2BaseTransf;    // Coordinates of the base frame in camera frame
        bool mArm2BaseTransfValid = false;

        /*---------joints----------*/
        float _joint_angles[3]; //joint angles
        float _joint_velocities[3]; //joint velocities

        //link lengths
        double _link_1, _link_2, _vacuum_offset_x, _vacuum_offset_z, _base_offset_x, _base_offset_z;
        float wv, ww, wx, wy, wz; //weighting factors
        float _alpha, _delta;
        Eigen::Vector4d ee_target, ee_pose;   //end effector target, end effector pose
        Vector6d joint_values, dq; //joint values
        Eigen::Matrix4d r2b; //robot to base transformation
        // parameters
        float _K, _X,_Y,_Z, _damping; //gain an height
        float _base_dist_err_threshold;
        bool _is_mobile_base, _is_slam; //if mobile base
        bool _is_vacuum_gripper; //if vacuum gripper
        bool _is_joint_vel_stopped; //if joint vel control is stopped
        bool _is_ee_pose_control; //if ee pose control
        bool _is_ee_pose_target_control; //if ee target pose control
        bool _is_joint_control; //if joint control
        bool _is_joint_vel_target_control; //if joint vel target control
        bool pose_reached, aruco_picked, aruco_placing; //if pose reached

        tf2::Transform robot_pose;//slam robot pose
        Eigen::Matrix4d robot_pose_eigen;//slam robot pose
        Eigen::DiagonalMatrix<double, 4> gain; //gain matrix
        Position arm_pose; //position task
        JointLimits joint_limits; //joint limits
        JointsPosition joints_position; //joints position task
        MobileManipulator mobile_manipulator; //mobile manipulator
        Eigen::Vector3d desired_joint_pick, desired_joint_place; //desired joint angles for placing object in the base
};

#endif 