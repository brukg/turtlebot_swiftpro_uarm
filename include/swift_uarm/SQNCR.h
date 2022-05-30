
#ifndef SQNCR_H
#define SQNCR_H
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

typedef Eigen::Matrix< double, 6, 1 > Vector6d;


class SQNCR
{
    public:
        SQNCR(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~SQNCR() {};

    private:
        void arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
        void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);

        void eeposeerrCallback(const std_msgs::Float64MultiArray::ConstPtr& msg); 
        void taskSequencer(const ros::TimerEvent& event); //task sequencer
        
        ros::Timer sequencer; //task sequencer
        ros::Subscriber robot_pose_sub, aruco_pose_sub, joints_sub, ee_err_sub; //subscriber robot pose, joint state, end effector pose(target)
        ros::Publisher joint_vel_pub_topic, goal_pub; //publishes geometry_msgs::PoseWithCovariance msgs
        ros::Publisher joints_vel_pub, base_velocity_pub;  //publishes sensor_msgs::PointCloud2
        bool is_initial, is_robot_pose, is_aruco_pose, is_joints_read; //boolean to tell if this is 1st iteration of the algo and start of imu and pose reading
        Eigen::Vector3f     aruco_pose; 
        Eigen::Vector3f ee_error, joint_error, _prev_time_stamp; //time stamp of the previous point cloud
        float _dist_err_threshold;
        /*---------sub parameters----------*/
        std::string _point_cloud_sub_topic; //point cloud ros topic to subscribe to
        std::string _pose_sub_topic, _odom_sub_topic; //pose ros topic to which to subscribe
        std::string _joint_state_sub_topic, _joint_vel_pub_topic, _base_vel_pub_topic;//joints ros topic to which to subscribe
        std::string _aruco_pose_topic, _ee_target_pose_sub_topic, __ee_pose_err_sub_topic; //ee target pose topic to which to subscribe

        /*---------pub parameters----------*/
        std::string _pose_pub_topic; //pose ros topic to which to publish
        std::string _odom_pub_topic; //odom ros topic to which to publish
        

        /* frames*/
        std::string _frame_id, _base_frame_id, _arm_frame_id; //frame ids
        tf2::Transform mArm2BaseTransf;    // Coordinates of the base frame in camera frame
        // parameters
        float _K, _X,_Y,_Z, _damping, _seq_freq; //gain an height
        bool _is_mobile_base, _is_slam; //if mobile base
        bool _is_vacuum_gripper; //if vacuum gripper
        bool _is_base_stopped; //if joint vel control is stopped
        bool _is_ee_pose_control; //if ee pose control
        bool _is_ee_pose_target_control; //if ee target pose control
        bool _is_joint_control; //if joint control
        bool _is_joint_vel_target_control; //if joint vel target control
        bool _is_error, _is_j_error;
        bool _is_aruco_picked, _aruco_placing, _aruco_placed;

        

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;
        tf2::Transform robot_pose;//ekf robot pose
};

#endif