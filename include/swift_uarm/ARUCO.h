
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

//opencv 
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class ARUCO
{
    public:
        ARUCO(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~ARUCO() {};

    private:
        // void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg); //image callback
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg); //camera info callback
        ros::Subscriber pose_sub, caminfo_sub; 
        ros::Publisher goal_pub; 

        image_transport::ImageTransport image_transport_; //image transport
        image_transport::Subscriber camera_sub;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv::Mat _camera_matrix, _dist_coeffs;
        int _dictionary_id, _dictionary;

        bool is_initial, is_pose_start, _is_slam; 
        double _marker_length_m; //time stamp of the previous point cloud

        /*---------sub parameters----------*/
        std::string _odom_sub_topic, _pose_sub_topic, _camera_topic, _camera_info, _aruco_pose_topic; //ee target pose topic to which to subscribe

        /*---------pub parameters----------*/
        std::string _pose_pub_topic; //pose ros topic to which to publish
        

        /* frames*/
        std::string _frame_id, _base_frame_id, _arm_frame_id; //frame ids


        tf2::Transform robot_pose_tf;//slam robot pose
        Eigen::Matrix4d robot_pose;//slam robot pose
        Eigen::Matrix4d c2b;

};

#endif 