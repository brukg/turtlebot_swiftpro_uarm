#include "swift_uarm/ARUCO.h"
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
ARUCO::ARUCO(ros::NodeHandle node, ros::NodeHandle private_nh):image_transport_(node)
{
    /* Loading parameters  */

    // subscribers params
    private_nh.param<string>("odom_sub_topic", _odom_sub_topic, "NONE");
    private_nh.param<string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<string>("camera_topic", _camera_topic, "NONE");
    private_nh.param<string>("camera_info", _camera_info, "NONE");
    private_nh.param<string>("aruco_pose_topic", _aruco_pose_topic, "NONE");
    private_nh.param<int>("dictionary", _dictionary, 0);
    private_nh.param<int>("dictionary_id", _dictionary_id, 0);
    private_nh.param<double>("marker_length_m", _marker_length_m, 0.0);
    private_nh.param<bool>("slam", _is_slam, 0);


    // private_nh.param<cv::Mat>("camera_matrix", _camera_matrix);
    // private_nh.param<cv::Mat>("dist_coeffs", _dist_coeffs);
    
    // frames
    private_nh.param<string>("frame_id", _frame_id, "NONE");
    private_nh.param<string>("base_frame_id", _base_frame_id, "NONE");

    // subscribers
    if(_is_slam)  pose_sub = node.subscribe(_pose_sub_topic, 1, &ARUCO::poseCallback, this);
    else  pose_sub = node.subscribe(_odom_sub_topic, 1, &ARUCO::odomCallback, this);
    // image_transport_(node);
    camera_sub = image_transport_.subscribe(_camera_topic, 1, &ARUCO::imageCallback, this);
    caminfo_sub = node.subscribe(_camera_info, 1, &ARUCO::cameraInfoCallback, this);
    
    // publishers
    goal_pub = node.advertise<geometry_msgs::PoseStamped>(_aruco_pose_topic, 1);
    
    //initialising values
    is_pose_start = false;

    // robot_pose.setIdentity();//initialize robot pose
    robot_pose << Eigen::Matrix4d::Identity();//initialize robot pose

    is_pose_start = false;

    dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(_dictionary));
    _camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    _dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    tf2::Matrix3x3 rot_1;
    rot_1.setRPY(-1.571, 0, -1.571);; 
    // =  tf2::Matrix3x3(q.getRotation());

    c2b << rot_1[0][0], rot_1[0][1], rot_1[0][2], 0.138,
            rot_1[1][0], rot_1[1][1], rot_1[1][2], 0,
            rot_1[2][0], rot_1[2][1], rot_1[2][2], 0.065,
            0, 0, 0, 1;


};

/* @brief pose callback */
void ARUCO::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{   
    // ROS_INFO("poseCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose_tf.setRotation(q);
    robot_pose_tf.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf2::Matrix3x3 rot_1;
    rot_1.setRotation(q); 
    // =  tf2::Matrix3x3(q.getRotation());

    robot_pose << rot_1[0][0], rot_1[0][1], rot_1[0][2], msg->pose.pose.position.x,
                        rot_1[1][0], rot_1[1][1], rot_1[1][2], msg->pose.pose.position.y,
                        rot_1[2][0], rot_1[2][1], rot_1[2][2], msg->pose.pose.position.z,
                        0, 0, 0, 1;
    is_pose_start = true;
    
}


void ARUCO::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    // ROS_INFO("poseCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose_tf.setRotation(q);
    robot_pose_tf.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf2::Matrix3x3 rot_1;
    rot_1.setRotation(q); 
    // =  tf2::Matrix3x3(q.getRotation());

    robot_pose << rot_1[0][0], rot_1[0][1], rot_1[0][2], msg->pose.pose.position.x,
                        rot_1[1][0], rot_1[1][1], rot_1[1][2], msg->pose.pose.position.y,
                        rot_1[2][0], rot_1[2][1], rot_1[2][2], msg->pose.pose.position.z,
                        0, 0, 0, 1;
    is_pose_start = true;
}

// image callback
void ARUCO::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("imageCallback");
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image_mat;
    Eigen::Vector4d p;

    Eigen::Vector4d aruco_pose;

    try
    {
        image_mat = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image_mat, dictionary, corners, ids);
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_mat, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, _marker_length_m,  _camera_matrix, _dist_coeffs, rvecs, tvecs);
            for (int i = 0; i < ids.size(); i++)
            {
                // cv::aruco::drawAxis(image_mat, _camera_matrix, _dist_coeffs, rvecs[i], tvecs[i], _marker_length_m);
                // cout<<"tvecs "<<ids[i]<<" "<<tvecs[i]<<endl;
                if (ids[i]==_dictionary_id || ids[i]==_dictionary_id+10){
                    p(0) = tvecs[0][0]-0.04;
                    p(1) = tvecs[0][1];
                    p(2) = tvecs[0][2]+0.03;
                    p(3) = 1;
                    // cout<<"p "<<p<<endl;
                    p = c2b * p;
                    // cout<<"p "<<p<<endl;
                    aruco_pose = robot_pose * p;
                    // ROS_INFO("aruco_pose from aruco %f %f %f", aruco_pose[0], aruco_pose[1], aruco_pose[2]);
                    //publish aruco_pose
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header.frame_id = _frame_id;
                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.pose.position.x = aruco_pose(0);
                    pose_msg.pose.position.y = aruco_pose(1);
                    pose_msg.pose.position.z = 0.3;//aruco_pose(2);
                    
                    goal_pub.publish(pose_msg);
                }
            }
        
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

void ARUCO::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    // if (haveCamInfo) {
    //     return;
    // }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                _camera_matrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            _dist_coeffs.at<double>(0,i) = msg->D[i];
        }

        // haveCamInfo = true;
        // frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}