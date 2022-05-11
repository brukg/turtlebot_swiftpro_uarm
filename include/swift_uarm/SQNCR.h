
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
        // void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); //imu data callback
        void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg); //imu data callback

        void eeposeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg); //imu data callback
        void taskSequencer(const ros::TimerEvent& event); //task sequencer
        
        bool getArm2BaseTransform(); //tf frame transformation
        void forwardKinematics(Eigen::Vector4d& joints, Eigen::Vector3d& ee_pose); //forward kinematics        
        void getJacobian(Eigen::Vector4d& joints, Eigen::Matrix3d &J); //Jacobian matrix
        void getDLS(Eigen::Matrix3d &J, double lambda, Eigen::Matrix3d &J_DLS); //DLS
        
        ros::Timer sequencer; //task sequencer
        ros::Subscriber pose_sub, joints_sub, ee_sub; //subscriber robot pose, joint state, end effector pose(target)
        ros::Publisher joint_vel_pub_topic, goal_pub; //publishes geometry_msgs::PoseWithCovariance msgs
        ros::Publisher joints_vel_pub, base_velocity_pub;  //publishes sensor_msgs::PointCloud2
        Eigen::Matrix4f prev_transformation; //cumulative transformation until the previous time instance
        bool is_initial, is_pose_start, is_joints_read; //boolean to tell if this is 1st iteration of the algo and start of imu and pose reading
        Eigen::Quaternionf  _prev_quat; 
        Eigen::Vector3f     _prev_trans; 
        float ee_error, _dist_err_threshold, _prev_time_stamp; //time stamp of the previous point cloud

        /*---------sub parameters----------*/
        std::string _point_cloud_sub_topic; //point cloud ros topic to subscribe to
        std::string _pose_sub_topic; //pose ros topic to which to subscribe
        std::string _joint_state_sub_topic, _joint_vel_pub_topic, _base_vel_pub_topic;//joints ros topic to which to subscribe
        std::string _ee_target_pose_sub_topic, __ee_pose_err_sub_topic; //ee target pose topic to which to subscribe

        /*---------pub parameters----------*/
        std::string _pose_pub_topic; //pose ros topic to which to publish
        std::string _odom_pub_topic; //odom ros topic to which to publish
        std::string _point_cloud_pub_topic;
        

        /* frames*/
        std::string _frame_id, _base_frame_id, _arm_frame_id; //frame ids
        tf2::Transform mArm2BaseTransf;    // Coordinates of the base frame in camera frame
        bool mArm2BaseTransfValid = false;

        /*---------joints----------*/
        float _joint_angles[3]; //joint angles
        float _joint_velocities[3]; //joint velocities

        //link lengths
        float _link_1, _link_2, _vacuum_offset_x, _vacuum_offset_z, _base_offset_x, _base_offset_z;
        Eigen::Vector3d ee_pose; //end effector pose
        Eigen::Vector4d ee_target,joint_values; //joint values
        Eigen::Matrix4d r2b; //robot to base transformation
        // parameters
        float _K, _X,_Y,_Z, _damping, _seq_freq; //gain an height
        bool _is_mobile_base; //if mobile base
        bool _is_vacuum_gripper; //if vacuum gripper
        bool _is_joint_vel_stopped; //if joint vel control is stopped
        bool _is_ee_pose_control; //if ee pose control
        bool _is_ee_pose_target_control; //if ee target pose control
        bool _is_joint_control; //if joint control
        bool _is_joint_vel_target_control; //if joint vel target control


        

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;
        tf2::Transform robot_pose;//ekf robot pose
};

#endif