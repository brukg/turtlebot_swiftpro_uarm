#include "swift_uarm/CNTRL.h"
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
CNTRL::CNTRL(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters  */

    // pose topic
    private_nh.param<std::string>("joint_vel_pub_topic", _joint_vel_pub_topic, "NONE");
    private_nh.param<std::string>("ee_pose_err_pub_sub_topic", _ee_pose_err_topic, "NONE");
    private_nh.param<std::string>("odom_pub_topic", _odom_pub_topic, "NONE");
    
    // subscribers params
    private_nh.param<std::string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<std::string>("joint_state_sub_topic", _joint_state_sub_topic, "NONE");
    private_nh.param<std::string>("ee_target_pose_pub_sub_topic", _ee_target_pose_sub_topic, "NONE");
    
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

    // subscribers
    // pose_sub = node.subscribe(_pose_sub_topic, 1, &CNTRL::poseCallback, this);
    joints_sub = node.subscribe(_joint_state_sub_topic, 9, &CNTRL::jointsCallback, this);
    ee_sub = node.subscribe(_ee_target_pose_sub_topic, 3, &CNTRL::controlCallback, this);
    
    
    // publishers
    joints_vel_pub = node.advertise<std_msgs::Float64MultiArray>(_joint_vel_pub_topic, 1);
    ee_pose_err_pub = node.advertise<std_msgs::Float64MultiArray>(_ee_pose_err_topic, 3);
    goal_pub = node.advertise<geometry_msgs::PoseStamped>(_ee_target_pose_sub_topic, 1);

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
}

/* @brief base_link to arm transformation */
bool CNTRL::getArm2BaseTransform()
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

/* @brief joints state listner */
void CNTRL::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{   
    // cout << "jointsCallback" << endl;
    //get joint names and values
    if (msg->name.size() ==4){
        // cout << "active joints update" << endl;
            //get joint values 
            for (int i = 0; i < msg->name.size(); i++)
            {   
                if (msg->name[i] == "joint1"){
                    joint_values(0,0) = msg->position[i];
                }else if (msg->name[i] == "joint2"){
                    joint_values(1,0) = msg->position[i];
                }else if (msg->name[i] == "joint3"){
                    joint_values(2,0)  = msg->position[i];
                }else if (msg->name[i] == "joint4"){
                    joint_values(3,0)  = msg->position[i];
                }
            }
        is_joints_read = true;
    }
}

/* @brief pose callback */
void CNTRL::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{   
    // ROS_INFO("poseCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose.setRotation(q);
    robot_pose.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    is_pose_start = false;
    cout<<"robot_pose: "<<endl<<robot_pose.getOrigin().x()<<" "<<robot_pose.getOrigin().y()<<" "<<robot_pose.getOrigin().z()<<endl;
    
}

/*controller callback*/
void CNTRL::controlCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    double dist_error = 0;
    ROS_INFO("control Callback");
    if(is_joints_read){
        std_msgs::Float64MultiArray joint_velocity, ee_pose_err_msg;
        // stop manipulator
        joint_velocity.data.resize(3);
        joint_velocity.data[0] = 0;
        joint_velocity.data[1] = 0;
        joint_velocity.data[2] = 0;
        // publish joint velocity
        joints_vel_pub.publish(joint_velocity);
        _is_joint_vel_stopped = false;

        ee_target[0] = msg->pose.position.x;
        ee_target[1] = msg->pose.position.y;
        ee_target[2] =  msg->pose.position.z;


        Eigen::MatrixXd error(3,1);
        Eigen::Matrix3d J, J_DLS;

        forwardKinematics(joint_values, ee_pose);
        cout<<"ee_pose: "<<endl<<ee_pose<<endl;
        // get Jacobian
        getJacobian(joint_values, J);
        error(0,0) = ee_target[0] - ee_pose[0];
        error(1,0) = ee_target[1] - ee_pose[1];
        error(2,0) = ee_target[2] - ee_pose[2];
        
        // joint velocity
        Eigen::MatrixXd dq(3,1);
        getDLS(J, _damping, J_DLS);
        dq = J_DLS*(_K*error);


        joint_velocity.data.resize(3);
        joint_velocity.data[0] = dq(0,0);
        joint_velocity.data[1] = dq(1,0);
        joint_velocity.data[2] = dq(2,0);
        // publish joint velocity
        joints_vel_pub.publish(joint_velocity);
        dist_error = hypot(hypot(ee_target[0]-ee_pose[0], ee_target[1]-ee_pose[1]), ee_target[2]-ee_pose[2]); 
        ee_pose_err_msg.data.resize(1);
        ee_pose_err_msg.data[0] = dist_error;
        ee_pose_err_pub.publish(ee_pose_err_msg);


    }    
}

