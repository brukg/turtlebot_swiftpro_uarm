#include "swift_uarm/CNTRL.h"
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
CNTRL::CNTRL(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    /* Loading parameters  */

    // pose topic
    private_nh.param<string>("joint_vel_pub_topic", _joint_vel_pub_topic, "NONE");
    private_nh.param<string>("ee_pose_err_pub_sub_topic", _ee_pose_err_topic, "NONE");
    private_nh.param<string>("odom_pub_topic", _odom_pub_topic, "NONE");
    
    // subscribers params
    private_nh.param<string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<string>("joint_state_sub_topic", _joint_state_sub_topic, "NONE");
    private_nh.param<string>("ee_target_pose_pub_sub_topic", _ee_target_pose_sub_topic, "NONE");
    
    // frames
    private_nh.param<string>("frame_id", _frame_id, "NONE");
    private_nh.param<string>("arm_frame_id", _arm_frame_id, "NONE");
    private_nh.param<string>("base_frame_id", _base_frame_id, "NONE");

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

    robot_pose.setIdentity();//initialize robot pose


    // link lengths in meters
    // _link_1 = 0.456;   _link_2 = 0.142;  _link_3 = 0.;  _link_4 = 0.142; _link_5 = 0.1588 +0.56;
    joint_values.setZero(6);

    // r2b <<  0, -1, 0, 0.037,
    //         1, 0, 0, 0,
    //         0, 0, 1, 0.147,
    //         0, 0, 0, 1;
    ee_pose.setZero(6); ee_target <<0, 0,_X, _Y, _Z, 0;
    is_joints_read = false;// _is_vacuum_gripper = false;// _is_joint_vel_stopped = false;

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
                    joint_values(2,0) = msg->position[i];
                }else if (msg->name[i] == "joint2"){
                    joint_values(3,0) = msg->position[i];
                }else if (msg->name[i] == "joint3"){
                    joint_values(4,0)  = msg->position[i];
                }else if (msg->name[i] == "joint4"){
                    joint_values(5,0)  = msg->position[i];
                }
            mobile_manipulator.setJoints(joint_values);
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
    robot_pose_eigen = Eigen::Isometry3d(Eigen::Translation3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)
                             * Eigen::Quaterniond(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    cout<<"robot_pose: "<<endl<<robot_pose.getOrigin().x()<<" "<<robot_pose.getOrigin().y()<<" "<<robot_pose.getOrigin().z()<<endl;

}

/*controller callback*/
void CNTRL::controlCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    // ROS_INFO("control Callback");
    double dist_error = 0;
    if(is_joints_read){
        std_msgs::Float64MultiArray joint_velocity, ee_pose_err_msg;
        // stop manipulator
        // joint_velocity.data.resize(4);
        // joint_velocity.data[0] = 0;
        // joint_velocity.data[1] = 0;
        // joint_velocity.data[2] = 0;
        // joint_velocity.data[3] = 0;
        // // publish joint velocity
        // joints_vel_pub.publish(joint_velocity);

        ee_target[2] = msg->pose.position.x;
        ee_target[3] = msg->pose.position.y;
        ee_target[4] =  msg->pose.position.z;
        tf2::Quaternion q(msg->pose.orientation.x,  msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ee_target[5] =  yaw;


        Vector6d  err(6,1);
        Vector6d dq(6,1);
        Matrix6d J, J_bar, J_DLS, P = Matrix6d::Identity();

        // joint limit task
        joint_limits.update(mobile_manipulator);
        RowVector6d JJ, J_bar_;
        Vector6d J_DLS_(6,1), J_bar_inv(6,1); 
        double err_;
        joint_limits.getJacobian(JJ);
        J_bar_ = JJ*P;
        mobile_manipulator.getDLS(J_bar_, _damping, J_DLS_);
        if (joint_limits.isActive()){
            joint_limits.getError(err_);
            cout<<"err_: "<<err_<<endl;
            dq += J_DLS_ *(_K*err_ - JJ*dq);
            J_bar_inv = J_bar_.transpose() * (J_bar_* J_bar_.transpose()).completeOrthogonalDecomposition().pseudoInverse();
            P -= J_bar_inv * J_bar_;
        }

        // arm pose task
        arm_pose.setDesired(ee_target); //set desired arm pose from sequencer 
        cout<<"ee_target: "<<endl<<ee_target<<endl;
        arm_pose.update(mobile_manipulator); //update arm pose
        arm_pose.getJacobian(J); //get Jacobian
        J_bar = J*P;
        mobile_manipulator.getDLS(J_bar, _damping, J_DLS);
        arm_pose.getError(err); //get error
        cout<<"err: "<<endl<<err<<endl;
        // cout<<"EE_target: "<<ee_target<<endl;
        dq += J_DLS *(_K*err - J*dq);
        P -= J_bar.completeOrthogonalDecomposition().pseudoInverse() * J_bar;

        // publish joint velocity
        joint_velocity.data.resize(4);
        joint_velocity.data[0] = dq(2,0);
        joint_velocity.data[1] = dq(3,0);
        joint_velocity.data[2] = dq(4,0);
        joint_velocity.data[3] = dq(5,0);
        joints_vel_pub.publish(joint_velocity);

        mobile_manipulator.getPose(ee_pose);
        cout<<"EE_pose: "<<endl<<ee_pose<<endl;
        // publish error message to sequencer node
        dist_error = hypot(hypot(ee_target[2]-ee_pose[2], ee_target[3]-ee_pose[3]), ee_target[4]-ee_pose[4]); 
        ee_pose_err_msg.data.resize(1);
        ee_pose_err_msg.data[0] = dist_error;
        ee_pose_err_pub.publish(ee_pose_err_msg);


    }    
}

