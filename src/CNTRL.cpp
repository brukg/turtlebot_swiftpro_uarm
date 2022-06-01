#include "swift_uarm/CNTRL.h"
#include "swift_uarm/utils.h"
#include <iostream>
#include <chrono>
#include <math.h> 

#include <geometry_msgs/PoseWithCovarianceStamped.h> //robot pose from slam
#include <geometry_msgs/PoseStamped.h> //part to pick
#include <std_msgs/Float64MultiArray.h> // joints state
#include <std_srvs/Empty.h> // joints state


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
    private_nh.param<string>("ee_pose_pub_topic", _ee_pose_topic, "NONE");
    private_nh.param<string>("joint_err_pub_sub_topic", _joint_err_topic, "NONE");
    private_nh.param<string>("odom_pub_topic", _odom_pub_topic, "NONE");
    private_nh.param<string>("base_vel_pub_topic", _base_vel_pub_topic, "NONE");
    private_nh.param<string>("vacuume_service_topic_on", _vacuume_service_topic_on, "NONE");
    private_nh.param<string>("vacuume_service_topic_off", _vacuume_service_topic_off, "NONE");
    
    // subscribers params
    private_nh.param<string>("pose_sub_topic", _pose_sub_topic, "NONE");
    private_nh.param<string>("odom_sub_topic", _odom_sub_topic, "NONE");
    private_nh.param<string>("joint_state_sub_topic", _joint_state_sub_topic, "NONE");
    private_nh.param<string>("ee_target_pose_pub_sub_topic", _ee_target_pose_sub_topic, "NONE");
    
    // frames
    private_nh.param<string>("frame_id", _frame_id, "NONE");
    private_nh.param<string>("arm_frame_id", _arm_frame_id, "NONE");
    private_nh.param<string>("base_frame_id", _base_frame_id, "NONE");

    // constants
    private_nh.param<double>("link_1", _link_1, 0.0);
    private_nh.param<double>("link_2", _link_2, 0.0);
    private_nh.param<double>("base_offset_x", _base_offset_x, 0.0);
    private_nh.param<double>("base_offset_z", _base_offset_z, 0.0);
    private_nh.param<double>("vacuum_offset_x", _vacuum_offset_x, 0.0);
    private_nh.param<double>("vacuum_offset_z", _vacuum_offset_z, 0.0);
    
    private_nh.param<float>("gain", _K, 10.0);
    private_nh.param<float>("wv", wv, 1.0);
    private_nh.param<float>("ww", ww, 1.0);
    private_nh.param<float>("wx", wx, 1.0);
    private_nh.param<float>("wy", wy, 1.0);
    private_nh.param<float>("wz", wz, 1.0);
    private_nh.param<float>("damping", _damping, 0.01);
    private_nh.param<bool>("mobile_base", _is_mobile_base, 1);
    private_nh.param<bool>("slam", _is_slam, 0);

    private_nh.param<float>("base_dist_err_threshold", _base_dist_err_threshold, 1);


    // subscribers
    if(_is_slam)  pose_sub = node.subscribe(_pose_sub_topic, 1, &CNTRL::poseCallback, this);
    else  pose_sub = node.subscribe(_odom_sub_topic, 1, &CNTRL::odomCallback, this);
    joints_sub = node.subscribe(_joint_state_sub_topic, 1, &CNTRL::jointsCallback, this);
    ee_sub = node.subscribe(_ee_target_pose_sub_topic, 1, &CNTRL::controlCallback, this);
    vacuum_service_off = node.serviceClient<std_srvs::Empty>(_vacuume_service_topic_off);
    vacuum_service_on = node.serviceClient<std_srvs::Empty>(_vacuume_service_topic_on);
    
    
    // publishers
    joints_vel_pub = node.advertise<std_msgs::Float64MultiArray>(_joint_vel_pub_topic, 1);
    ee_pose_err_pub = node.advertise<std_msgs::Float64MultiArray>(_ee_pose_err_topic, 1);
    joint_err_pub = node.advertise<std_msgs::Float64MultiArray>(_joint_err_topic, 1);
    ee_pose_pub = node.advertise<std_msgs::Float64MultiArray>(_ee_pose_topic, 1);
    // goal_pub = node.advertise<geometry_msgs::PoseStamped>(_ee_target_pose_sub_topic, 1);
    base_velocity_pub = node.advertise<geometry_msgs::Twist>(_base_vel_pub_topic, 1);
    //initialising values

    is_pose_start = false;

    robot_pose.setIdentity();//initialize robot pose
    joint_values.setZero();
    dq.setZero(6,1);
    //set the eigen diagonal matrix
    gain.diagonal()<< 2*_K, 2*_K, 2*_K, 0.2*_K;
    
    ee_pose.setZero(4); ee_target <<_X, _Y, _Z, 0;
    is_joints_read = false;  is_pose_start = false; _is_vacuum_gripper = false; pose_reached = false;  aruco_picked = false; aruco_placing = false;// _is_joint_vel_stopped = false;
    mobile_manipulator.setLinkValues(_link_1, _link_2, _vacuum_offset_x, _vacuum_offset_z, _base_offset_x, _base_offset_z);
    mobile_manipulator.setMobile(_is_mobile_base);
    desired_joint_pick << -1.5, 0, 0.0;
    desired_joint_place << 1.445,-0.556 , 0;
}


void CNTRL::vacuumServiceOn(){
    std_srvs::Empty req;
    // std_srvs::Empty::Request req;
    // std_srvs::Empty::Response res;
    //empty service call
    
    ros::service::waitForService(_vacuume_service_topic_on, ros::Duration(0.3));
    vacuum_service_on.call(req);
    _is_vacuum_gripper = true;
}

void CNTRL::vacuumServiceOff(){
    std_srvs::Empty req;
    // std_srvs::Empty::Request req;
    // std_srvs::Empty::Response res;
    //empty service call
    
    ros::service::waitForService(_vacuume_service_topic_off, ros::Duration(0.3));
    vacuum_service_off.call(req);
    _is_vacuum_gripper = false;
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
    // cout<<"joint_values"<<joint_values<<endl;
}

/* @brief pose callback from slam  */
void CNTRL::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{   
    // ROS_INFO("poseCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose.setRotation(q);
    robot_pose.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    is_pose_start = false;
    tf2::Matrix3x3 rot_1(q);
    // rot_1.setRotation(q); 
    // =  tf2::Matrix3x3(q.getRotation());

    robot_pose_eigen << rot_1[0][0], rot_1[0][1], rot_1[0][2], msg->pose.pose.position.x,
                        rot_1[1][0], rot_1[1][1], rot_1[1][2], msg->pose.pose.position.y,
                        rot_1[2][0], rot_1[2][1], rot_1[2][2], msg->pose.pose.position.z,
                        0, 0, 0, 1;
    
    mobile_manipulator.setBasePose(robot_pose_eigen);
    double yaw, pitch, roll;
    rot_1.getRPY(roll,pitch,yaw);
    joint_values(1,0) = yaw; //robot yaw angle
    is_pose_start = true;
}


void CNTRL::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    // ROS_INFO("odomCallback");
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_pose.setRotation(q);
    robot_pose.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    is_pose_start = false;
    tf2::Matrix3x3 rot_1(q);
    // rot_1.setRotation(q); 
    // =  tf2::Matrix3x3(q.getRotation());

    robot_pose_eigen << rot_1[0][0], rot_1[0][1], rot_1[0][2], msg->pose.pose.position.x,
                        rot_1[1][0], rot_1[1][1], rot_1[1][2], msg->pose.pose.position.y,
                        rot_1[2][0], rot_1[2][1], rot_1[2][2], msg->pose.pose.position.z,
                        0, 0, 0, 1;
    
    mobile_manipulator.setBasePose(robot_pose_eigen);
    double yaw, pitch, roll;
    rot_1.getRPY(roll,pitch,yaw);
    joint_values(1,0) = yaw; //robot yaw angle
    is_pose_start = true;
}

/*controller callback*/
void CNTRL::controlCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    // ROS_INFO("control Callback");
    Eigen::Vector3d dist_error; dist_error.setZero();
    RowVector6d JJ, J_bar_; JJ.setZero(); J_bar_.setZero();
    Vector6d J_DLS_(6,1), J_bar_inv(6,1);  J_DLS_.setZero(); J_bar_inv.setZero();
    Eigen::Vector4d  err; err.setZero();
    Matrix6d J, J_bar, J_DLS, W(6,6), Winv(6,6), P = Matrix6d::Identity(6,6); J_bar.setZero(); J_DLS.setZero();
    double err_ =0; // for joint limit and position task

    if(is_joints_read && is_pose_start){
        std_msgs::Float64MultiArray joint_velocity, ee_pose_err_msg, joint_err_msg, ee_pose_msg;
        geometry_msgs::Twist base_velocity;

        ee_target[0] = msg->pose.position.x;
        ee_target[1] = msg->pose.position.y;
        ee_target[2] =  msg->pose.position.z;
        ROS_INFO("ee_target: %f, %f, %f", ee_target[0], ee_target[1], ee_target[2]);
        tf2::Quaternion q(msg->pose.orientation.x,  msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ee_target[3] =  0;

        if (ee_target(0) == -100&& ee_target(1) == -100 && ee_target(2) == 0.14){
            // move near aruco

            mobile_manipulator.getPose(ee_pose);
            ee_target << ee_pose(0), ee_pose(1), 0.14, 0;
            pose_reached = true;
        }else if (ee_target(0) == -100&& ee_target(1) == -100 && ee_target(2) == 0.138){
            //move Z down to aruco
            vacuumServiceOn();
            mobile_manipulator.getPose(ee_pose);
            ee_target << ee_pose(0), ee_pose(1), 0.138, 0;
            pose_reached = true;
        }else if(ee_target(0) == -100 && ee_target(1) == -100 && ee_target(2) == -100){
            //pick up aruco
            vacuumServiceOn();

            ROS_INFO("picking aruco controller");
            pose_reached = true;
            aruco_picked = true;

            ee_target << ee_pose(0), ee_pose(1), 0.45, 0;
            //joint position task
            for(int i = 1; i < 2; i++){
                joints_position.setDesired(desired_joint_pick(i));
                joints_position.update(mobile_manipulator, i+2); //shift by 2 to skip the first two base joints
                joints_position.getJacobian(JJ);
                J_bar_ = JJ*P;
                mobile_manipulator.getDLS(J_bar_, _damping, J_DLS_);
                joints_position.getError(err_);
                dq += J_DLS_ *(_K*err_ - JJ*dq);
                J_bar_inv = J_bar_.transpose() * (J_bar_* J_bar_.transpose()).inverse();
                P -= J_bar_inv * J_bar_;

            }
        }else if(ee_target(0) == -200 && ee_target(1) == -200 && ee_target(2) == -200){
            // vacuumServiceOn();
            aruco_picked = true;
            aruco_placing = true;
            ROS_INFO("placing aruco on base");
            pose_reached = true;
            ee_target << ee_pose(0), ee_pose(1), 0.45, 0;
            //joint position task
            for(int i = 0; i < 3; i++){
                joints_position.setDesired(desired_joint_place(i));
                joints_position.update(mobile_manipulator, i+2); //shift by 2 to skip the first two joints
                joints_position.getJacobian(JJ);
                J_bar_ = JJ*P;
                mobile_manipulator.getDLS(J_bar_, _damping, J_DLS_);
                joints_position.getError(err_);
                dq += J_DLS_ *(_K*err_ - JJ*dq);
                J_bar_inv = J_bar_.transpose() * (J_bar_* J_bar_.transpose()).inverse();
                P -= J_bar_inv * J_bar_;

            }

        }else if(ee_target(0) == -300 && ee_target(1) == -300 && ee_target(2) == -300){
            ROS_INFO("vacuum off");
            vacuumServiceOff();
            // aruco_picked = true;
            

        }else  if(ee_target(0) == -400 && ee_target(1) == -400 && ee_target(2) == -400){
            dq.setZero();
            ee_target << ee_pose(0), ee_pose(1), ee_pose(2), 0;

            aruco_picked = false;

        }
        W.setZero(6,6);
        Winv.setZero(6,6);
        
        
        // joint limit task for the 3 arm joints
        for (int i = 0; i < 3; i++)
        {
            joint_limits.update(mobile_manipulator, i+2);
            joint_limits.getJacobian(JJ);
            J_bar_ = JJ*P;
            mobile_manipulator.getDLS(J_bar_, _damping, J_DLS_);
            if (joint_limits.isActive()){
                joint_limits.getError(err_);
                cout<<i+2<<" err_: "<<err_<<endl;
                dq += J_DLS_ *(_K*err_ - JJ*dq);
                J_bar_inv = J_bar_.transpose() * (J_bar_* J_bar_.transpose()).completeOrthogonalDecomposition().pseudoInverse();
                P -= J_bar_inv * J_bar_;
            }
        }

        // position task always active except when aruco is getting placed to the back of base
        if (!aruco_picked){
            // arm pose task
            arm_pose.setDesired(ee_target); //set desired arm pose from sequencer 
            arm_pose.update(mobile_manipulator); //update arm pose
            arm_pose.getJacobian(J); //get Jacobian
            // cout<<"J: "<<endl<<J<<endl;
            J_bar = J*P;

                // adjust weight matrix based on distance
                double base_dist = hypot(ee_target(0) - robot_pose_eigen(0,3), ee_target(1) - robot_pose_eigen(1,3));
                if (base_dist> 2*_base_dist_err_threshold) Winv.diagonal() << 0.09, 1.5, 1, 1, 1, 1;
                else if (base_dist> _base_dist_err_threshold) Winv.diagonal() << 0.05, 1, 1, 1, 1, 1;
                else {Winv.diagonal() << wv, ww, wx, wy, wz, 1; };

            // Winv = W.transpose() *(W*W.transpose()).inverse();
            mobile_manipulator.getDLS(J_bar, _damping, J_DLS, Winv);
            arm_pose.getError(err); //get error
            
            dq += J_DLS *(gain*err - J*dq);

            P -= J_bar.completeOrthogonalDecomposition().pseudoInverse() * J_bar;
            
        }
        // cout<<"err: "<<err<<endl;

        // velocity limit
        double S = ((dq.cwiseAbs())/0.25).maxCoeff();
        if (S>1) dq << dq/S;
        
        for (int i=0; i<dq.rows(); i++){
            if(std::isnan(dq(i))){
                cout<<"dq is nan"<<endl;
                dq(i) = 0.2;
            }
        }
        
        cout<<"dq: "<<dq<<endl;
        // publish joint velocity
        if(ee_target(0) == -400 && ee_target(1) == -400 && ee_target(2) == -400){
            dq.setZero();
            aruco_picked = false;

        }
        joint_velocity.data.resize(4);
        joint_velocity.data[0] = dq(2,0); 
        joint_velocity.data[1] = dq(3,0); 
        joint_velocity.data[2] = dq(4,0); 
        joint_velocity.data[3] = dq(5,0); 
        
        joints_vel_pub.publish(joint_velocity);

        // publish base velocity
        if (pose_reached){
            // ROS_INFO("base stoped");

            base_velocity.linear.x = 0; 
            base_velocity.linear.y = 0;
            base_velocity.linear.z = 0;
            base_velocity.angular.x = 0;
            base_velocity.angular.y = 0;
            base_velocity.angular.z = 0;
        
        }else{
            base_velocity.linear.x = dq(0,0); 
            base_velocity.linear.y = 0;
            base_velocity.linear.z = 0;
            base_velocity.angular.x = 0;
            base_velocity.angular.y = 0;
            base_velocity.angular.z = dq(1,0);

        }
        base_velocity_pub.publish(base_velocity);

        mobile_manipulator.getPose(ee_pose);
        ee_pose_msg.data.resize(3);
        ee_pose_msg.data[0] = ee_pose(0);
        ee_pose_msg.data[1] = ee_pose(1);
        ee_pose_msg.data[2] = ee_pose(2);
        ee_pose_pub.publish(ee_pose_msg);
        cout<<"EE_pose: "<<endl<<ee_pose<<endl;
        // publish error message to sequencer node
        if(!aruco_picked){
            dist_error(0) = abs(ee_target[0]-ee_pose[0]); 
            dist_error(1) =  abs(ee_target[1]-ee_pose[1]); 
            dist_error(2) = abs(ee_target[2]-ee_pose[2]); 

            ee_pose_err_msg.data.resize(3);
            ee_pose_err_msg.data[0] = dist_error(0);
            ee_pose_err_msg.data[1] = dist_error(1);
            ee_pose_err_msg.data[2] = dist_error(2);
            ee_pose_err_pub.publish(ee_pose_err_msg);

        }else if(aruco_picked && !aruco_placing){
            dist_error(0) = 0;    
            dist_error(2) = 0;    

            for (int i = 1; i < 2; i++){
                joints_position.setDesired(desired_joint_pick(i));
                joints_position.update(mobile_manipulator, i+2);
                joints_position.getError(err_);
                dist_error(i) = abs(err_);    
            }

            joint_err_msg.data.resize(3);
            joint_err_msg.data[0] = dist_error(0);
            joint_err_msg.data[1] = dist_error(1);
            joint_err_msg.data[2] = dist_error(2);
            joint_err_pub.publish(joint_err_msg);
            ROS_INFO("joint  picking error: %f, %f, %f", dist_error(0), dist_error(1), dist_error(2));
        }else if(aruco_placing){

            for (int i = 0; i < 3; i++){
                joints_position.setDesired(desired_joint_place(i));
                joints_position.update(mobile_manipulator, i+2);
                joints_position.getError(err_);
                dist_error(i) = abs(err_);    
            }

            joint_err_msg.data.resize(3);
            joint_err_msg.data[0] = dist_error(0);
            joint_err_msg.data[1] = dist_error(1);
            joint_err_msg.data[2] = dist_error(2);
            joint_err_pub.publish(joint_err_msg);
            ROS_INFO("joint  placing error: %f, %f, %f", dist_error(0), dist_error(1), dist_error(2));
        }

    }    
}

