#ifndef TASK_H
#define TAsk_H
#include <vector>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;

// #include <tf2_ros/buffer.h>

class MobileManipulator
{
    public:
        MobileManipulator();
        ~MobileManipulator() {};
        void getEEJacobian(Eigen::Matrix3d &J);
        void getPose(Eigen::Vector3d& ee_pose);
        void getDLS(Eigen::Matrix3d &J, double lambda, Eigen::Matrix3d &J_DLS); //DLS
        void getDLS(Eigen::RowVector3d &J, double lambda, Eigen::Vector3d &J_DLS); //DLS
        void setJoints(Eigen::Vector4d joints);
        void getJoints(Eigen::Vector3d &q);

    private:
        void forwardKinematics(Eigen::Vector4d& joints, Eigen::Vector3d& ee_pose); //forward kinematics        
        void getJacobian(Eigen::Vector4d& joints, Eigen::Matrix3d &J); //Jacobian matrix
        
        void update();
        void getEETransform();
        void getJointPos( );
        void getDOF();

        // variables
        Eigen::Vector4d joint_values;
        float _link_1, _link_2, _vacuum_offset_x, _vacuum_offset_z, _base_offset_x, _base_offset_z;
        Eigen::Matrix4d r2b; //robot to base transformation
        bool _is_mobile_base; 

};

class TASK
{
    public:
        TASK();
        ~TASK() {};
        void update(MobileManipulator &robot);
        void isActive();
        void setDesired();
        void getDesired();
        void getJacobian(Eigen::Matrix3d J);
        void getError();
    
        void setK(double& K);
        
        void getK(double& K);
        void setFF(double& FF);
        void getFF(double& FF);
        bool isActive(bool& is_active);
        void euler_to_quaternion();

    private:
        double K, FF;
        Eigen::Vector3d sigma_d;
        bool active;

       
};

class Position : public TASK
{
    public:
        Position();
        ~Position() {};
        void update(MobileManipulator &robot);
        void getJacobian(Eigen::Matrix3d &J);
        void getError(Eigen::Vector3d &error);
        bool isActive();
        void setDesired(Eigen::Vector3d &sigma_d);
        void getDesired(Eigen::Vector3d &sigma_d);

    private:
        Eigen::Matrix3d  J, J_DLS;
        Eigen::Vector3d error, ee_pose, sigma_d;
        bool active;

};

class JointLimits : public TASK
{
    public:
        JointLimits();
        ~JointLimits() {};
        void update(MobileManipulator &robot);
        bool isActive();
        void setDesired(Eigen::Vector3d &sigma_d);
        void getDesired(Eigen::Vector3d &sigma_d);
        void getJacobian(Eigen::RowVector3d &J);
        void getError(double &error);

    private:
        Eigen::RowVector3d  J, J_DLS;
        Eigen::Vector3d sigma_d, q;
        std::vector<Eigen::Vector2d> jointlimits;
        bool active;
        Eigen::Vector3d joints;
        double dxe;
        

};

#endif