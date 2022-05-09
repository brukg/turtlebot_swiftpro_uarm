#ifndef TASK_H
#define TAsk_H
#include <vector>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< double, 1, 6 > RowVector6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
// #include <tf2_ros/buffer.h>

class MobileManipulator
{
    public:
        MobileManipulator();
        ~MobileManipulator() {};
        void getEEJacobian(Matrix6d &J);
        void getPose(Vector6d& ee_pose);
        void getDLS(Matrix6d &J, double lambda, Matrix6d &J_DLS); //DLS
        void getDLS(RowVector6d &J, double lambda, Vector6d &J_DLS); //DLS
        void setJoints(Vector6d joints);
        void getJoints(Vector6d &q);

    private:
        void forwardKinematics(Vector6d& joints, Vector6d& ee_pose); //forward kinematics        
        void getJacobian(Vector6d& joints, Matrix6d &J); //Jacobian matrix
        
        void update();
        void getEETransform();
        void getJointPos( );
        void getDOF();

        // variables
        Vector6d joint_values;
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
        void getJacobian(Matrix6d &J);
        void getError(Vector6d &error);
        bool isActive();
        void setDesired(Eigen::Vector3d &sigma_d);
        void getDesired(Eigen::Vector3d &sigma_d);

    private:
        Matrix6d  J, J_DLS;
        Vector6d error;
        Eigen::Vector3d sigma_d;
        Vector6d ee_pose;
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
        void getJacobian(RowVector6d &J);
        void getError(double &error);

    private:
        RowVector6d  J, J_DLS;
        Eigen::Vector3d sigma_d;
        Vector6d q;
        std::vector<Eigen::Vector2d> jointlimits;
        bool active;
        Eigen::Vector3d joints;
        double dxe;
        

};

#endif