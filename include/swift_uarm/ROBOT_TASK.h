#ifndef TASK_H
#define TAsk_H
#include <vector>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
typedef Eigen::MatrixXd Matrix6d;
typedef Eigen::Matrix< double, 1, 6 > RowVector6d;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
// #include <tf2_ros/buffer.h>

class MobileManipulator
{
    public:
        MobileManipulator();
        ~MobileManipulator() {};
        /*
        * @brief  update the robot arm linkes
        */
        void setLinkValues(double &link_1, double &link_2, double &vacuum_offset_x, double &vacuum_offset_z, double &base_offset_x, double &base_offset_z);
        /*
        * @brief  get robot jacobian
        */
        void getEEJacobian(Matrix6d &J);
        /*
        * @brief  get EE pose
        */
        void getPose(Eigen::Vector4d& ee_pose);
        /*
        * @brief  get mobile pose
        */
        void getBasePose(Eigen::Matrix4d& base_pose);
        /*
        * @brief  set mobile base pose
        
        */
        void setBasePose(Eigen::Matrix4d& base_pose);

        /*
        * @brief  get DLS for arm
        */
        void getDLS(Matrix6d &J, double lambda, Matrix6d &J_DLS, Matrix6d &W); //DLS
        
        /*
        * @brief  get DLS for joints
        */
        void getDLS(RowVector6d &J, double lambda, Vector6d &J_DLS); //DLS

        /*
        * @brief  set Joints
        */
        void setJoints(Vector6d joints);
       
        /*
        * @brief  set Joints
        */
        void getJoints(Vector6d &q);
        /*
        * @brief  set mobile base as mobile or static
        */
        void setMobile(bool& is_active);

    private:
        /*
        * @brief get the forward kinematics of the arm
        */
        void forwardKinematics(Vector6d& joints, Eigen::Vector4d& ee_pose); 
        
        /*
        * @brief get the Jacobian of the arm
        */      
        void getJacobian(Vector6d& joints, Matrix6d &J); 
        
        void update();
        void getEETransform();
        void getJointPos( );
        void getDOF();
        void wrap_angle(double& a);
        // variables
        Vector6d joint_values;
        float _link_1, _link_2, _vacuum_offset_x, _vacuum_offset_z, _base_offset_x, _base_offset_z;
        Eigen::Matrix4d r2b; //robot to base transformation
        Eigen::Matrix4d base_pose;
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
        void getError(Eigen::Vector4d &error);
        bool isActive();
        void setDesired(Eigen::Vector4d &sigma_d);
        void getDesired(Eigen::Vector4d &sigma_d);

    private:
        Matrix6d  J, J_DLS;
        Eigen::Vector4d error, sigma_d, ee_pose;
        bool active;

};

class JointLimits : public TASK
{
    public:
        JointLimits();
        ~JointLimits() {};
        void update(MobileManipulator &robot, int j);
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
        double dxe, _alpha, _delta;
        

};

class JointsPosition : public TASK
{
    public:
        JointsPosition();
        ~JointsPosition() {};
        void update(MobileManipulator &robot, int j);
        bool isActive();
        void setDesired(double &sigma_d);
        void getDesired(double &sigma_d);
        void getJacobian(RowVector6d &J);
        void getError(double &error);

    private:
        RowVector6d  J, J_DLS;
        double sigma_d, error;
        Vector6d q;
        std::vector<Eigen::Vector2d> jointlimits;
        Eigen::Vector3d joints;
        

};

#endif