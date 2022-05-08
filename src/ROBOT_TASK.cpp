#include "swift_uarm/ROBOT_TASK.h"
// #include "swift_uarm/utils.h"

MobileManipulator::MobileManipulator(){


    // sigma_d = 0;
    _link_1 = 0.142; _link_2 = 0.1588; 
    _vacuum_offset_x = 0.0565; _vacuum_offset_z = 0.0722;
     _base_offset_x = 0.0132; _base_offset_z = 0.108;
    _is_mobile_base = false;
    r2b <<  0, -1, 0, 0.037, //manipulator to base transformation
            1, 0, 0, 0,
            0, 0, 1, 0.147,
            0, 0, 0, 1;

}


TASK::TASK(){


    // sigma_d = 0;

}

Position::Position(){
    active = false;

}
JointLimits::JointLimits(){
    this->J = Eigen::Vector3d::Zero(1,3);
    Eigen::Vector2d j0_lim, j1_lim, j2_lim;
    j0_lim << -1.5708, 1.5708;  j1_lim<< -1.5708/4, 1.5708/2;  j2_lim<< 0, 1.75;

    this->jointlimits.push_back(j0_lim); //joint 0 limits -pi/2 to pi/2
    this->jointlimits.push_back(j1_lim); //joint 1 limits -pi/4 to pi/2
    this->jointlimits.push_back(j2_lim); //joint 2 limits 0 to 1.75 0 to 100deg 
    this->dxe = 0;
}

void MobileManipulator::getEEJacobian(Eigen::Matrix3d &J)
{
    getJacobian(this->joint_values, J);

}
void MobileManipulator::getPose(Eigen::Vector3d& ee_pose)
{
    forwardKinematics(joint_values, ee_pose);
}


void MobileManipulator::setJoints(Eigen::Vector4d joints)
{
    this->joint_values = joints;
}
void MobileManipulator::getJoints(Eigen::Vector3d &q)
{   
    q = this->joint_values.head(2);
}
/*
* @brief get the forward kinematics of the arm
*/
void MobileManipulator::forwardKinematics(Eigen::Vector4d& joints, Eigen::Vector3d& ee_pose)
{
	//swiftpro uarm forward kinematics
    // cout<<"joints"<<joints<<endl;
    double horizontal = _link_1 * sin(joints[1]) 
				        + _link_2 * cos(joints[2]) + _base_offset_x + _vacuum_offset_x;

	double verical = _link_1 * cos(joints[1]) 
				    - _link_2 * sin(joints[2]);

    Eigen::Vector4d ee_p;            
    ee_p(0) = horizontal * cos(joints[0]);
	ee_p(1) = horizontal * sin(joints[0]);
	ee_p(2) = verical + _base_offset_z - _vacuum_offset_z;
    ee_p(3) = 1;
    double yaw = joints[0] + joints[3];
    Eigen::Vector4d temp = r2b * ee_p; //using arm on mobile base
    if (_is_mobile_base) ee_pose = temp.head(3);
    else ee_pose = ee_p.head(3); //only arm
	// cout<<"ee from fk"<< ee_pose;
}

/*
* @brief get the Jacobian of the arm
*/
void MobileManipulator::getJacobian(Eigen::Vector4d& joints, Eigen::Matrix3d &J)
{
    //swiftpro uarm jacobian matrix
    double horizontal = _link_1 * sin(joints[1]) 
				   + _link_2 * cos(joints[2]) + _base_offset_x + _vacuum_offset_x;

	double verical = _link_1 * cos(joints[1]) 
				  - _link_2 * sin(joints[2]) + _base_offset_z;

    // derivative of x with respect to joints
	double dx_j0 = horizontal * -sin(joints[0]);
    double dx_j1 = _link_1 * cos(joints[1]) * cos(joints[0]); 
	double dx_j2 =  _link_2 * -sin(joints[2]) * cos(joints[0]);
                    
    // derivative of y with respect to joints
	double dy_j0 = horizontal * cos(joints[0]);
	double dy_j1 = _link_1 * cos(joints[1] )  * sin(joints[0]); 
	double dy_j2 = _link_2 * -sin(joints[2])  * sin(joints[0]);

    // derivative of z with respect to joints
	double dz_j0 = 0;
	double dz_j1 = _link_1 * -sin(joints[1] ); 
	double dz_j2 = -_link_2 * cos(joints[2]);
	
    J << dx_j0, dx_j1, dx_j2,
         dy_j0, dy_j1, dy_j2,
         dz_j0, dz_j1, dz_j2;
}


void MobileManipulator::getDLS(Eigen::Matrix3d &J, double lambda, Eigen::Matrix3d &J_DLS)
{
    J_DLS = J.transpose()*(J *J.transpose()  + pow(lambda, 2) * Eigen::Matrix3d::Identity()).inverse();
}

void MobileManipulator::getDLS(Eigen::RowVector3d &J, double lambda, Eigen::Vector3d &J_DLS)
{
    J_DLS = J.transpose()/(J *J.transpose()  + pow(lambda, 2));
}


bool TASK::isActive(bool& is_active){
    return true;
}

void TASK::setK(double& K){
    this->K = K;
}
//
void TASK::update(MobileManipulator &robot){
}


void TASK::setDesired(){
}
void TASK::getDesired(){
}

void TASK::getError(){

}

void TASK::getK(double& K){
    K = this->K;
}
void TASK::setFF(double& FF){
    this->FF = FF;
}
void TASK::getFF(double& FF){ 
    FF = this->FF;
}

void TASK::euler_to_quaternion(){

}




//
void Position::update(MobileManipulator &robot){
    robot.getEEJacobian(this->J);
    robot.getPose(ee_pose);
    getDesired(sigma_d);
    // cout<<"ee_pose"<<ee_pose<<endl;
    error = sigma_d - ee_pose;
    // cout<<"error"<<error<<endl;
}

void Position::setDesired(Eigen::Vector3d &sigma_d){
    this-> sigma_d = sigma_d;
}

void Position::getDesired(Eigen::Vector3d &sigma_d){
    sigma_d = this->sigma_d;
}

bool Position::isActive(){
    return this->active;
}

void Position::getJacobian(Eigen::Matrix3d &J)
{
    J = this->J;
}

void Position::getError(Eigen::Vector3d &error)
{

    error = this->error;
}
//
void JointLimits::update(MobileManipulator &robot){
    // robot.getEEJacobian(this->J);
    this->J[0] = 1; 
    robot.getJoints(q);
    // cout<<this->jointlimits<<endl;
    cout<<"q"<<this->q<<endl;
    cout<<"j"<<this->J<<endl;
    if (dxe==0 && q[0]>=jointlimits[0][1]-0.05){this->dxe=-1; this->active=true;}
    if (dxe==0 && q[0]<=jointlimits[0][0]+0.05){this->dxe=1; this->active=true;}
    if (dxe==-1 && q[0]<=jointlimits[0][1]-0.09){this->dxe=0; this->active=false;}
    if (dxe==1 && q[0]>=jointlimits[0][0]+0.09){this->dxe=0; this->active=false;}
}
void JointLimits::setDesired(Eigen::Vector3d &sigma_d)
{
    this-> sigma_d = sigma_d;
}

void JointLimits::getDesired(Eigen::Vector3d &sigma_d)
{
    sigma_d = this->sigma_d;
}
void JointLimits::getJacobian(Eigen::RowVector3d &J)
{
    J = this->J;
}

bool JointLimits::isActive(){
    return this->active;
}

void JointLimits::getError(double &error)
{
    error = this->dxe;

}