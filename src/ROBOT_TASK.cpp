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
    _is_mobile_base = false;
    base_pose.setIdentity();

}

void MobileManipulator::getEEJacobian(Matrix6d &J)
{
    getJacobian(this->joint_values, J);

}
void MobileManipulator::getPose(Vector6d& ee_pose)
{
    forwardKinematics(joint_values, ee_pose);
}

void MobileManipulator::getBasePose(Eigen::Matrix4d& base_pose){
    base_pose = this->base_pose;
}
void MobileManipulator::setBasePose(Eigen::Matrix4d& base_pose){
    this->base_pose = base_pose;
}
void MobileManipulator::setJoints(Vector6d joints)
{
    this->joint_values = joints;
}
void MobileManipulator::getJoints(Vector6d &q)
{   
    q = this->joint_values;
}

void MobileManipulator::setMobile(bool& _is_mobile_base)
{
    this->_is_mobile_base = _is_mobile_base;
}
/*
* @brief get the forward kinematics of the arm
*/
void MobileManipulator::forwardKinematics(Vector6d& joints, Vector6d& ee_pose)
{
	//swiftpro uarm forward kinematics
    // cout<<"joints"<<joints<<endl;
    double horizontal = _link_1 * sin(joints[3]) 
				        + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x;

	double verical = _link_1 * cos(joints[3]) 
				    - _link_2 * sin(joints[4]);
    Eigen::Vector2d eta;
    // eta[1] = yaw;
    eta[0] = joints[1];
    eta[1] = joints[0];
    Eigen::Vector4d ee_p;            
    ee_p(0) = horizontal * cos(joints[2]); //x arm position 
	ee_p(1) = horizontal * sin(joints[2]); //y arm position
	ee_p(2) = verical + _base_offset_z - _vacuum_offset_z; //z arm position
    ee_p(3) =  1;
    double yaw = joints[2] + joints[5];
    Eigen::Vector4d temp;  //using arm on mobile base
    // cout<<"base_pose"<<base_pose<<endl;
    // cout<<"r2b"<<r2b<<endl;
    if (_is_mobile_base) {temp = base_pose * r2b * ee_p; ee_pose << 0, 0, temp[0], temp[1], temp[2], yaw;} //if using mobile base
    else ee_pose << 0, 0, ee_p[0], ee_p[1], ee_p[2], yaw; //only arm
	// cout<<"ee from fk"<< ee_pose;
}

/*
* @brief get the Jacobian of the arm
*/
void MobileManipulator::getJacobian(Vector6d& joints, Matrix6d &J)
{
    double r_yaw = atan2(base_pose(1,0), base_pose(0,0)); //yaw of the base in the world frame
    double b_yaw = atan2(r2b(1,0), r2b(0,0)); //yaw of the base to the arm

    //swiftpro uarm jacobian matrix
    double horizontal =  _link_1 * sin(joints[3]) 
				   + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x;

	double verical = _link_1 * cos(joints[3]) 
				  - _link_2 * sin(joints[4]) + _base_offset_z;
    double ee_yaw = joints[2] + joints[5];
    // if (_is_mobile_base) { 
    //     horizontal += base_pose(0,3); 
    //     horizontal += r2b(0,3); 
    //     verical += base_pose(2,3); 
    //     verical += r2b(2,3); 
    //     ee_yaw += b_yaw;
    // }
    // derivative of x with respect to joints
	double dx_j0 = horizontal * -sin(joints[0] + 1.57 + joints[2]);
    double dx_j1 = _link_1 * cos(joints[3]) * cos(joints[2]); 
	double dx_j2 =  _link_2 * -sin(joints[4]) * cos(joints[2]);
                    
    // derivative of y with respect to joints
	double dy_j0 = horizontal * cos(joints[0] + 1.57 + joints[2]);
	double dy_j1 = _link_1 * cos(joints[3] )  * sin(joints[2]); 
	double dy_j2 = _link_2 * -sin(joints[4])  * sin(joints[2]);

    // derivative of z with respect to joints
	double dz_j0 = 0;
	double dz_j1 = _link_1 * -sin(joints[3]); 
	double dz_j2 = -_link_2 * cos(joints[4]);


    // derivative of yaw with respect to joints
    double dyaw_j0 = 1;
    double dyaw_j1 = 0;
    double dyaw_j2 = 0;
    double dyaw_j3 = 1;
    Eigen::MatrixXd J_temp(3,2), J_temp_inv(2,3);
    J_temp << cos(joints(0)),   0,
              sin(joints(0)),   0,
              0,               1;
    // J_temp_inv = J_temp.transpose() * (J_temp * J_temp.transpose()).inverse();
	// cout<<"J_temp_inv"<<J_temp_inv<<endl;
    J << 0, 0, 0,       0,      0,      0,
         0, 0, 0,       0,      0,      0,
         0, 0, dx_j0,   dx_j1, dx_j2,   0,
         0, 0, dy_j0,   dy_j1, dy_j2,   0,
         0, 0, 0,       dz_j1, dz_j2,   0,
         0, 0, dyaw_j0, 0,      0,      dyaw_j3;

    // J.block<2,3>(0,0) = J_temp.inverse();
    // cout<<"J"<<J<<endl;

    if (_is_mobile_base) {

    }
}


void MobileManipulator::getDLS(Matrix6d &J, double lambda, Matrix6d &J_DLS)
{
    J_DLS = J.transpose()*(J *J.transpose()  + pow(lambda, 2) * Matrix6d::Identity()).inverse();
}

void MobileManipulator::getDLS(RowVector6d &J, double lambda, Vector6d &J_DLS)
{
    J_DLS = J.transpose()/(J *J.transpose()  + pow(lambda, 2));
}




TASK::TASK(){


    // sigma_d = 0;

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



/*
*
*/
Position::Position(){
    active = false;

}

//
void Position::update(MobileManipulator &robot){
    robot.getEEJacobian(this->J);
    robot.getPose(this->ee_pose);
    // getDesired(sigma_d);
    this-> error = sigma_d - ee_pose;
}

void Position::setDesired(Vector6d &sigma_d){
    this-> sigma_d = sigma_d;
}

void Position::getDesired(Vector6d &sigma_d){
    sigma_d = this->sigma_d;
}

bool Position::isActive(){
    return this->active;
}

void Position::getJacobian(Matrix6d &J)
{
    J = this->J;
}

void Position::getError(Vector6d &error)
{

    error = this->error;
}


/*
* 
*/
JointLimits::JointLimits(){
    this->J = Vector6d::Zero(1,6);
    Eigen::Vector2d j0_lim, j1_lim, j2_lim;
    j0_lim << -1.5708, 1.5708;  j1_lim<< -1.5708/4, 1.5708/2;  j2_lim<< 0, 1.75;

    this->jointlimits.push_back(j0_lim); //joint 0 limits -pi/2 to pi/2
    this->jointlimits.push_back(j1_lim); //joint 1 limits -pi/4 to pi/2
    this->jointlimits.push_back(j2_lim); //joint 2 limits 0 to 1.75 0 to 100deg 
    this->dxe = 0;
}
//
void JointLimits::update(MobileManipulator &robot){
    // robot.getEEJacobian(this->J);
    this->J[2] = 1; 
    robot.getJoints(q);
    // cout<<this->jointlimits<<endl;
    // cout<<"q "<<this->q<<endl;
    // cout<<"j "<<this->J<<endl;
    if (dxe==0 && q[2]>=jointlimits[0][1]-0.05){this->dxe=-1; this->active=true;}
    if (dxe==0 && q[2]<=jointlimits[0][0]+0.05){this->dxe=1; this->active=true;}
    if (dxe==-1 && q[2]<=jointlimits[0][1]-0.09){this->dxe=0; this->active=false;}
    if (dxe==1 && q[2]>=jointlimits[0][0]+0.09){this->dxe=0; this->active=false;}
}
void JointLimits::setDesired(Eigen::Vector3d &sigma_d)
{
    this-> sigma_d = sigma_d;
}

void JointLimits::getDesired(Eigen::Vector3d &sigma_d)
{
    sigma_d = this->sigma_d;
}
void JointLimits::getJacobian(RowVector6d &J)
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