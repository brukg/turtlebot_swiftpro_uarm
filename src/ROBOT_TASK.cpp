#include "swift_uarm/ROBOT_TASK.h"
// #include "swift_uarm/utils.h"

MobileManipulator::MobileManipulator(){


    // sigma_d = 0;
    // _link_1 = 0.142; _link_2 = 0.1588; 
    // _vacuum_offset_x = 0.0565; _vacuum_offset_z = 0.0722;
    //  _base_offset_x = 0.0132; _base_offset_z = 0.108;
    // _is_mobile_base = false;
    r2b <<  0, -1, 0, 0.037, //manipulator to base transformation
            1, 0, 0, 0,
            0, 0, 1, 0.147,
            0, 0, 0, 1;
    _is_mobile_base = false;
    base_pose.setIdentity();

};


void MobileManipulator::setLinkValues(double &link_1, double &link_2, double &vacuum_offset_x, double &vacuum_offset_z, double &base_offset_x, double &base_offset_z){
    _link_1 = link_1;
    _link_2 = link_2;
    _vacuum_offset_x = vacuum_offset_x;
    _vacuum_offset_z = vacuum_offset_z;
    _base_offset_x = base_offset_x;
    _base_offset_z = base_offset_z;
};
void MobileManipulator::getEEJacobian(Matrix6d &J)
{
    getJacobian(this->joint_values, J);

};
void MobileManipulator::getPose(Eigen::Vector4d& ee_pose)
{
    forwardKinematics(joint_values, ee_pose);
};

void MobileManipulator::getBasePose(Eigen::Matrix4d& base_pose){
    base_pose = this->base_pose;
};
void MobileManipulator::setBasePose(Eigen::Matrix4d& base_pose){
    this->base_pose = base_pose;
};
void MobileManipulator::setJoints(Vector6d joints)
{
    this->joint_values = joints;
};
void MobileManipulator::getJoints(Vector6d &q)
{   
    q = this->joint_values;
};

void MobileManipulator::setMobile(bool& _is_mobile_base)
{
    this->_is_mobile_base = _is_mobile_base;
};

void MobileManipulator::forwardKinematics(Vector6d& joints, Eigen::Vector4d& ee_pose)
{
	//swiftpro uarm forward kinematics
    // cout<<"joints"<<joints<<endl;
    // Eigen::Vector2d eta;
    // // eta[1] = yaw;
    // eta[0] = joints[0];
    // eta[1] = joints[1];
    double b_yaw = atan2(r2b(1,0), r2b(0,0)); //yaw of the base to the arm 90 degrees
  
    Eigen::Vector4d ee_p;            
    
    ee_p(0) = (_link_1 * sin(joints[3])  + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * cos(joints[2]); //x arm position 

    ee_p(1) = (_link_1 * sin(joints[3]) + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * sin(joints[2]); //y arm position
	
    ee_p(2) = (_link_1 * cos(joints[3])  - _link_2 * sin(joints[4])) + _base_offset_z - _vacuum_offset_z; //z arm position
    
    ee_p(3) =  1;
    double yaw;  //yaw of eend effector is the sum of yaw of base and yaw of joints 0 and 3
    Eigen::Vector4d temp;  //using arm on mobile base

    // if(_is_mobile_base) //if using mobile base    
    if (_is_mobile_base) {
        temp = base_pose * r2b * ee_p; 
        yaw = joints[1] + b_yaw + joints[2] + joints[5];
        wrap_angle(yaw);
        ee_pose << temp[0], temp[1], temp[2], yaw; 
    }else { //if using fixed base //only arm on fixed base
    yaw = joints[2] + joints[5];
    ee_pose << ee_p[0], ee_p[1], ee_p[2], yaw;
    } 
};

void MobileManipulator::wrap_angle(double& a)
{
  while ((a) >  M_PI) a -= 2*M_PI;
  while ((a) < -M_PI) a += 2*M_PI;
};


void MobileManipulator::getJacobian(Vector6d& joints, Matrix6d &J)
{   
    double r_yaw = joints[1]; //atan2(base_pose(1,0), base_pose(0,0)); //yaw of the base in the world frame joints[1]
    double b_yaw = atan2(r2b(1,0), r2b(0,0)); //yaw of the base to the arm 90 degrees

    Eigen::Vector4d ee_p;            
    
    ee_p(0) = base_pose(0,3) + r2b(0,3)*cos(r_yaw) + (_link_1 * sin(joints[3])  + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * cos(b_yaw +joints[2]); //x arm position 

    ee_p(1) = base_pose(1,3) + r2b(0,3)*sin(r_yaw) +  (_link_1 * sin(joints[3]) + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * sin(b_yaw +joints[2]); //y arm position
	
    ee_p(2) = base_pose(2,3) + r2b(2,3) + (_link_1 * cos(joints[3])  - _link_2 * sin(joints[4])) + _base_offset_z - _vacuum_offset_z; //z arm position
    
    ee_p(3) =  1;

    double yaw = joints[1] + b_yaw + joints[2] + joints[5]; wrap_angle(yaw);


    double dx_j0 = cos(r_yaw );
    double dx_j1 = -r2b(0,3)*sin(r_yaw);
    double dx_j2 = -(_link_1 * sin(joints[3])  + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * sin(b_yaw +joints[2] );
    double dx_j3 = (_link_1 * cos(joints[3])) * cos(b_yaw + joints[2] );
    double dx_j4 = _link_2 * -sin(joints[4]) *  cos(b_yaw + joints[2]);
    double dx_j5 = 0;

    double dy_j0 = sin(r_yaw );
    double dy_j1 = r2b(0,3)*cos(r_yaw);
    double dy_j2 = (_link_1 * sin(joints[3])  + _link_2 * cos(joints[4]) + _base_offset_x + _vacuum_offset_x) * cos(b_yaw + joints[2]);   
    double dy_j3 = (_link_1 * cos(joints[3])) * sin(b_yaw + joints[2]); 
    double dy_j4 = _link_2 * -sin(joints[4]) *  sin(b_yaw + joints[2]);
    double dy_j5 = 0;
    
    double dz_j0 = 0;
    double dz_j1 = 0;
    double dz_j2 = 0;
    double dz_j3 = _link_1 * -sin(joints[3]);
    double dz_j4 = _link_2 * -cos(joints[4]);
    double dz_j5 = 0;

    double dyaw_j0 = 0;
    double dyaw_j1 = 1;
    double dyaw_j2 = 1;
    double dyaw_j3 = 0;
    double dyaw_j4 = 0;
    double dyaw_j5 = 1;
    J.setZero(4,6);

    J <<    dx_j0,      dx_j1,      dx_j2,      dx_j3,      dx_j4, dx_j5,
            dy_j0,      dy_j1,      dy_j2,      dy_j3,      dy_j4, dy_j5,
            dz_j0,      dz_j1,      dz_j2,      dz_j3,      dz_j4, dz_j5,
            dyaw_j0,    dyaw_j1,    dyaw_j2,    dyaw_j3,    dyaw_j4, dyaw_j5;
    // cout<<"J: "<<J<<endl;
};


void MobileManipulator::getDLS(Matrix6d &J, double lambda, Matrix6d &J_DLS, Matrix6d &W)
{
    J_DLS = W*J.transpose()* (J *W*J.transpose()  + pow(lambda, 2) * Matrix6d::Identity(4,4)).completeOrthogonalDecomposition().pseudoInverse();
};

void MobileManipulator::getDLS(RowVector6d &J, double lambda, Vector6d &J_DLS)
{
    J_DLS = J.transpose()/(J *J.transpose()  + pow(lambda, 2));
};




TASK::TASK(){


    // sigma_d = 0;

};
bool TASK::isActive(bool& is_active){
    return true;
};

void TASK::setK(double& K){
    this->K = K;
};
//
void TASK::update(MobileManipulator &robot){
};


void TASK::setDesired(){
};
void TASK::getDesired(){
};

void TASK::getError(){

};

void TASK::getK(double& K){
    K = this->K;
};
void TASK::setFF(double& FF){
    this->FF = FF;
};
void TASK::getFF(double& FF){ 
    FF = this->FF;
};

void TASK::euler_to_quaternion(){

};



/*
*
*/
Position::Position(){
    active = false;

};

//
void Position::update(MobileManipulator &robot){
    robot.getEEJacobian(this->J);
    robot.getPose(this->ee_pose);
    // getDesired(sigma_d);
    this-> error = sigma_d - ee_pose;
};

void Position::setDesired(Eigen::Vector4d &sigma_d){
    this-> sigma_d = sigma_d;
};

void Position::getDesired(Eigen::Vector4d &sigma_d){
    sigma_d = this->sigma_d;
};

bool Position::isActive(){
    return this->active;
};

void Position::getJacobian(Matrix6d &J)
{
    J = this->J;
};

void Position::getError(Eigen::Vector4d &error)
{

    error = this->error;
};


/*
* 
*/
JointLimits::JointLimits(){
    this->J = Vector6d::Zero(1,6);
    Eigen::Vector2d j0_lim, j1_lim, j2_lim;
    j0_lim << -1.5708, 1.5708;  j1_lim<< -0.469, 1.5708;  j2_lim<< 0, 1.3;
    // j0_lim << -1.519, 1.361;  j1_lim<< -0.469, 1.283;  j2_lim<< 0.064, 1.116;//1.75;

    this->jointlimits.push_back(j0_lim); //joint 0 limits -pi/2 to pi/2
    this->jointlimits.push_back(j1_lim); //joint 1 limits -pi/4 to pi/2
    this->jointlimits.push_back(j2_lim); //joint 2 limits 0 to 1.75 0 to 100deg 
    this->dxe = 0;
};

//
void JointLimits::update(MobileManipulator &robot, int j){
    // robot.getEEJacobian(this->J);
    J.setZero(1,6);
    this->J[j] = 1; 
    robot.getJoints(q);
    double _alpha = 0.1, _delta = 0.2;

    if (j = 4){
        if (dxe==0 && q[j]>=jointlimits[j-2][1]-_delta){this->dxe=-1; this->active=true;}
        if (dxe==0 && q[j]<=jointlimits[j-2][0]+_delta){this->dxe=1; this->active=true;}
        if (dxe==-1 && q[j]<=jointlimits[j-2][1]-_alpha){this->dxe=0; this->active=false;}
        if (dxe==1 && q[j]>=jointlimits[j-2][0]+_alpha){this->dxe=0; this->active=false;}
    }else{
        if (dxe==0 && q[j]>=jointlimits[j-2][1]-_alpha){this->dxe=-1; this->active=true;}
        if (dxe==0 && q[j]<=jointlimits[j-2][0]+_alpha){this->dxe=1; this->active=true;}
        if (dxe==-1 && q[j]<=jointlimits[j-2][1]-_delta){this->dxe=0; this->active=false;}
        if (dxe==1 && q[j]>=jointlimits[j-2][0]+_delta){this->dxe=0; this->active=false;}
    }
};
void JointLimits::setDesired(Eigen::Vector3d &sigma_d)
{
    this-> sigma_d = sigma_d;
};

void JointLimits::getDesired(Eigen::Vector3d &sigma_d)
{
    sigma_d = this->sigma_d;
};

void JointLimits::getJacobian(RowVector6d &J)
{
    J = this->J;
};

bool JointLimits::isActive(){
    return this->active;
};

void JointLimits::getError(double &error)
{
    error = this->dxe;

};





/*
* 
*/
JointsPosition::JointsPosition(){
    this->J = Vector6d::Zero(1,6);
    Eigen::Vector2d j0_lim, j1_lim, j2_lim;

};
//
void JointsPosition::update(MobileManipulator &robot, int j){
    // robot.getEEJacobian(this->J);
    this->J[j] = 1; 
    robot.getJoints(q);
    this-> error = this->sigma_d - q[j];


};

void JointsPosition::getError(double &error)
{
    error = this->error;

};
void JointsPosition::setDesired(double &sigma_d)
{
    this-> sigma_d = sigma_d;
};

void JointsPosition::getDesired(double &sigma_d)
{
    sigma_d = this->sigma_d;
};

void JointsPosition::getJacobian(RowVector6d &J)
{
    J = this->J;
};