#include <string>
#include "swift_uarm/CNTRL.h"


/*
* @brief get the forward kinematics of the arm
*/
void CNTRL::forwardKinematics(Eigen::Vector4d& joints, Eigen::Vector3d& ee_pose)
{
	//swiftpro uarm forward kinematics
    double horizontal = _link_1 * sin(joints[1]) 
				        + _link_2 * cos(joints[2]) + _base_offset_x + _vacuum_offset_x;

	double verical = _link_1 * cos(joints[1]) 
				    - _link_2 * sin(joints[2]);

    Eigen::Vector4d ee_p;            
    ee_p(0) = horizontal * cos(joints[0]);
	ee_p(1) = horizontal * sin(joints[0]);
	ee_p(2) = verical + _base_offset_z - _vacuum_offset_z;
    ee_p(3) = 1;
    Eigen::Vector4d temp = r2b * ee_p; //using arm on mobile base
    if (_is_mobile_base) ee_pose = temp.head(3);
    else ee_pose = ee_p.head(3); //only arm
	return;
}

/*
* @brief get the Jacobian of the arm
*/
void CNTRL::getJacobian(Eigen::Vector4d& joints, Eigen::Matrix3d &J)
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
    return;
}


void CNTRL::getDLS(Eigen::Matrix3d &J, double lambda, Eigen::Matrix3d &J_DLS)
{
    J_DLS = J.transpose()*(J *J.transpose()  + pow(lambda, 2) * Eigen::Matrix3d::Identity()).inverse();
    return;
}
