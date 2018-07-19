/* 
 * lpetrich 01/07/18
 */

#ifndef UVS_UTILITIES_H
#define UVS_UTILITIES_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include "wam_control/arm_control.h"
#include "wam_control/MatrixMN.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"


Eigen::VectorXd concatenate_vectorxd(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
{
    Eigen::VectorXd v(v1.rows() + v2.rows());
    v << v1, v2;
    return v;
}

std::vector<Eigen::Vector2d> slice(const std::vector<Eigen::Vector2d> &v, int start, int end)
{
    auto first = v.cbegin() + start;
    auto last = v.cbegin() + end + 1;

    std::vector<Eigen::Vector2d> subv(first, last);
    return subv;
}

/// Moore-Penrose pseudoinverse
/** Implementation taken from: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
 */
template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) *
                                               svd.singularValues().array().abs().maxCoeff();

    result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
             tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() *
             svd.matrixU().adjoint();
}

Eigen::VectorXd toEulerAngle(const Eigen::VectorXd& q)
{
    Eigen::VectorXd euler(3);
    double roll;
    double pitch;
    double yaw;
    // roll (x-axis rotation)
    double sinr = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    double cosr = +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    double cosy = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  
    yaw = atan2(siny, cosy);

    euler << roll, pitch, yaw;
    return euler;
}

Eigen::VectorXd state_to_vector(sensor_msgs::JointState js, int dof)
{
	Eigen::VectorXd joints(dof);
	for (int i = 0; i < dof; ++i) {
		joints[i] = js.position[i];
	}
	return joints;
}

std::string int_to_string(int i)
{
    std::string data;
    std::stringstream ss;
    ss << i << "\n"; 
    return data = ss.str();
}

std::string double_to_string(double d)
{
    std::string data;
    std::stringstream ss;
    ss << d << "\n"; 
    return data = ss.str();
}

std::string vector_to_string(Eigen::VectorXd v)
{
    std::string data;
    std::stringstream ss;
    for (int i = 0; i < v.size(); ++i) { 
        ss << v[i] << " "; 
    }
    ss << "\n";
    return data = ss.str();
}

std::string matrix_to_string(Eigen::MatrixXd m)
{
    std::string data;
    std::stringstream ss;
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            ss << m(i,j) << " "; 
        }
    }
    ss << "\n";
    return data = ss.str();
}

void log(std::string filename, std::string msg, bool write)
{
    std::cout << msg;
    if (write) { write_to_file(filename, msg); }
}

void log(std::string filename, std::string msg, int i, bool write)
{
    std::string s = int_to_string(i);
    std::cout << msg << s;
    if (write) { write_to_file(filename, msg + s); }
}

void log(std::string filename, std::string msg, double d, bool write)
{
    std::string s = double_to_string(d);
    std::cout << msg << s;
    if (write) { write_to_file(filename, msg + s); }
}

void log(std::string filename, std::string msg, Eigen::VectorXd v, bool write)
{
    std::string s = vector_to_string(v);
    std::cout << msg << s;
    if (write) { write_to_file(filename, msg + s); }
}

void log(std::string filename, std::string msg, Eigen::MatrixXd m, bool write)
{
    std::string s = matrix_to_string(m);
    std::cout << msg << s;
    if (write) { write_to_file(filename, msg + s); }
}

void log(std::string filename, std::string msg, wam_control::MatrixMN m, bool write)
{
    Eigen::VectorXd v;
    v = Eigen::Map<Eigen::VectorXd>(&m.data[0], m.data.size());
    std::string s = vector_to_string(v);
    std::cout << msg << s;
    if (write) { write_to_file(filename, msg + s); }
}

void log(std::string filename, std::string msg, sensor_msgs::JointState js, bool write)
{
    Eigen::VectorXd v;
    std::string v_str;
    v = Eigen::Map<Eigen::VectorXd>(&js.position[0], js.position.size());
    v_str += ("joint positions: " + vector_to_string(v));
    v = Eigen::Map<Eigen::VectorXd>(&js.velocity[0], js.velocity.size());
    v_str += ("joint velocities: " + vector_to_string(v));
    v = Eigen::Map<Eigen::VectorXd>(&js.effort[0], js.effort.size());
    v_str += ("joint efforts: " + vector_to_string(v));
    std::cout << msg << v_str;
    if (write) { write_to_file(filename, msg + v_str); }
}

void log(std::string filename, std::string msg, geometry_msgs::PoseStamped ps, bool write)
{
    Eigen::VectorXd vp(3);
    Eigen::VectorXd vq(4);
    std::string v_str;
    vp << ps.pose.position.x, ps.pose.position.y, ps.pose.position.z;
    v_str += ("tool position: " + vector_to_string(vp));
    vq << ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w;
    Eigen::VectorXd eul = toEulerAngle(vq);
    v_str += ("tool orientation quaternion: " + vector_to_string(vq));
    v_str += ("tool orientation euler: " + vector_to_string(eul));
    std::cout << msg << v_str;
    if (write) { write_to_file(filename, msg + v_str); }
}

void print_joint_position(sensor_msgs::JointState joints, std::string msg, int dof) 
{
	std::cout << msg << std::endl;
	for (int i = 0; i < dof; ++i) {
		std::cout << "Joint " << i + 1 << " position: " << joints.position[i] << std::endl;
	}
}

Eigen::VectorXd get_xy_error(ArmControl *arm)
{
	// return error between current and target xy position
    geometry_msgs::PoseStamped p = arm->get_pose();
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;
    Eigen::VectorXd xy(2);
    xy << x/z, y/z;
    std::cout << "xy: (" << xy[0] << ", " << xy[1] << ")" << std::endl;
    return xy;
}

Eigen::VectorXd get_xyz_error(ArmControl *arm)
{
	// return error between current and target xyz position
    geometry_msgs::PoseStamped p = arm->get_pose();
    Eigen::VectorXd xyz(3);
    xyz << p.pose.position.x, p.pose.position.y, p.pose.position.z;
    std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
    return xyz;
}

Eigen::VectorXd get_pose_error(ArmControl *arm)
{
	// return error between current and target pose
    geometry_msgs::PoseStamped p = arm->get_pose();
    Eigen::VectorXd fp(7);
    fp << p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w;
    // std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
    return fp;
}

Eigen::VectorXd get_image_error(ArmControl *arm)
{
	// return error between current and target pose
    geometry_msgs::PoseStamped p = arm->get_pose();
    Eigen::VectorXd fp(7);
    fp << p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w;
    // std::cout << "xyz: (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")" << std::endl;
    return fp;
}


Eigen::VectorXd vector_target(Eigen::VectorXd current_state, int idx, double delta)
{
    Eigen::VectorXd target_state(current_state.size());
    for (int i = 0; i < current_state.size(); ++i) {
        if (i != idx) { 
            target_state[i] = current_state[i]; 
        }
        else {
            target_state[i] = (current_state[i] + delta); 
        }
    }
    return target_state;
}

Eigen::VectorXd vector_target(Eigen::VectorXd current_state, std::vector<int> joints, double delta)
{
    Eigen::VectorXd target_state(current_state.size());
    for (int i = 0; i < current_state.size(); ++i) {
        for (int j = 0; j < joints.size(); ++j) {
            if (i == (joints[j] - 1)) {
                target_state[i] = (current_state[i] + delta);
            } else {
                target_state[i] = current_state[i]; 
            }
        }
    }
    return target_state;
}

#endif // UVS_UTILITIES_H