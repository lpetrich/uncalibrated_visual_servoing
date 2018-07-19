/* 
 * lpetrich 01/07/18
 */

#ifndef UVS_CONTROL_H
#define UVS_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <boost/timer.hpp>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "wam_control/misc_utilities.h"
#include "wam_control/arm_control.h"
#include "wam_control/bhand_control.h"
#include "uncalibrated_visual_servoing/Error.h"
#include "uncalibrated_visual_servoing/TrackPoint.h"
#include "uncalibrated_visual_servoing/uvs_utilities.h"

class UVSControl 
{
	public:
		ArmControl *arm;
		BHandControl *bhand;
		int dof;
		double image_tol = 100.0;
		int total_joints;
		double MAX_COND;
		std::string robot_namespace;
		std::string msg;
		std::string prefix;
		std::string filename;
		Eigen::VectorXd previous_eef_position;
		Eigen::VectorXd previous_joint_positions;
		Eigen::MatrixXd previous_jacobian;
		Eigen::MatrixXd initial_jacobian;
		Eigen::MatrixXd jacobian;
		Eigen::MatrixXd jacobian_inverse;
		std::vector<int> active_joints = {1, 1, 1, 1, 1, 1, 1};
		UVSControl(ros::NodeHandle nh);
		~UVSControl();
		Eigen::VectorXd calculate_delta_q();
		Eigen::VectorXd calculate_target(const Eigen::VectorXd& pos, const Eigen::VectorXd& delta);
		Eigen::VectorXd calculate_step(const Eigen::VectorXd& current_error_value, double lambda = 0.175);
		Eigen::VectorXd calculate_rampdown_and_endtime(const Eigen::VectorXd& delta, const Eigen::VectorXd& current_velocities);
		// bool limit_check(const Eigen::VectorXd& target); 
		void converge(double lambda = 0.175, double alpha = 1.0, double steps = 50.0, bool continous_motion = false);
		int move_step(double lambda = 0.175, bool continous_motion = false);
		bool broyden_update(double alpha = 1.0);
		bool jacobian_estimate(double perturbation_delta = 0.0875);
		void set_active_joints();
		void loop(); 
		void initialize();

	private:
		// Callbacks
		ros::Subscriber error_sub;
		ros::Subscriber eef_sub;
		Eigen::VectorXd image_error_vector;
		Eigen::VectorXd image_eef_pos;
		
		bool ready() {
			if (get_error().size() == 0 || get_eef_position().size() == 0) { 
				std::cout << "please initialize trackers" << std::endl;
				return false; 
			}
			else { return true; }
		}
		bool check_condition() {
		    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
		    double cond = (svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1));
		    log(filename, "condition number: ", cond, false);
		    if (cond > MAX_COND) { return true; } 
		    else { return false; }
		}

		Eigen::VectorXd get_error() { return image_error_vector; }
		Eigen::VectorXd get_eef_position() { return image_eef_pos; }
		
		void error_cb(uncalibrated_visual_servoing::Error::ConstPtr error) {
			uncalibrated_visual_servoing::Error current_error = *error;
			int sz = current_error.error.size();
	    	Eigen::VectorXd e(sz);
		    for(int i = 0; i < sz; ++i) { 
		    	e[i] = current_error.error[i]; 
		    }
		    image_error_vector = e;
		}
		
		void eef_cb(uncalibrated_visual_servoing::TrackPoint::ConstPtr eef) {
			uncalibrated_visual_servoing::TrackPoint current_eef = *eef;
			Eigen::VectorXd eef_pos(2);
			eef_pos[0] = current_eef.x; 
			eef_pos[1] = current_eef.y;
			image_eef_pos = eef_pos;
		}
};

#endif // UVS_CONTROL_H