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
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "wam_control/misc_utilities.h"
#include "wam_control/arm_control.h"
#include "wam_control/bhand_control.h"
#include "uncalibrated_visual_servoing/Error.h"
#include "uncalibrated_visual_servoing/TrackPoint.h"
#include "uncalibrated_visual_servoing/EndEffectorPoints.h"
#include "uncalibrated_visual_servoing/uvs_utilities.h"

class UVSControl 
{
	public:
		ArmControl *arm;
		BHandControl *bhand;
		bool reset;
		bool ready_to_grasp;
		bool stereo_vision;
		int dof;
		int num_active_joints;
		double image_tol;
		double default_lambda;
		double lambda;
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
		Eigen::VectorXd projected_delta_q(const Eigen::VectorXd& delta_q);
		Eigen::VectorXd calculate_delta_q();
		Eigen::VectorXd calculate_target(const Eigen::VectorXd& pos, const Eigen::VectorXd& delta);
		Eigen::VectorXd calculate_step(const Eigen::VectorXd& current_error_value);
		Eigen::VectorXd calculate_rampdown_and_endtime(const Eigen::VectorXd& delta, const Eigen::VectorXd& current_velocities);
		bool convergence_check(const Eigen::VectorXd& current_error);
		Eigen::MatrixXd control_plane_vectors(Eigen::VectorXd & delta_q);
		void converge(double alpha, int max_iterations, bool continous_motion);
		int move_step(bool continous_motion);
		bool broyden_update(const Eigen::VectorXd& dy, double alpha);
		bool jacobian_estimate(double perturbation_delta);
		void set_active_joints();
		void loop(); 
		void initialize();

	private:
		// Callbacks
		ros::Subscriber error_sub;
		ros::Subscriber eef_sub;
		ros::Subscriber reset_sub;
		ros::Subscriber move_sub;
		Eigen::VectorXd singular_values;
		Eigen::VectorXd image_error_vector;
		Eigen::VectorXd current_eef;
		Eigen::VectorXd delta_y;
		bool new_error;
		bool new_eef;

		bool ready() {
			if (get_error().size() == 0 || get_eef_position().size() == 0) { 
				std::cout << "please initialize trackers" << std::endl;
				return false; 
			}
			else { return true; }
		}

		/// Moore-Penrose pseudoinverse - Implementation taken from: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
		template<typename _Matrix_Type_>
		bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
		{
			double max = 40.0;
		    Eigen::JacobiSVD<Eigen::MatrixXd> svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		    Eigen::VectorXd singular_values = svd.singularValues();
		    double cond = (singular_values(0) / singular_values(singular_values.size()-1));
		    log(filename, "singular values: ", singular_values, false);
		    log(filename, "condition number: ", cond, false);
		    if (cond > max) { 
			    singular_values(singular_values.size()-1) = 0;
			    double cond = (singular_values(0) / singular_values(singular_values.size()-2));
			    std::cout << "condition number failed, setting smallest singular value to 0" << std::endl;
			    log(filename, "new singular values: ", singular_values, false);
			    log(filename, "new condition number: ", cond, false);
		    } 
		    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * singular_values.array().abs().maxCoeff();
		    result = svd.matrixV() * _Matrix_Type_( (singular_values.array().abs() >
		             tolerance).select(singular_values.array().inverse(), 0) ).asDiagonal() *
		             svd.matrixU().adjoint();
		    return true;
		}

		Eigen::VectorXd get_error() 
		{ 
			return image_error_vector; 
		}
		
		Eigen::VectorXd get_eef_position() 
		{ 
			return current_eef; 
		}

		Eigen::VectorXd get_dy() 
		{ 
			return delta_y; 
		}
		
		void error_cb(uncalibrated_visual_servoing::Error::ConstPtr error) {
			uncalibrated_visual_servoing::Error current_error = *error;
			int sz = current_error.error.size();
	    	Eigen::VectorXd e(sz);
		    for(int i = 0; i < sz; ++i) { 
		    	e[i] = current_error.error[i]; 
		    }
		    image_error_vector = e;
		}
		
		void eef_cb(uncalibrated_visual_servoing::EndEffectorPoints::ConstPtr data) {
			uncalibrated_visual_servoing::EndEffectorPoints eef = *data;
			Eigen::VectorXd eef_vector(eef.points.size() * 2);
			int j = 0;
			for (int i = 0; i < eef.points.size(); ++i) {
				eef_vector[j] = eef.points[i].x; 
				eef_vector[j+1] = eef.points[i].y;
				j += 2;
			}
			current_eef = eef_vector;
			try { 
				delta_y = current_eef - previous_eef_position;
			} catch (...) { std::cout << "dy calculation failed" << std::endl; }
		}

		void reset_cb(std_msgs::Bool data) {
			bool b = data.data;
			if (b) { 
				std::cout << "UVS: reset received" << std::endl;
				reset = true; 
			}
		}
};

#endif // UVS_CONTROL_H
