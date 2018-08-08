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
#include "std_msgs/String.h"
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
		Eigen::VectorXd previous_delta_e;
		Eigen::VectorXd previous_joint_positions;
		Eigen::MatrixXd previous_jacobian;
		Eigen::MatrixXd initial_jacobian;
		Eigen::MatrixXd jacobian;
		Eigen::MatrixXd jacobian_inverse;
		std::vector<int> active_joints = {1, 1, 1, 1, 1, 1, 1};
		ros::Publisher dRt_command_pub;

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
		void publish_dRt_command(std::string s);

	private:
		// Callbacks
		ros::Subscriber dRt_sub;
		Eigen::VectorXd de;
		bool new_de;

		/// Moore-Penrose pseudoinverse - Implementation taken from: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
		template<typename _Matrix_Type_>
		bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
		{
			double max = 40.0;
		    Eigen::JacobiSVD<Eigen::MatrixXd> svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		    Eigen::VectorXd singular_values = svd.singularValues();
		    double cond = (singular_values(0) / singular_values(singular_values.size()-1));
		    // log(filename, "singular values: ", singular_values, false);
		    // log(filename, "condition number: ", cond, false);
		    if (cond > max) { 
			    singular_values(singular_values.size()-1) = 0;
			    double cond = (singular_values(0) / singular_values(singular_values.size()-2));
			    // std::cout << "condition number failed, setting smallest singular value to 0" << std::endl;
			    // log(filename, "new singular values: ", singular_values, false);
			    // log(filename, "new condition number: ", cond, false);
		    } 
		    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * singular_values.array().abs().maxCoeff();
		    result = svd.matrixV() * _Matrix_Type_( (singular_values.array().abs() >
		             tolerance).select(singular_values.array().inverse(), 0) ).asDiagonal() *
		             svd.matrixU().adjoint();
			log(filename, "jacobian: ", a, false);
			log(filename, "jacobian inverse: ", result, false);
		    return true;
		}

		bool ready() {
			if (get_dRt().size() == 0) { 
				std::cout << "waiting for delta e" << std::endl;
				return false; 
			}
			else { return true; }
		}

		Eigen::VectorXd get_dRt() {
			while (!new_de) { 
				//std::cout << "waiting for new delta e" << std::endl;
				continue; 
			}
			new_de = false;
			log(filename, "de: ", de, false);
			return de; 
		}
		
		void dRt_callback(std_msgs::String::ConstPtr msg) {
	    	std::vector<double> v;
	    	double d;
	    	std::stringstream ss;
	    	ss << msg->data.c_str();
	    	std::cout<<"i'm hrere 1"<<std::endl;
	    	while (ss >> d) {
	    		v.push_back(d);
	    	}
	    	Eigen::VectorXd v2(v.size());
	    	for (int i = 0; i < v.size(); ++i) {
	    		v2[i] = v[i];
	    		std::cout<<"v2"<<v[i]<<std::endl;
	    	}
			de = v2;
			std::cout<<"de assigned"<<std::endl;
			new_de = true;
		}
};

#endif // UVS_CONTROL_H
