/* 
 * lpetrich 27/06/18
 */

#include "uncalibrated_visual_servoing/uvs_control.h"

UVSControl::UVSControl(ros::NodeHandle nh_) 
{
	dof = 0;
	num_active_joints = 0;
	image_tol = 100.0;
	default_lambda = 0.15;
	new_de = false;
	prefix = "/home/froglake/irl_workspace/src/irl_experiments/log_data/";
	filename = prefix + current_time() + ".txt";
	arm = new ArmControl(nh_);
	// Get DOF of arm
	do {
		ROS_INFO_STREAM("Waiting to find robot DOF");
		ros::Duration(1.0).sleep();
		dof = arm->get_dof();
	} while (dof == 0);
	if (dof == 7) {
		num_active_joints = 7;
		bhand = new BHandControl("/zeus", nh_);
		bhand->set_spread_velocity(25);
		bhand->set_grasp_velocity(60);
	} else if (dof == 4) {
		num_active_joints = 4;
	} else {
		ROS_WARN_STREAM("Invalid DOF, reset and try again");
		exit(EXIT_FAILURE);
	}
	ROS_INFO_STREAM("Robot has " << dof << " DOF");
	// check for stereovision
	ros::V_string nodes;
	ros::master::getNodes(nodes);
	for (int i = 0; i < nodes.size(); i++) {
		std::string prefix = "/cam2";
		if(nodes[i].substr(0, prefix.size()) == prefix) {
			std::cout << "UVS: Found 2 cameras" << std::endl;
			stereo_vision = true;
			break;
		} 
	}
	if (!stereo_vision) { std::cout << "UVS: Found 1 camera" << std::endl; }
	dRt_sub = nh_.subscribe("/dRt", 1, &UVSControl::dRt_callback, this);
    dRt_command_pub = nh_.advertise<std_msgs::String>("/dRt_CMD", 10);
}

UVSControl::~UVSControl() 
{ // shutdown ROS subscribers and publishers properly
	dRt_sub.shutdown();
	dRt_command_pub.shutdown();
}

Eigen::VectorXd UVSControl::calculate_delta_q()
{ // calculates the actual motion change in joint space to use in Broyden's update
	Eigen::VectorXd total_dq;
	Eigen::VectorXd dq(num_active_joints);
	Eigen::VectorXd current_joint_positions = arm->get_positions();
	total_dq = current_joint_positions - previous_joint_positions;
	int j = 0;
	for (int i = 0; i < dof; ++i) {
		if (active_joints[i]) {
			dq[j] = total_dq[i];
			j++;
		}
	}
	return dq;
}

Eigen::VectorXd UVSControl::calculate_target(const Eigen::VectorXd& current_state, const Eigen::VectorXd& delta)
{ // calculates target vector in joint space to move to with given delta added to active joints
	Eigen::VectorXd target_state(dof);
	int j = 0;
	for (int i = 0; i < dof; ++i) {
		if (active_joints[i]) {
			target_state[i] = (current_state[i] + delta[j]);
			j++;
		} else {
			target_state[i] = current_state[i]; 
		}
	}
	return target_state;
}

bool UVSControl::convergence_check(const Eigen::VectorXd& current_error)
{ // should we make lambda larger as we get closer to the target? Test
	double pixel_step_size = 30.0;
	double n = current_error.norm();

	if (n < image_tol) {
		std::cout << "current error norm is less than image tolerance -- we have arrived at our destination" << std::endl;
		return true;
	} 
	lambda = std::max(0.1, pixel_step_size / n);
	if ((1.0 - lambda) * n < image_tol) { 
		lambda = 1.0 - image_tol / n;
		std::cout << "Next move places EEF too close to target - adjusting lambda to " << lambda << std::endl;
	} 
	std::cout << "current error norm: " << n << std::endl;
	std::cout << "lambda: " << lambda << std::endl;
	return false;
}

bool UVSControl::broyden_update(const Eigen::VectorXd& de, double alpha)
{ // update jacobian 
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd dq;
	double dq_norm;
	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm == 0) { return false; } // return early to avoid dividing by zero
	std::cout << (dq.transpose() * dq) << " ==? " << (dq_norm * dq_norm) << std::endl;
    update = ((de - jacobian * dq) * dq.transpose()) / (dq.transpose() * dq);
	// update = ((((-de) - jacobian * dq)) * dq.transpose()) / (dq_norm * dq_norm);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	// previous_eef_position = get_eef_position();
	log(filename, "broyden update: ", update, false);
	log(filename, "new jacobian: ", jacobian, false);
	return true;
}

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd& current_error_value) 
{ // calculates new motion step to take with the control law: step = −λJ+e 
	Eigen::VectorXd step;
	step = -lambda * (jacobian_inverse * current_error_value).transpose();
	// step = projected_delta_q(step);
	return step;
}

int UVSControl::move_step(bool continous_motion)
{ // step through one loop of VS
	Eigen::VectorXd delta_e;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;
	Eigen::VectorXd predicted_times;
	Eigen::VectorXd current_velocity;
	double sleep_time;
	// grab and use current error, check for convergence
	// current_error = get_error();
	delta_e = get_dRt();
	if (convergence_check(delta_e)) {return 0;}
	step_delta = calculate_step(delta_e);
	// grab and use current joint positions, check if valid
	current_joint_positions = arm->get_positions();
	target_position = calculate_target(current_joint_positions, step_delta);
	if (!limit_check(target_position, dof)) { return 1; }
	// write to screen for debugging
	log(filename, "delta_e: ", delta_e, false);
	log(filename, "previous_joint_positions: ", previous_joint_positions, false);
	log(filename, "current_joint_positions: ", current_joint_positions, false);
	log(filename, "step delta: ", step_delta, false);
	log(filename, "target position: ", target_position, false);
	previous_joint_positions = current_joint_positions;
	arm->call_move_joints(target_position, false);
	// check for continuous motion and adjust sleep times accordingly
	// if (continous_motion) {
	// 	sleep_time = std::min(1.0, std::max(0.3, (predicted_times[1] + predicted_times[2]) * 0.5)); // range between [0.3, 1.0]
	// } else {
	// 	sleep_time = 1.0;
	// }
	// std::cout << "// sleep time: " << sleep_time << std::endl;
	ros::Duration(0.3).sleep();
	return 2;
}

void UVSControl::converge(double alpha, int max_iterations, bool continous_motion)
{
	int c;
	Eigen::VectorXd delta_e;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < max_iterations; ++i) {
		std::cout << "iteration: " << i << std::endl;
		ros::Time begin = ros::Time::now();
		c = move_step(continous_motion);
		switch (c) {
			case 0: // convergence - return early
				return;
			case 1: // joints out of limit, reset jacobian
				jacobian = previous_jacobian;
				log(filename, "target not within joint limits, resetting jacobian to: ", jacobian, false);
				break;
			case 2: // step completed successfully
			// experiment with what is a good step size given the new error vector
				delta_e = get_dRt();
				log(filename, "delta_e norm: ", delta_e.norm(), false);
				if (delta_e.norm() > 30) {
					std::cout << "delta_e large enough, performing broyden update" << std::endl;
					if (broyden_update(delta_e, alpha) && !pseudoInverse(jacobian, jacobian_inverse)) { 
						jacobian = previous_jacobian;
						log(filename, "resetting jacobian to: ", jacobian, false);
					}
				}
				break;
		}
		std::cout << "loop duration: " << ros::Time::now() - begin << "\n**************************************" << std::endl;
	}
}

void UVSControl::set_active_joints()
{ // set which joints to use according to users input
	for (int i = 0; i < dof; ++i) { 
		active_joints[i] = 0; 
	}
	std::string line;
	std::cout << "Which joints would you like to use? >> ";
	std::getline(std::cin, line);
	std::istringstream ss(line); 
	int n;
	num_active_joints = 0;
	while (ss >> n) {
		active_joints[n-1] = 1;
		num_active_joints += 1;
	}
}

bool UVSControl::jacobian_estimate(double perturbation_delta) 
{ // perturb each active joint for the initial jacobian estimation
	publish_dRt_command("0");
	ros::Duration(0.5).sleep();
	Eigen::VectorXd delta_e;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	publish_dRt_command("1");
	ros::Duration(0.5).sleep();
	int sz = get_dRt().size();
	jacobian.resize(sz, num_active_joints);
	initial_jacobian.resize(sz, num_active_joints);
	jacobian_inverse.resize(num_active_joints, sz);
	int j = 0;
	std::string s;
	for (int i = 0; i < dof ; ++i) {
		if (active_joints[i]) {
			std::getline(std::cin, s);
			ros::Duration(0.2).sleep();
			position = arm->get_positions();
			target = vector_target(position, i, perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			publish_dRt_command("1");
			delta_e = get_dRt();
			ros::Duration(0.2).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = delta_e / perturbation_delta;
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_delta_e = delta_e;
	if (!pseudoInverse(jacobian, jacobian_inverse)) { 
		std::cout << "Initial jacobian estimate failed -- condition number too large" << std::endl;
		return false; 
	}
	return true;
}

Eigen::MatrixXd UVSControl::control_plane_vectors(Eigen::VectorXd & delta_q)
{
	/*
		Returns a dof x 2 matrix, with columns that define the VS control plane.

		Input: 
			delta_q:  next VS step in joint manifold tangent space (dof-vector)(assumed to be a small step, for local linearity purposes)

			//TODO theta: desired rotation of the plane about the delta_q vector, with respect to the vertical axis in the robot base frame.

		Output:
			control_plane: a dof x 2 matrix with elements in the joint manifold tangent space. 
			The second column is the change in robot joint coordinates from the VS step. Using the tool jacobian to transform this to a direction in robot base
			coordinates (and normalizing), this defines the cartesian 3-vector that we will define our plane around, e_1.
			If we take the cross product of e1 with the vertical vector (0, 0, 1)^T, e1 x (0, 0, 1)^T and then rotate about e1 by an angle of (theta - pi/2), 
			we obtain e2. 

		cstephens 23/07/2018
	*/
	Eigen::MatrixXd control_vectors;
	Eigen::MatrixXd jacobian(3, dof); 
	Eigen::MatrixXd jacobian_inv(dof, 3);
	//TODO Eigen::MatrixXd rotation_mat;
	Eigen::VectorXd d_q_1;
	Eigen::Vector3d d_x_1;
	Eigen::Vector3d d_x_2;
	Eigen::Vector2d e1(1.0, 0.0);
	Eigen::Vector2d e2(0.0, 1.0);
	Eigen::Vector3d vertical(0, 0.0, 1.0);

	jacobian = arm->get_lin_tool_jacobian();
	pseudoInverse(jacobian, jacobian_inv);
	delta_q *= 0.01/delta_q.norm(); // set delta_q norm to be small, here 0.01
	d_x_2 = jacobian * delta_q;
	d_x_1 = d_x_2.cross(vertical); 
	d_x_1 *= d_x_2.norm()/d_x_1.norm();
	pseudoInverse(jacobian, jacobian_inv);
	d_q_1 = jacobian_inv * d_x_1;
	control_vectors = d_q_1 * e1.transpose() + delta_q * e2.transpose();
	return control_vectors;
}

Eigen::VectorXd UVSControl::calculate_rampdown_and_endtime(const Eigen::VectorXd& delta, const Eigen::VectorXd& current_velocities)
{ // Code from libbarrett copied here and refactored to determine rampDown time and end time
	// First element of vector is start of plateau (negative is no plateau), second element is ramp down time, last element is predicted end time.
	// cstephens ??/07/2018
	double temp_time_start_down;
	double temp_time_end;
	double temp_time_start_plateau;
	double time_end = -10.0;
	double time_startdown = -10.0;
	double time_start_plateau = -10.0;
	double length;
	double v_init;
	double acc = 0.5;
	double vel = 0.5;
	double v_diff;
	int sign;
	Eigen::VectorXd times(3);
	for (int i = 0; i < delta.size(); ++i) {
		length = fabs(delta[i]);
		sign = (int) delta[i]/fabs(delta[i]);
		v_init = sign*current_velocities[i];
	    /* First, is the length too short to decellerate to stop?
	     * |\
	     * | \
	     * |  \__ */
	    if ( ( length < 0.5 * v_init * v_init / acc )  && v_init > 0.0 ) {	
		      /* There are no up or plateau phases */
	    	time_start_plateau = -1.0;
	        temp_time_start_down = 0.0;
	        temp_time_end = 2 * length / v_init;
	        if ( temp_time_start_down > time_startdown) {
	        	time_startdown = temp_time_start_down; 
	        	time_start_plateau = temp_time_start_plateau;
	        	time_end = temp_time_end;
	     	}
	     	continue;
	    }    
		/* OK, do we not have enough space to plateau?
		 * |
		 * |/\
		 * |  \__ */
		v_diff = vel - v_init;
		if ( length < (0.5*vel*vel + 0.5*v_diff*v_diff + v_init*v_diff)/acc ) {
			double v_top;
			v_top = sqrt( length * acc + 0.5 * v_init * v_init );
			temp_time_start_plateau = -1.0;
		    temp_time_start_down = (v_top - v_init) / acc;
		    temp_time_end = temp_time_start_down + v_top / acc;
		    if ( temp_time_start_down > time_startdown) {
		        time_startdown = temp_time_start_down; 
		        time_start_plateau = temp_time_start_plateau;
		        time_end = temp_time_end;
		    }
		    continue;
		} 
	    /* OK, we're going to plateau, either up or down
	     * | __        |\__
	     * |/  \    or |   \
	     * |    \__    |    \__ */
	    double time_endup = fabs(v_diff) / acc;
	    double s_endup = time_endup * (vel + v_init) / 2;
	    /* Compute the ramp down portion */
	    double s_startdown = length - 0.5 * vel * vel / acc;
	    temp_time_start_down = time_endup + (s_startdown - s_endup) / vel;
	    temp_time_end = time_startdown + vel / acc;
	    temp_time_start_plateau = time_endup;
	    if ( temp_time_start_down > time_startdown) {
	        time_startdown = temp_time_start_down;
        	time_start_plateau = temp_time_start_plateau;
	        time_end = temp_time_end;
	    }
   	}
   	times[0] = time_start_plateau;
    times[1] = time_startdown;
    times[2] = time_end;
    return times;
}


Eigen::VectorXd UVSControl::projected_delta_q(const Eigen::VectorXd& delta_q)
{
	/*
	Returns the projection of delta_q that only allows eef rotation about the vertical axis in the base frame.
	Input: 
		delta_q:  next VS step in joint manifold tangent space (dof-vector)
	Output:
		projected_delta_q, obtained by first obtaining the null space (or kernel) of the angular tool jacobian in the x and y directions, and then
		projecting delta_q onto the range of this kernel. 
	cstephens 24/07/2018
	*/		
	Eigen::MatrixXd ang_jacobian;
	Eigen::MatrixXd reduced_ang_jacobian;
	
	Eigen::MatrixXd kernel;
	Eigen::MatrixXd projection_matrix;
	Eigen::VectorXd projected_delta_q;
	ang_jacobian = arm->get_ang_tool_jacobian();
	reduced_ang_jacobian.resize(ang_jacobian.rows()-1, ang_jacobian.cols());
	reduced_ang_jacobian.row(0) = ang_jacobian.row(0);
	reduced_ang_jacobian.row(1) = ang_jacobian.row(1);
	Eigen::FullPivLU<Eigen::MatrixXd> lu(reduced_ang_jacobian);
	kernel = lu.kernel();
	projection_matrix = kernel * (kernel.transpose() * kernel ).inverse() * kernel.transpose();
	projected_delta_q = projection_matrix * delta_q;
	return projected_delta_q;
}


void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	bool continous_motion = true;
	double perturbation_delta = 0.0349066;
	// double perturbation_delta = 0.0875;
	double alpha = 1.0; // update rate
	int max_iterations = 25;
	double d;
	int c;
	Eigen::VectorXd start_position(7);
	start_position << 0.21147, 0.768076, -0.0682357, 1.73757, -0.782726, -0.637551, 0.542801; 
	std::string line;
	std::string s;
	lambda = default_lambda; // convergence rate
	while (ros::ok() && !exit_loop) {
		std::cout << "************************************************************************************************" <<
			"\nSelect option:" <<
			"\n\te: Move to experiment starting position" <<
			"\n\tp: Lock joint position" <<
			"\n\tu: Unlock joint position" <<
			"\n\td: Set Jacobian delta movement (current = " << perturbation_delta << ")" <<
			"\n\tl: Set step convergence lambda value (current = " << lambda << ")" <<
			"\n\ta: Set alpha value for broyden update (current = " << alpha << ")" <<
			"\n\tt: Set image_tol - prevents collision in image space (current = " << image_tol << ")" <<
			"\n\tc: Set max iteration for convergence (current = " << max_iterations << ")" <<
			"\n\tm: Set continuous motion (current = " << continous_motion << ")" <<
			"\n\tj: Compute Jacobian" <<
			"\n\tx: Compute Jacobian with chosen joints" <<
			"\n\tv: Complete VS convergence with set max iterations " << 
			"\n\ts: Compute and move one step" <<
			"\n\ti: Move to initial position" <<
			"\n\th: Move to home position" <<
			"\n\to: Open grasp" <<
			"\n\tg: Close grasp" <<
			"\n\tz: Close spread" <<
			"\n\tn: Start new logging file" <<
			"\n\tq: quit" <<
			"\n\t>> " << std::endl;
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'e':
			arm->call_move_joints(start_position, true);
			break;
		case 'm':
			s = string_input("Would you like to set motion to continuous or non-continuous?"); // wam_control --> misc_utilities.h
			if (s[0] == 'c' || s[0] == 'C') { continous_motion = true; } 
			else { continous_motion = false; }
			std::cout << "continuous motion set to " << BoolToString(continous_motion) << std::endl; // wam_control --> misc_utilities.h
			break;
		case 'p':
			arm->lock_joint_position(true);
			break;
		case 'u':
			arm->lock_joint_position(false);
			break;
		case 'j':
			num_active_joints = 7;
			for (int i = 0; i < dof; ++i) { active_joints[i] = 1; }
			jacobian_initialized = jacobian_estimate(perturbation_delta);
			break;
		case 'x':
			set_active_joints();
			jacobian_initialized = jacobian_estimate(perturbation_delta);
			break;
		case 'i':
			arm->move_to_initial_position();
			bhand->open_grasp();
			bhand->close_spread();
			break;
		case 'd':
			perturbation_delta = degreesToRadians(double_input(1, 20));
			break;
		case 'l':
			lambda = double_input(0, 1);
			break;
		case 'a':
			alpha = double_input(0, 1);
			break;
		case 'c':
			max_iterations = double_input(0, 500);
			break;
		case 't':
			image_tol = double_input(0, 500);
			lambda = default_lambda;
			break;
		case 'v':
			if (ready() && jacobian_initialized) {
				converge(alpha, max_iterations - 1, continous_motion);
				lambda = default_lambda;
			} else { ROS_WARN_STREAM("Jacobian is not initialized"); }
			break;
		case 's':
			if (ready() && jacobian_initialized) { 
				converge(alpha, 1, false);
				lambda = default_lambda;
			} else { ROS_WARN_STREAM("Jacobian is not initialized"); }
			break;
		case 'h':
			arm->move_to_home_position();
			break;
		case 'o':
			bhand->open_grasp();
			bhand->open_spread();
			break;
		case 'g':
			bhand->close_grasp();
			break;
		case 'z':
			bhand->close_spread();
			break;
		case 'n':
			filename = prefix + current_time() + ".txt";
			break;
		case 'q':
			exit_loop = true;
			break;
		default:
			ROS_WARN_STREAM("Unknown option");
		}
	}
}

void UVSControl::publish_dRt_command(std::string s)
{
	std_msgs::String msg;
    // std::stringstream ss;
    // ss << s;
    msg.data = s;
    dRt_command_pub.publish(msg);
}


void UVSControl::initialize()
{
	if (dof == 7) {
		ROS_INFO_STREAM("Initializing, please wait...");
		while (!arm->move_to_initial_position()) { continue; }
		ros::Duration(5).sleep();            
		while (!bhand->initialize()) { continue; }
		while (!bhand->close_spread()) { continue; }
		ros::Duration(10).sleep();
		bhand->hold_finger_position(true);
		ROS_INFO_STREAM("Initialization complete");
	}
}

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "UVSControl");
	ros::NodeHandle nh_("~");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	UVSControl VS(nh_);
	// VS.initialize();
	VS.loop();
	return 0;
}
