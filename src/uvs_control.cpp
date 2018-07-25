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
	reset = false;
	ready_to_grasp = false;
	// prefix = "/home/laura/ComputerVision/vs_workspace/src/uncalibrated_visual_servoing/log_data/";
	prefix = "/home/froglake/vs_workspace/src/uncalibrated_visual_servoing/log_data/";
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
	error_sub = nh_.subscribe("/image_error", 1, &UVSControl::error_cb, this);
	eef_sub = nh_.subscribe("/eef_pos", 1, &UVSControl::eef_cb, this);
	reset_sub = nh_.subscribe("/reset", 1, &UVSControl::reset_cb, this);
}

UVSControl::~UVSControl() 
{ // shutdown ROS subscribers properly
	error_sub.shutdown();
	eef_sub.shutdown();
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
	*/		Eigen::MatrixXd ang_jacobian;
			Eigen::MatrixXd reduced_ang_jacobian;
			
			Eigen::MatrixXd kernel;
			Eigen::MatrixXd projection_matrix;
			Eigen::VectorXd projected_delta_q;
			std::cout << "here?" << std::endl;
			ang_jacobian = arm->get_ang_tool_jacobian();
			reduced_ang_jacobian.resize(ang_jacobian.rows()-1, ang_jacobian.cols());
			reduced_ang_jacobian.row(0) = ang_jacobian.row(0);
			reduced_ang_jacobian.row(1) = ang_jacobian.row(1);
			Eigen::FullPivLU<Eigen::MatrixXd> lu(reduced_ang_jacobian);
			kernel = lu.kernel();
			std::cout << "nope" << std::endl;
			projection_matrix = kernel * (kernel.transpose() * kernel ).inverse() * kernel.transpose();
			projected_delta_q = projection_matrix * delta_q;
			return projected_delta_q;
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

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd& current_error_value) 
{ // calculates new motion step to take with the control law: step = −λJ+e 
	Eigen::VectorXd step;
	step = -lambda * (jacobian_inverse * current_error_value).transpose();
	std::cout << "step calculated" << std::endl;
	step = projected_delta_q(step);
	return step;
}

bool UVSControl::convergence_check(const Eigen::VectorXd& current_error)
{ // should we make lambda larger as we get closer to the target? Test
	double pixel_step_size = 30.0;
	double n = current_error.norm();

	if (n < 300 && !(ready_to_grasp)) {
		bhand->open_grasp();
		bhand->open_spread();
		ready_to_grasp = true;
		std::cout << "ready to grasp object" << std::endl;
	}
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

bool UVSControl::broyden_update(double alpha, Eigen::VectorXd dy)
{ // update jacobian 
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd current_eef_position;
	// Eigen::VectorXd dy;
	Eigen::VectorXd dq;
	double dq_norm;

	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm == 0) { return false; } // return early to avoid dividing by zero
	current_eef_position = get_eef_position();
	dy = current_eef_position - previous_eef_position;
	update = ((( (-dy) - jacobian * dq)) * dq.transpose()) / (dq_norm * dq_norm);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	// log(filename, "previous eef position: ", previous_eef_position, false);
	log(filename, "current eef position: ", current_eef_position, false);
	// log(filename, "dq: ", dq, false);
	// log(filename, "dq norm: ", dq_norm, false);
	// log(filename, "dy: ", dy, false);
	log(filename, "dy norm: ", dy.norm(), false);
	log(filename, "broyden update: ", update, false);
	log(filename, "new jacobian: ", jacobian, false);
	// log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

int UVSControl::move_step(bool continous_motion)
{ // step through one loop of VS
	Eigen::VectorXd current_error;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;
	Eigen::VectorXd predicted_times;
	Eigen::VectorXd current_velocity;
	double sleep_time;
	// grab and use current error, check for convergence
	current_error = get_error();
	if (convergence_check(current_error)) {return 0;}
	step_delta = calculate_step(current_error);
	// grab and use current joint positions, check if valid
	current_joint_positions = arm->get_positions();
	target_position = calculate_target(current_joint_positions, step_delta);
	if (!limit_check(target_position, dof)) { return 1; }
	// calculate move run time
	current_velocity = arm->get_velocities();
	predicted_times = calculate_rampdown_and_endtime(step_delta, current_velocity);
	// write to screen for debugging
	log(filename, "current_error: ", current_error, false);
	log(filename, "previous_joint_positions: ", previous_joint_positions, false);
	log(filename, "current_joint_positions: ", current_joint_positions, false);
	log(filename, "previous_eef_position: ", previous_eef_position, false);
	log(filename, "step delta: ", step_delta, false);
	log(filename, "target position: ", target_position, false);
	std::cout << "Predicted ramp-down time: " << predicted_times[1] << std::endl;
	std::cout << "Predicted end time: " << predicted_times[2] << std::endl;
	// save previous state before move
	previous_joint_positions = current_joint_positions;
	previous_eef_position = get_eef_position();
	arm->call_move_joints(target_position, false);
	// check for continuous motion and adjust sleep times accordingly
	if (continous_motion) {
		// sleep_time = std::max(0.2, predicted_times[1] - 0.05);
		// sleep_time = std::max(0.3, predicted_times[1] - 0.01);
		sleep_time = std::min(1.0, std::max(0.3, (predicted_times[1] + predicted_times[2]) * 0.5)); // range between [0.3, 1.0]
	} else {
		sleep_time = 1.0;
	}
	std::cout << "// sleep time: " << sleep_time << std::endl;
	ros::Duration(sleep_time).sleep();
	return 2;
}

void UVSControl::converge(double alpha, int max_iterations, bool continous_motion)
{
	int c;
	Eigen::VectorXd dy;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < max_iterations; ++i) {
		if (reset) { 
			std::cout << "received reset request, exiting converge loop" << std::endl;
			reset = false;
			return; 
		}
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
				std::cout << "BROYDEN UPDATE:" << std::endl;
				dy = get_eef_position() - previous_eef_position;
				log(filename, "dy norm: ", dy.norm(), false);
				if (dy.norm() > 30) {
					std::cout << "dy large enough, performing broyden update" << std::endl;
					if (broyden_update(alpha) && !pseudoInverse(jacobian, jacobian_inverse)) { return false; }
					jacobian = previous_jacobian;
					log(filename, "resetting jacobian to: ", jacobian, false);
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
	Eigen::VectorXd e1;
	Eigen::VectorXd e2;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	jacobian.resize(get_error().size(), num_active_joints);
	initial_jacobian.resize(get_error().size(), num_active_joints);
	jacobian_inverse.resize(num_active_joints, get_error().size());
	int j = 0;
	for (int i = 0; i < dof ; ++i) {
		if (active_joints[i]) {
			ros::Duration(0.2).sleep();
			e1 = get_eef_position();
			position = arm->get_positions();
			target = vector_target(position, i, perturbation_delta);
			arm->call_move_joints(target, true);
			ros::Duration(0.2).sleep();
			e2 = get_eef_position();
			ros::Duration(0.2).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = (e1 - e2) / perturbation_delta;
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_eef_position = get_eef_position();
	if (!pseudoInverse(jacobian, jacobian_inverse)) { 
		std::cout << "Initial jacobian estimate failed -- condition number too large" << std::endl;
		return false; 
	}
	log(filename, "initial jacobian: ", initial_jacobian, false);
	log(filename, "inverse jacobian: ", jacobian_inverse, false);
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


void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	bool continous_motion = true;
	double perturbation_delta = 0.0875;
	double alpha = 1.0; // update rate
	int max_iterations = 25;
	double d;
	int c;
	std::string line;
	std::string s;
	lambda = default_lambda; // convergence rate
	while (ros::ok() && !exit_loop) {
		std::cout << "************************************************************************************************" <<
			"\nSelect option:" <<
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
			if (ready()) {
				num_active_joints = 7;
				for (int i = 0; i < dof; ++i) { active_joints[i] = 1; }
				jacobian_initialized = jacobian_estimate(perturbation_delta);
			}
			break;
		case 'x':
			if (ready()) { 
				set_active_joints();
				jacobian_initialized = jacobian_estimate(perturbation_delta);
			}
			break;
		case 'i':
			arm->move_to_initial_position();
			bhand->open_grasp();
			bhand->close_spread();
			ready_to_grasp = false;
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
