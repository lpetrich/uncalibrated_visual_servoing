/* 
 * lpetrich 27/06/18
 */

#include "uncalibrated_visual_servoing/uvs_control.h"

UVSControl::UVSControl(ros::NodeHandle nh_) 
{
	dof = 0;
	total_joints = 0;
	MAX_COND = 50000.0;
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
		total_joints = 7;
		bhand = new BHandControl("/zeus", nh_);
	} else if (dof == 4) {
		total_joints = 4;
	} else {
		ROS_WARN_STREAM("Invalid DOF, reset and try again");
		exit(EXIT_FAILURE);
	}
	ROS_INFO_STREAM("Robot has " << dof << " DOF");
	error_sub = nh_.subscribe("/image_error", 1, &UVSControl::error_cb, this);
	eef_sub = nh_.subscribe("/eef_pos", 1, &UVSControl::eef_cb, this);
}

UVSControl::~UVSControl() 
{ // shutdown ROS subscribers properly
	error_sub.shutdown();
	eef_sub.shutdown();
}

Eigen::VectorXd UVSControl::calculate_delta_q()
{ // calculates the actual motion change in joint space to use in Broyden's update
	Eigen::VectorXd total_dq;
	Eigen::VectorXd dq(dof);
	Eigen::VectorXd current_joint_positions = arm->get_positions();
	log(filename, "previous position: ", previous_joint_positions, false);
	log(filename, "current position: ", current_joint_positions, false);
	total_dq = current_joint_positions - previous_joint_positions;
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			dq[j] = total_dq[i];
			j++;
		}
	}
	return dq;
}

Eigen::VectorXd UVSControl::calculate_target(const Eigen::VectorXd& current_state, const Eigen::VectorXd& delta)
{ // calculates target vector in joint space to move to with given delta added to active joints
	Eigen::VectorXd target_state(total_joints);
	int j = 0;
	for (int i = 0; i < total_joints; ++i) {
		if (active_joints[i]) {
			target_state[i] = (current_state[i] + delta[j]);
			j++;
		} else {
			target_state[i] = current_state[i]; 
		}
	}
	return target_state;
}

Eigen::VectorXd UVSControl::calculate_step(const Eigen::VectorXd& current_error_value, double lambda) 
{ // calculates new motion step to take with the control law: step = −λJ+e 
	Eigen::VectorXd step;
	step = -lambda * (jacobian_inverse * current_error_value).transpose();
	return step;
}

int UVSControl::move_step(double lambda, bool continous_motion)
{ // step through one loop of VS
	ros::Rate default_rate(ros::Duration(0.3));
	Eigen::VectorXd current_error;
	Eigen::VectorXd current_joint_positions;
	Eigen::VectorXd step_delta;
	Eigen::VectorXd target_position;

	current_error = get_error();
	current_joint_positions = arm->get_positions();
	// check for convergence
	if ((1.0 - lambda) * current_error.norm() < image_tol) { // FIX
		lambda = 1.0 - image_tol/current_error.norm();
		std::cout << "Next move places EEF too close to target - adjusting lambda to " << lambda << std::endl;
		if (lambda != lambda || lambda < 0.01) {
			std::cout << "Adjusted lambda is converged, or NaN" << std::endl;
			return 0;
		}
	}
	step_delta = calculate_step(current_error, lambda);
	target_position = calculate_target(current_joint_positions, step_delta);
	previous_joint_positions = current_joint_positions;
	log(filename, "step delta: ", step_delta, false);
	log(filename, "target: ", target_position, false);
	log(filename, "current position: ", current_joint_positions, false);
	if (!limit_check(target_position, total_joints)) { // wam_control --> misc_utilities.h 
		ROS_WARN_STREAM("Target not within joint limits, please reinitialize jacobian."); 
		return 1; 
	}

	if (continous_motion) {
		Eigen::VectorXd predicted_times;
		Eigen::VectorXd current_velocity;
		current_velocity = arm->get_velocities();
		predicted_times = calculate_rampdown_and_endtime(step_delta, current_velocity);
		std::cout << "Predicted ramp-down time: " << predicted_times[0] << std::endl;
		std::cout << "Predicted plateau start time: " << predicted_times[2] << std::endl;
		std::cout << "Predicted end time: " << predicted_times[1] << std::endl;
		arm->call_move_joints(target_position, false);
		if (predicted_times[0] < 5.0 && predicted_times[0] > 0.1) {
			ros::Rate r(ros::Duration(std::max(0.3, predicted_times[0]/2)));
			r.sleep();
			// ros::Duration(std::max(predicted_times[0]/2, 0.2)).sleep();
			// ros::Duration(predicted_times[0] - 0.01).sleep();
		}
		else {
			std::cout << "Problem determining move-time: defaulting to 0.5 seconds" << std::endl;
			ros::Duration(0.5).sleep();
		}
	} else {
		arm->call_move_joints(target_position, true);
		default_rate.sleep();
	}
	return 2;
}

void UVSControl::converge(double lambda, double alpha, double iterations, bool continous_motion) 
{ // converge at target or run through set number of iterations
	int c;
	std::cout << "\n**************************************" << std::endl;
	for (int i = 0; i < iterations ; ++i) {
		std::cout << "iteration: " << i << std::endl;
		ros::Time begin = ros::Time::now();
		c = move_step(lambda, continous_motion);
		switch (c) {
			case 0: // convergence - return early
				std::cout << "Converged early" << std::endl;
				return;
			case 1: // joints out of limit, reset jacobian
				jacobian = previous_jacobian;
				break;
			case 2: // step completed successfully
				if (!broyden_update(alpha)) { // condition number failed, reset to previous jacobian
					jacobian = previous_jacobian;
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
	dof = 0;
	while (ss >> n) {
		active_joints[n-1] = 1;
		dof += 1;
	}
}

bool UVSControl::broyden_update(double alpha)
{ // update jacobian 
	Eigen::MatrixXd update(jacobian.rows(), jacobian.cols());
	Eigen::VectorXd current_eef_pos;
	Eigen::VectorXd dy;
	Eigen::VectorXd dq;
	double dq_norm;

	dq = calculate_delta_q();
	dq_norm = dq.norm();
	if (dq_norm == 0) { return false; }
	current_eef_pos = get_eef_position();
	dy = current_eef_pos - previous_eef_position;
	previous_eef_position = current_eef_pos;
	update = ((( (-dy) - jacobian * dq)) * dq.transpose()) / (dq_norm * dq_norm);
	previous_jacobian = jacobian;
	jacobian = jacobian + (alpha * update);
	pseudoInverse(jacobian, jacobian_inverse);

	log(filename, "dq: ", dq, false);
	log(filename, "dq norm: ", dq_norm, false);
	log(filename, "dy: ", dy, false);
	log(filename, "dy norm: ", dy.norm(), false);
	log(filename, "broyden update: ", update, false);
	log(filename, "old jacobian: ", previous_jacobian, false);
	log(filename, "new jacobian: ", jacobian, false);
	log(filename, "inverse jacobian: ", jacobian_inverse, false);

	if (check_condition()) { return false; }
	return true;
}

bool UVSControl::jacobian_estimate(double perturbation_delta) 
{ // perturb each active joint for the initial jacobian estimation
	Eigen::VectorXd e1;
	Eigen::VectorXd e2;
	Eigen::VectorXd target;
	Eigen::VectorXd position;
	jacobian.resize(get_error().size(), dof);
	int j = 0;
	for (int i = 0; i < total_joints ; ++i) {
		if (active_joints[i]) {
			// perturb only active joints
			ros::Duration(0.1).sleep();
			e1 = get_error();
			position = arm->get_positions();
			target = vector_target(position, i, perturbation_delta);
			log(filename, "current position: ", position, false);
			log(filename, "target: ", target, false);
			arm->call_move_joints(target, true);
			ros::Duration(0.1).sleep();
			e2 = get_error();
			ros::Duration(0.1).sleep();
			arm->call_move_joints(position, true);
			jacobian.col(j) = (e2 - e1) / perturbation_delta;
			j++;
		}
	}
	initial_jacobian = jacobian;
	previous_joint_positions = arm->get_positions();
	previous_eef_position = get_eef_position();
	pseudoInverse(jacobian, jacobian_inverse);
	log(filename, "initial jacobian: ", initial_jacobian, false);
	log(filename, "inverse jacobian: ", jacobian_inverse, false);
	return true;
}

Eigen::VectorXd UVSControl::calculate_rampdown_and_endtime(const Eigen::VectorXd& delta, const Eigen::VectorXd& current_velocities)
{ // Code from libbarrett copied here and refactored to determine rampDown time and end time
	// First element of vector is rampdown, second element is end time.
	double time_startdown = 0.0;
	double temp_time_startdown = 1e3;
	double temp_time_end;
	double time_end;
	double time_start_plateau = 0.0;
	double length;
	double v_init;
	double acc = 0.5;
	double vel = 0.5;
	double v_diff;
	Eigen::VectorXd times(3);
	for (int i = 0; i < delta.size(); ++i) {
		length = fabs(delta[i]);
		v_init = fabs(current_velocities[i]);
	    /* First, is the length too short to decellerate to stop?
	     * |\
	     * | \
	     * |  \__ */
	    if ( length < 0.5 * v_init * v_init / acc ) {	
		      /* There are no up or plateau phases */
	        temp_time_startdown = 0.0;
	        temp_time_end = 2 * length / v_init;
	        if ( temp_time_startdown > time_startdown || temp_time_end > time_end ) {
	        	time_startdown = temp_time_startdown; 
	        	time_start_plateau = time_startdown;
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
		    temp_time_startdown = (v_top - v_init) / acc;
		    temp_time_end = temp_time_startdown + v_top / acc;
		    if ( temp_time_startdown > time_startdown || temp_time_end > time_end ) {
		        time_startdown = temp_time_startdown; 
	        	time_start_plateau = time_startdown;
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
	    temp_time_startdown = time_endup + (s_startdown - s_endup) / vel;
	    temp_time_end = time_startdown + vel / acc;
	    if ( temp_time_startdown > time_startdown || temp_time_end > time_end ) {
	        time_startdown = temp_time_startdown;
        	time_start_plateau = time_endup;
	        time_end = temp_time_end;
	    }
   	}
    times[0] = time_startdown;
    times[1] = time_end;
    times[2] = time_start_plateau;
    return times;
}

void UVSControl::loop()
{ // main loop for user interaction
	bool jacobian_initialized = false;
	bool exit_loop = false;
	bool continous_motion = true;
	double perturbation_delta = 0.0875;
	double lambda = 0.15; // convergence rate
	double alpha = 1.0; // update rate
	double max_iterations = 10;
	double d;
	std::string line;
	std::string s;
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
			break;
		case 'v':
			if (ready() && jacobian_initialized) { 
				try { converge(lambda, alpha, max_iterations, continous_motion); } 
				catch(...) { ROS_WARN_STREAM("Convergence failed"); }
			} else { ROS_WARN_STREAM("Jacobian is not initialized"); }
			break;
		case 's':
			if (ready() && jacobian_initialized) {
				if (move_step(lambda, false) == 2) { broyden_update(alpha); }
				else { jacobian_initialized = false; }
			} else { ROS_WARN_STREAM("Jacobian is not initialized"); }
			break;
		case 'h':
			arm->move_to_home_position();
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
	UVSControl UVS(nh_);
	// VS.initialize();
	UVS.loop();
	return 0;
}