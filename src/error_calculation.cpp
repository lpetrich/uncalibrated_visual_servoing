/* 
 * lpetrich 17/07/18
 */

#include "uncalibrated_visual_servoing/error_calculation.h"


ErrorCalculator::ErrorCalculator(ros::NodeHandle nh_) 
{ // error calculator constructor
	max = 1.0e7;
	stereo_vision = false;
	calculate_now = false;
	new_error1 = false;
	new_error2 = false;
	// check how many cameras are connected and setup subscribers/publishers
	ros::V_string nodes;
	ros::master::getNodes(nodes);
	for (int i = 0; i < nodes.size(); i++) {
		std::string prefix = "/cam2";
		if(nodes[i].substr(0, prefix.size()) == prefix) {
			std::cout << "ErrorCalculator: Found 2 cameras" << std::endl;
			stereo_vision = true;
			sub_trackers2 = nh_.subscribe("/cam2/trackers/centers", 3, &ErrorCalculator::callback_centers2, this);
			break;
		} 
	}
	if (!stereo_vision) { std::cout << "ErrorCalculator: Found 1 camera" << std::endl; }
	sub_task_ids = nh_.subscribe("/task_ids", 3, &ErrorCalculator::callback_task_ids, this);
	sub_calculate = nh_.subscribe("/calculate", 3, &ErrorCalculator::callback_calculate, this);
	sub_trackers = nh_.subscribe("/cam1/trackers/centers", 3, &ErrorCalculator::callback_centers, this);
    pub_reset = nh_.advertise<std_msgs::Bool>("/error_calculation/reset", 10);
    pub_end_effector_position = nh_.advertise<uncalibrated_visual_servoing::TrackPoint>("/eef_pos", 10);
    pub_error = nh_.advertise<uncalibrated_visual_servoing::Error>("/image_error", 10);
}

ErrorCalculator::~ErrorCalculator() 
{ // destructor
	sub_task_ids.shutdown();
	sub_calculate.shutdown();
	sub_trackers.shutdown();
	pub_error.shutdown();
	pub_reset.shutdown();
	pub_end_effector_position.shutdown();
	if (stereo_vision) { 
		sub_trackers2.shutdown(); 
	}
}

Eigen::VectorXd ErrorCalculator::normalize_errors(const Eigen::VectorXd& e, int d)
{ // TODO
	return e;
}

void ErrorCalculator::group_error()
{
	uncalibrated_visual_servoing::Error error_msg;
	std::vector< std::vector<double> > ev = get_error();
	Eigen::VectorXd errors_1D;
	Eigen::VectorXd errors_2D;
	Eigen::VectorXd n1;
	Eigen::VectorXd n2;
	Eigen::VectorXd normalized_error;
	errors_1D = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev[0].data(), ev[0].size());
	errors_2D = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev[1].data(), ev[1].size());
	if (stereo_vision) {
		std::vector< std::vector<double> > ev2 = get_error2();
		Eigen::VectorXd errors_1D2;
		Eigen::VectorXd errors_2D2;
		errors_1D2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev2[0].data(), ev2[0].size());
		errors_2D2 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ev2[1].data(), ev2[1].size());
		errors_1D = concatenate_vectorxd(errors_1D, errors_1D2);
		errors_2D = concatenate_vectorxd(errors_2D, errors_2D2);
	} 
	new_error1 = false;
	new_error2 = false;
	n1 = normalize_errors(errors_1D, 1);
	n2 = normalize_errors(errors_2D, 2);
	normalized_error = concatenate_vectorxd(n1, n2);
	for (int i = 0; i < normalized_error.size(); ++i) {
		error_msg.error.push_back(normalized_error(i));
	}
	pub_error.publish(error_msg);
}

std::vector< std::vector<double> > ErrorCalculator::calculate_error(std::vector<Eigen::Vector2d> centers) 
{ // calculate error according to task id
	std::vector<double> error_1D;
	std::vector<double> error_2D;
	std::vector< std::vector<double> > return_vector;
	int start_idx = 0;
	int skip = 0;
	for (int i = 0; i < task_ids.size(); ++i) {
		Eigen::VectorXd v;
		std::vector<Eigen::Vector2d> sub_vector;
		double e;
		switch ((int)task_ids[i]) {
            case 0:
    			skip = 2;
    			sub_vector = slice(centers, start_idx, start_idx + skip); // uncalibrated_visual_servoing/vs_utilities.h
                v = point_to_point(sub_vector);
    			error_2D.push_back(v(0));
				error_2D.push_back(v(1));
                e = v.norm();
                break;
            case 1:
            	skip = 3;
    			sub_vector = slice(centers, start_idx, start_idx + skip);
                e = point_to_line(sub_vector);
                error_1D.push_back(e);
                break;
            case 2:
            	skip = 4;
            	sub_vector = slice(centers, start_idx, start_idx + skip);           
				v = line_to_line(sub_vector);
    			error_2D.push_back(v(0));
				error_2D.push_back(v(1));
                e = v.norm();	                    
                break;
            case 3:
            	skip = 4;
            	sub_vector = slice(centers, start_idx, start_idx + skip);               
                e = parallel_lines(sub_vector);
                error_1D.push_back(e);
                break;
            case 4:
            	skip = 0;
            	sub_vector = slice(centers, start_idx, start_idx + skip);              	
                e = points_to_conic(sub_vector);
                error_1D.push_back(e);
                break;
            case 5:
            	skip = 0;
            	sub_vector = slice(centers, start_idx, start_idx + skip);
            	e = point_to_plane(sub_vector);
                break;
            default:
            	skip = 0;
                ROS_WARN_STREAM("ErrorCalculator: Invalid task id");
                break;
        }
        start_idx += skip;                   
        if (fabs(e) > max) {
        	std::cout << fabs(e) << "\n";
        	ROS_WARN_STREAM("ErrorCalculator: Lost tracker");
        	reset();
			std_msgs::Bool msg;
			msg.data = true;
			pub_reset.publish(msg);
        }
    }
    return_vector.push_back(error_1D);
    return_vector.push_back(error_2D);
    return return_vector;
}

Eigen::VectorXd ErrorCalculator::point_to_point(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector2d robot;
  	Eigen::Vector2d target;
  	Eigen::Vector2d result;
	target = v[0];
  	robot = v[1];
	publish_end_effector(robot(0), robot(1));
  	// return result.cwiseAbs(); 
  	return target - robot;
}

double ErrorCalculator::point_to_line(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d robot, p1, p2;
	robot << v[0](0), v[0](1), 1;
	p1 << v[1](0), v[1](1), 1;
	p2 << v[2](0), v[2](1), 1;
	publish_end_effector(robot(0), robot(1));
  	return p1.cross(p2).dot(robot);
}

Eigen::VectorXd ErrorCalculator::line_to_line(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d r1, r2, p1, p2;
  	Eigen::Vector2d result(2);
	p1 << v[0](0), v[0](1), 1;
	p2 << v[1](0), v[1](1), 1;
	r1 << v[2](0), v[2](1), 1;
	r2 << v[3](0), v[3](1), 1;
	publish_end_effector(r1(0), r1(1));
	Eigen::Vector3d line = r1.cross(r2);  
	// double result;
	// change to 2 dimensional error
	result << line.dot(p1), line.dot(p2);
	// result = line.dot(p1) + line.dot(p2); 
  	return result; 
}

double ErrorCalculator::parallel_lines(std::vector<Eigen::Vector2d> v)
{
	Eigen::Vector3d r1, r2, p1, p2, l1, l2, intersection;
	p1 << v[0](0), v[0](1), 1;
	p2 << v[1](0), v[1](1), 1;
	r1 << v[2](0), v[2](1), 1;
	r2 << v[3](0), v[3](1), 1;
	publish_end_effector(r1(0), r1(1));
	double result;
	l1 = r1.cross(r2);
	l2 = p1.cross(p2);
	intersection = l1.cross(l2);
	result = intersection(2);
  	return result; 
}

double ErrorCalculator::points_to_conic(std::vector<Eigen::Vector2d> v)
{ // TODO
	// uses code from visp
	// Eigen::MatrixXd A(points_.size() - 1, 5);
	// Eigen::VectorXd b(points_.size() - 1);
	// Eigen::Vector3d p;
	// p << normX(points_[0]->coord.x), normY(points_[0]->coord.y), 1;
	// // A = (y^2 2xy 2x 2y 1)   x = (K0 K1 K2 K3 K4)^T  b = (-x^2 )
	// for(int i = 1; i < points_.size(); i++) {
	//     b(i - 1) = -(normX(points_[i]->coord.x) * normX(points_[i]->coord.x));
	//     A(i - 1, 0) = normY(points_[i]->coord.y) * normY(points_[i]->coord.y);
	//     A(i - 1, 1) = 2 * normX(points_[i]->coord.x) * normY(points_[i]->coord.y);
	//     A(i - 1, 2) = 2 * normX(points_[i]->coord.x);
	//     A(i - 1, 3) = 2 * normY(points_[i]->coord.y);
	//     A(i - 1, 4) = 1;
	// }
	// // Solve Ax = b, least squares minimization.  
	// Eigen::JacobiSVD<Eigen::MatrixXd> svd = A.jacobiSvd(Eigen::ComputeThinU | 
	//                                                       Eigen::ComputeThinV);                                                  
	// e = svd.solve(b); // the ellipse parameters are stored as a class variable. */
	double result;
	// // result(0) = (p(0) * p(0)) + (e(0) * p(1) * p(1)) + (2 * e(1) * p(0) * p(1)) + (2 * e(2) * p(0)) + (2 * e(3) * p(1)) + e(4);  
	std::cout << "Ellipse Error" << std::endl;
  	return result; 

}

double ErrorCalculator::point_to_plane(std::vector<Eigen::Vector2d> v)
{ // TODO
	Eigen::Vector3d p, p0, vec1, vec2, n, n_unit;
	// p << points_[0]->coord.x, points_[0]->coord.y, 1;
	// p0 << points_[2]->coord.x, points_[2]->coord.y, 1;
	// vec1 << points_[2]->coord.x - points_[1]->coord.x, points_[2]->coord.y - points_[1]->coord.y, 1;
	// vec2 << points_[2]->coord.x - points_[3]->coord.x, points_[2]->coord.y - points_[3]->coord.y, 1;
	double result;
	// n = (vec1).cross(vec2);
	// n_unit = (1/n.norm()) * n;
	// double  d = (-(n(0) * p0(0)) - (n(1) * p0(1)) - (n(2) * p0(2)));// / n.norm();  
	// //result(0) = n_unit.dot(p) + d;
	// result(0) = (n(0) * p(0)) + (n(1) * p(1)) + (n(0) * p(2)) + d;
	//std::cout << "Plane: " << result(0) << std::endl;
  	return result; 
}

void ErrorCalculator::publish_end_effector(double u, double v) 
{
  	uncalibrated_visual_servoing::TrackPoint eef_pt;
	eef_pt.x = u;
	eef_pt.y = v;
	pub_end_effector_position.publish(eef_pt);	
}

void ErrorCalculator::reset() 
{
	calculate_now = false;
	task_ids.setZero();
	std::cout << "ErrorCalculator: resetting\n";
}

void ErrorCalculator::spin() 
{
	ros::Rate r(30);
    while (ros::ok()) {
    	if ((!stereo_vision && new_error1) || (stereo_vision && new_error1 && new_error2)) {
    		group_error();
    	} 
    	ros::spinOnce();
        r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "error_calculation");
	ros::NodeHandle nh_("~");
	ErrorCalculator EC(nh_);
	EC.spin();
	return EXIT_SUCCESS;
}