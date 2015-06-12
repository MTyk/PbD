
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>
#include <kdl/frames.hpp>

#include "KUKAInterfacePose.hpp"

#define ASSISTANCE 1

namespace iros {
namespace orocos {
namespace pbd {


using namespace geometry_msgs;
using namespace std;
using namespace RTT;


	KUKAInterfacePose::KUKAInterfacePose(string nodeName) : KUKAInterfacePosition(nodeName) {
		Logger::In in((this->getName()));

		port_state_msr.setDataSample(vector<double>(7));

		if(!ASSISTANCE){	
			stiffness.linear.x = 2000;
			stiffness.linear.y = 2000;
			stiffness.linear.z = 2000;
			stiffness.angular.x = 200; // KIM: 200 // Plane: 400
			stiffness.angular.y = 200; // KIM: 200 // Plane: 300
			stiffness.angular.z = 200; // KIM: 200 // Plane: 400

			damping.linear.x = 0.7;
			damping.linear.y = 0.7;
			damping.linear.z = 0.7;
			damping.angular.x = 0.7;
			damping.angular.y = 0.7;
			damping.angular.z = 0.7;
		}	
	}

	void KUKAInterfacePose::approach_state() {
		Logger::In in((this->getName()));

		double stiffnessValue;
		double dampingValue;
		if (ASSISTANCE) {
			/*log(Info)<< "KUKAInterfacePose" <<endlog();
			std::ifstream ifs(
					"/home/intelligentrobotics/ws/pbd/Applications/params/ComplianceParameters.txt",
					std::fstream::in);
			{
				boost::archive::text_iarchive ia(ifs);
				ia >> stiffnessValue;
				ia >> dampingValue;
			}
			log(Info) << "Stiffness: " << stiffnessValue << endlog();
			log(Info) << "Damping: " << dampingValue << endlog();

			stiffness.linear.x = stiffnessValue;
			stiffness.linear.y = stiffnessValue;
			stiffness.linear.z = stiffnessValue;
			stiffness.angular.x = 200; 
			stiffness.angular.y = 200; 
			stiffness.angular.z = 200;

			damping.linear.x = dampingValue;
			damping.linear.y = dampingValue;
			damping.linear.z = dampingValue;
			damping.angular.x = dampingValue;
			damping.angular.y = dampingValue;
			damping.angular.z = dampingValue;*/

		}


		Pose p {}, p_cur {};
		port_position_measured.read(p_cur);
		
		// Convert vector to pose and publish
		quatNorm(p_cur.orientation); // normalize quaternion
		p.position.x = state[0];
		p.position.y = state[1];
		p.position.z = state[2];
		double norm = quatNorm(state[3], state[4], state[5], state[6]);
		p.orientation.x = state[3] / norm;
		p.orientation.y = state[4] / norm;
		p.orientation.z = state[5] / norm;
		p.orientation.w = state[6] / norm;
		port_position.write(p);
		port_position_ros.write(p);
		setCartesianImpedance(stiffness, damping);

		// Conversion of pose to vector
		// The positional dimensions are just copied
		vector<double> state_msr(7);
		state_msr[0] = p_cur.position.x;
		state_msr[1] = p_cur.position.y;
		state_msr[2] = p_cur.position.z;
		/*state_msr[3] = p_cur.orientation.x;
		state_msr[4] = p_cur.orientation.y;
		state_msr[5] = p_cur.orientation.z;
		state_msr[6] = p_cur.orientation.w;*/

		// For the quaternion, a measure for the error is returned 
		// Determine error in angle
		geometry_msgs::Quaternion q_cur = p_cur.orientation;
		geometry_msgs::Quaternion q_des = p.orientation;
		// Relative angle between the current and desired quaternion
		double angle = 2 * acos(q_cur.w*q_des.w + (q_cur.x*q_des.x + q_cur.y*q_des.y + q_cur.z*q_des.z));
		angle =  angle > M_PI ? -(2 * M_PI - angle) : angle;
		// X, Y and Z are just copied
		state_msr[3] = state[3]; // p_cur.orientation.x;
		state_msr[4] = state[4]; // p_cur.orientation.y;
		state_msr[5] = state[5]; // p_cur.orientation.z;
		// Artificially generated error to better handle quaternions
		double rel_err = angle / M_PI * 0.05;
		// W gets the measure of error
		state_msr[6] = state[6] + rel_err; // p_cur.orientation.w;


		if(true) {
			log(Debug) << "Msr pos: " << p_cur.position << endlog();
			log(Debug) << "Cmd pos: " << p.position << endlog();
			log(Debug) << "Diff rot: " << angle << endlog();
		}

		port_state_msr.write(state_msr);
	}

	void KUKAInterfacePose::approach_start_state() {
		Logger::In in((this->getName()));
		Pose p {};
		
		// Convert vector to pose and normalize quaternion
		p.position.x = start_state.at(0);
		p.position.y = start_state.at(1);
		p.position.z = start_state.at(2);
		double norm = quatNorm(start_state.at(3), start_state.at(4), start_state.at(5), start_state.at(6));
		start_state[3] /= norm;
		start_state[4] /= norm;
		start_state[5] /= norm;
		start_state[6] /= norm;
		p.orientation.x = start_state.at(3);
		p.orientation.y = start_state.at(4);
		p.orientation.z = start_state.at(5);
		p.orientation.w = start_state.at(6);
		
		// Use KRC PTP command to approach the start state using the generated pose object
		CartesianPTPMotion(p, 50);
		check_start_state = true;
	}

	void KUKAInterfacePose::updateHook() {
		KUKAInterface::updateHook();
		Logger::In in((this->getName()));

		if(check_start_state) {
			Pose p_cur {};
			port_position_measured.read(p_cur);
			
			// Generate general rotations from quaternions
			KDL::Rotation rot_msr = KDL::Rotation::Quaternion(p_cur.orientation.x, p_cur.orientation.y, p_cur.orientation.z, p_cur.orientation.w);
			KDL::Rotation rot_cmd = KDL::Rotation::Quaternion(start_state[3], start_state[4], start_state[5], start_state[6]);

			// Retrieve quaternion elements from rotations
			double ax, ay, az, aw, bx, by, bz, bw;
			rot_msr.GetQuaternion(ax, ay, az, aw);
			rot_cmd.GetQuaternion(bx, by, bz, bw);

			double min_dist = 0.01;
			double min_rot = 0.01;
			double angle = 2 * acos(aw*bw + (ax*bx + ay*by + az*bz)); // Determine relative error between quaternions
			angle =  angle > M_PI ? -(2 * M_PI - angle) : angle; // shift to be between -180 and +180
			log(Debug) << "Target:" << start_state[0] << " " << start_state[1] << " " << start_state[2] << " 0" << endlog();
			log(Debug) << "Moment:" << p_cur.position.x << " " << p_cur.position.y << " " << p_cur.position.z << " " << angle << endlog();
			// Check for deviations from desired start state
			if(fabs(start_state[0] - p_cur.position.x) > min_dist ||
					fabs(start_state[1] - p_cur.position.y) > min_dist ||
					fabs(start_state[2] - p_cur.position.z) > min_dist ||
					fabs(angle) > min_rot)
				return;

			check_start_state = false;
			start_state_reached();
		}
	}


	double KUKAInterfacePose::quatNorm(double x, double y, double z, double w) {
		return sqrt(x*x + y*y + z*z + w*w);
	}

	void KUKAInterfacePose::quatNorm(geometry_msgs::Quaternion& q) {
		double norm = quatNorm(q.x, q.y, q.z, q.w);
		q.x /= norm;
		q.y /= norm;
		q.z /= norm;
		q.w /= norm;
	}

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(KUKAInterfacePose)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//ORO_CREATE_COMPONENT(iros::orocos::pbd::KUKAInterfacePose)
//ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(iros::orocos::pbd::KUKAInterfacePose)
ORO_CREATE_COMPONENT_LIBRARY()
