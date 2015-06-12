#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>

#include "KUKAInterfaceJoints.hpp"

//If used in combination with AssistanceController
#define ASSISTANCE 0

namespace iros {
namespace orocos {
namespace pbd {

using namespace geometry_msgs;
using namespace std;
using namespace RTT;

size_t KUKAInterfaceJoints::num = 7;

KUKAInterfaceJoints::KUKAInterfaceJoints(string nodeName) :
		KUKAInterface(nodeName, FRI_CTRL_JNT_IMP) {
	Logger::In in((this->getName()));

	this->addPort("desPosition", port_position).doc("Desired position");
	this->addPort("desPositionROS", port_position_ros).doc(
			"Desired position for ROS");
	this->addPort("msrPosition", port_position_measured).doc(
			"Measured position");

	port_state_msr.setDataSample(vector<double>(num));

	check_start_state = false;

	if(!ASSISTANCE) {
		for (size_t i = 0; i < num; i++) {
			stiffness[i] = 1000;
			damping[i] = 0.7;

		}
	}
}

void KUKAInterfaceJoints::approach_state() {
	Logger::In in((this->getName()));

	static int i = -1;
	i++;
	JointsOut p { };
	JointsIn p_cur { };
	port_position_measured.read(p_cur);

	// Convert vector to joint configuration and publish
	p.positions = state;
	port_position.write(p);
	port_position_ros.write(p);
	//if(i % 50 == 0) {
	if (false) {
		log(Info) << "Msr pos: " << p_cur << endlog();
		log(Info) << "Cmd pos: " << p << endlog();
	}
	

	// Convert current joint configuration to vector for feedback
	vector<double> state_msr = p_cur.position;
	port_state_msr.write(state_msr);
}

void KUKAInterfaceJoints::approach_start_state() {
	Logger::In in((this->getName()));

	// Convert vector to joint configuration and call jointPTPmotion with that object
	JointsIn p_cmd { };
	boost::array<double, 7> p;
	port_position_measured.read(p_cmd);
	for (size_t i = 0; i < num; i++) {
		p[i] = start_state.at(i);
	}
	jointPTPMotion(p, 100);
	check_start_state = true;
}

void KUKAInterfaceJoints::updateHook() {
	KUKAInterface::updateHook();
	Logger::In in((this->getName()));


	double stiffnessValue=1000;
	double dampingValue=0.7;
	//Set impedance in update only after start position was reached, otherwise no gravity compensation possible
	if (ASSISTANCE && check_start_state) {
		//log(Info)<< "KUKAInterfaceJoints" <<endlog();
		std::ifstream ifs(
				"/home/intelligentrobotics/ws/pbd/Applications/params/ComplianceParameters.txt",
				std::fstream::in);
		{
			boost::archive::text_iarchive ia(ifs);
			ia >> stiffnessValue;
			ia >> dampingValue;
			//log(Info) << "stiffnessValue: " << stiffnessValue << " dampingValue: " << dampingValue << endlog();
		}
		//log(Info) << "Stiffness: " << stiffnessValue << endlog();
		//log(Info) << "Damping: " << dampingValue << endlog();
		for (size_t i = 0; i < num; i++) {
			stiffness[i] = stiffnessValue;
			damping[i] = dampingValue;

		}
		setJointImpedance(stiffness, damping);

	}

	// Check whether the start state has been reached
	if (check_start_state) {
		JointsIn p_cur { };
		port_position_measured.read(p_cur);

		double min_dist = 0.01; // Maximum allowed deviation

		for (size_t i = 0; i < num; i++) {
			if (fabs(start_state[i] - p_cur.position[i]) > min_dist) { // Compare each joint
				if(!ASSISTANCE)
				log(Info) << "Still deviation at axis " << i + 1 << ": Des = "
						<< start_state[i] << " Msr = " << p_cur.position[i]
						<< endlog();
				return;
			}
		}

		check_start_state = false;
		start_state_reached();
	}
}

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(KUKAInterfaceJoints)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
//ORO_CREATE_COMPONENT(iros::orocos::pbd::KUKAInterfaceJoints)
//ORO_CREATE_COMPONENT_TYPE()
ORO_LIST_COMPONENT_TYPE(iros::orocos::pbd::KUKAInterfaceJoints)
//ORO_CREATE_COMPONENT_LIBRARY()
