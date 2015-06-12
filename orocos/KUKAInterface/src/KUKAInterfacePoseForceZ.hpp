/*
 * KUKAInterfacePoseForceZ.h
 *
 *  Created on: May 5, 2014
 *      Author: Franz Steinmetz
 */

#ifndef KUKAINTERFACEPOSEFORCEZ_H_
#define KUKAINTERFACEPOSEFORCEZ_H_

#include "KUKAInterface.hpp"

namespace iros {
namespace orocos {
namespace pbd {

using namespace std;
using namespace RTT;

class KUKAInterfacePoseForceZ: public iros::orocos::pbd::KUKAInterface {

public:
	KUKAInterfacePoseForceZ(string name);
	virtual ~KUKAInterfacePoseForceZ() {}

private:
	virtual void approach_state();
	virtual void approach_start_state();

protected:
	virtual void updateHook();
	double quatNorm(double x, double y, double z, double w);
	void quatNorm(geometry_msgs::Quaternion& q);

	OutputPort<geometry_msgs::Pose> port_position;
	OutputPort<geometry_msgs::Pose> port_position_ros;
	OutputPort<geometry_msgs::Wrench> port_force;
	OutputPort<std_msgs::Float64> port_force_ros;
	OutputPort<std_msgs::Float64> port_force_cmd_ros;
	InputPort<geometry_msgs::Pose> port_position_msr;
	InputPort<geometry_msgs::Wrench> port_force_msr;
	InputPort<geometry_msgs::Vector3> port_ft_force_msr;

	geometry_msgs::Twist stiffness, damping;
	boost::array<float, 7> stiffnessJoints, dampingJoints;
	bool check_start_state;
};

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */

#endif /* KUKAINTERFACEPOSEFORCEZ_H_ */
