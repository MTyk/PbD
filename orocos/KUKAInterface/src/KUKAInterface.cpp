
#include "KUKAInterface.hpp"

#include <rtt/Component.hpp>
#include <iostream>

namespace iros {
namespace orocos {
namespace pbd {

using namespace RTT;
using namespace std;

KUKAInterface::KUKAInterface(string name, FRI_CTRL control_mode) : TaskContext(name), control_mode {control_mode}, prepare {false},
		// Initialize operation callers from KUKACommander
		getCurrentState("getCurrentState"), getCurrentControlMode("getCurrentControlMode"),
		switchToMonitorState("switchToMonitorState"), switchToCommandState("switchToCommandState"),
		stopCommunication("stopCommunication"),
		setControlMode("setControlMode"),
		setCartesianImpedance("setCartesianImpedance"), setJointImpedance("setJointImpedance"),
		activateGravityCompensation("activateGravityCompensation"),
		moveToJointPosition("moveToJointPosition"), moveToCartesianPosition("moveToCartesianPosition"),
		stopMovements("stopMovements"), isMoving("isMoving"),
		jointPTPMotion("jointPTPMotion"),
		CartesianLINMotion("CartesianLINMotion"), CartesianPTPMotion("CartesianPTPMotion"),
		getQuaternionFromRPY("getQuaternionFromRPY") {
	Logger::In in((this->getName()));

	this->addEventPort("start_state", port_start_state, boost::bind(&KUKAInterface::incoming_start_state, this, _1)).doc("Retrieves start state to be approached");
	this->addEventPort("desState", port_state_cmd, boost::bind(&KUKAInterface::incoming_state, this, _1)).doc("Retrieves state to be approached");
	this->addPort("friState", port_fri_state).doc("Current FRI state");
	this->addPort("msrState", port_state_msr).doc("Outputs the current state");
	this->addPort("prepared", port_prepared).doc("Outputs when preparation completed");
	this->addPort("start_pos_reached", port_start_pos_reached).doc("Outputs when start position was reached");

	this->addOperation("prepare", &KUKAInterface::prepare_state_approach, this, OwnThread).doc("Set appropriate state and control mode");

	state.reserve(21);
	start_state.reserve(21);

	log(Debug) << "KUKAInterface constructed !" << endlog();
}


bool KUKAInterface::configureHook(){
	Logger::In in((this->getName()));

	// Configure OperationCaller of KUKACommander
	TaskContext::PeerList peerList = getPeerList();
	if (1 != peerList.size()) {
		log(Error) << ("Failed to configure KUKAInterface. There should be exactly one peer.") << endlog();
		return false;
	}
	commander = getPeer(peerList.front());

	this->requires("Commander")->addOperationCaller(getCurrentState);
	this->requires("Commander")->addOperationCaller(getCurrentControlMode);
	this->requires("Commander")->addOperationCaller(switchToMonitorState);
	this->requires("Commander")->addOperationCaller(switchToCommandState);
	this->requires("Commander")->addOperationCaller(stopCommunication);
	this->requires("Commander")->addOperationCaller(setControlMode);
	this->requires("Commander")->addOperationCaller(setCartesianImpedance);
	this->requires("Commander")->addOperationCaller(setJointImpedance);
	this->requires("Commander")->addOperationCaller(activateGravityCompensation);
	this->requires("Commander")->addOperationCaller(moveToJointPosition);
	this->requires("Commander")->addOperationCaller(moveToCartesianPosition);
	this->requires("Commander")->addOperationCaller(stopMovements);
	this->requires("Commander")->addOperationCaller(isMoving);
	this->requires("Commander")->addOperationCaller(jointPTPMotion);
	this->requires("Commander")->addOperationCaller(CartesianLINMotion);
	this->requires("Commander")->addOperationCaller(CartesianPTPMotion);
	this->requires("Commander")->addOperationCaller(getQuaternionFromRPY);

	this->requires("Commander")->connectTo(commander->provides("Commander"));

	if(!this->requires("Commander")->ready() ||
			!getCurrentState.ready() ||	!getCurrentControlMode.ready() || !switchToMonitorState.ready() ||
			!switchToCommandState.ready() || !stopCommunication.ready() || !setControlMode.ready() ||
			!setCartesianImpedance.ready() || !setJointImpedance.ready() || !setJointImpedance.ready() ||
			!moveToJointPosition.ready() ||	!moveToCartesianPosition.ready() ||	!stopMovements.ready() ||
			!isMoving.ready() || !jointPTPMotion.ready() || !CartesianLINMotion.ready() ||
			!CartesianPTPMotion.ready() || !getQuaternionFromRPY.ready()) {
		log(Error) << "Operation interface to KUKACommander not ready." << endlog();
		return false;
	}

	log(Debug) << "KUKAInterface configured !" << endlog();
	return true;
}


bool KUKAInterface::startHook(){
	Logger::In in((this->getName()));
	log(Debug) << "KUKAInterface started !" << endlog();
	return true;
}

void KUKAInterface::updateHook(){
	Logger::In in((this->getName()));
	//log(Debug) << "KUKAInterface executes updateHook !" << endlog();

	// Write message when preparation is done
	if(prepare) {
		// The control mode depends on the implementation of the KUKAInterface
		if(getCurrentControlMode() == control_mode && getCurrentState() == FRI_STATE_CMD) {
			prepare = false;
			port_prepared.write(std_msgs::Empty {});
		}
	}

}

void KUKAInterface::stopHook() {
	Logger::In in((this->getName()));
	log(Debug) << "KUKAInterface executes stopping !" << endlog();
}

void KUKAInterface::cleanupHook() {
	Logger::In in((this->getName()));
	log(Debug) << "KUKAInterface cleaning up !" << endlog();
}


void KUKAInterface::incoming_state(RTT::base::PortInterface*) {
	Logger::In in((this->getName()));
	
	// Call approach_state method, when a new state is received
	if(port_state_cmd.read(state) == NewData) {
		port_fri_state.read(fri_state);
		approach_state(); // Abstract method, implemented in child class
	}
}


void KUKAInterface::incoming_start_state(RTT::base::PortInterface*) {
	Logger::In in((this->getName()));
	
	// Call approach_start_state method, when a new start state is received
	if(port_start_state.read(start_state) == NewData) {
		log(Info) << "Received start state, vec len = " << start_state.size() << endlog();
		approach_start_state(); // Abstract method, implemented in child class
	}
}

void KUKAInterface::prepare_state_approach() {
	Logger::In in((this->getName()));
	
	// If the current mode and state are not correct, call setControlMode
	if(getCurrentControlMode() != control_mode || getCurrentState() != FRI_STATE_CMD) {
		setControlMode(control_mode, FRI_STATE_CMD);
		prepare = true; // Observe mode and state in further iterations
		return;
	}
	// mode and state are correct, publish this information
	port_prepared.write(std_msgs::Empty {});
}

void KUKAInterface::start_state_reached() {
	Logger::In in((this->getName()));
	
	// Publish a message when the start state has been reached
	port_start_pos_reached.write(std_msgs::Empty {});
}


}  // namespace pbd
}  // namespace orocos
}  // namespace iros
