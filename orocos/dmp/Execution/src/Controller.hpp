#ifndef OROCOS_EXECUTION_COMPONENT_HPP
#define OROCOS_EXECUTION_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <helpers/friComm.h>

#include "ExecutionInterface/Executor.h"

#include "helpers/Exponential.h"
#include "helpers/Gaussian.h"
#include "helpers/vonMises.h"
#include "helpers/PhaseOscillator.h"

namespace iros {
namespace orocos {
namespace pbd {
namespace dmp {
namespace execution {

using namespace RTT;
using namespace RTT::os;
using namespace std;


class Controller : public RTT::TaskContext{
  public:
	Controller(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    double slow_down;

    void prepare(RTT::base::PortInterface*);
    void startExecution(RTT::base::PortInterface*);
    void stopExecution(RTT::base::PortInterface*);
    void startPosition(RTT::base::PortInterface*);
    void haltExecution(RTT::base::PortInterface*);

    void ready_to_start(RTT::base::PortInterface*);
    void start_pos_reached(RTT::base::PortInterface*);

    bool prepared, execute;

	TaskContext* KUKAInterface;

    InputPort<size_t> port_counter;
    InputPort<tFriIntfState> port_friState;

    InputPort<std_msgs::String> port_prepare;
    InputPort<std_msgs::Empty> port_startEvent;
    InputPort<std_msgs::Empty> port_stopEvent;
    InputPort<std_msgs::Empty> port_startPosEvent;
    InputPort< std_msgs::Empty > port_in_prepared;
    InputPort< std_msgs::Empty > port_in_start_pos_reached;
    InputPort< vector<double> > port_state_msr;
    //added by Martin Tykal
    InputPort< std_msgs::Empty> port_halt_executor;

    OutputPort< std_msgs::Empty > port_out_prepared;
    OutputPort< std_msgs::Empty > port_out_finished;
    OutputPort< std_msgs::Empty > port_out_start_pos_reached;

    OutputPort< vector<double> > port_start_state;
    OutputPort< vector<double> > port_state_cmd;

    OutputPort< std_msgs::Float64 > port_canonical;
    OutputPort< std_msgs::Float64 > port_temp_coupling;

    iros::pbd::dmp::execution::Parameters params;
    unique_ptr<iros::pbd::dmp::execution::Executor> executor;

    OperationCaller<void()> prepare_execution;

    TimeService::ticks start_ticks;
    TimeService::Seconds last_iteration_time;

    vector<double> last_state_cmd;
    vector<double> error;
    vector<double> max_allowed_error;
    //added by Martin Tykal
    vector<double> goal_state;
   
};


}  // namespace execution
}  // namespace dmp
}  // namespace pbd
}  // namespace orocos
}  // namespace iros

#endif
