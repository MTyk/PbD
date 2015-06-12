

#include "KUKAInterface.hpp"
#include <sensor_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <lwr_fri/typekit/Types.hpp>


typedef motion_control_msgs::JointPositions JointsOut;
typedef sensor_msgs::JointState JointsIn;

namespace iros {
namespace orocos {
namespace pbd {

using namespace std;
using namespace RTT;

class KUKAInterfaceJoints: public KUKAInterface {

  public:
	KUKAInterfaceJoints(string name);
	virtual ~KUKAInterfaceJoints() {}

	static size_t num;

  private:
	virtual void approach_state();
	virtual void approach_start_state();

  protected:
    virtual void updateHook();

	OutputPort<JointsOut> port_position;
	OutputPort<JointsOut> port_position_ros;
	InputPort<JointsIn> port_position_measured;

	boost::array<float, 7> stiffness, damping;

	bool check_start_state;
};

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */
