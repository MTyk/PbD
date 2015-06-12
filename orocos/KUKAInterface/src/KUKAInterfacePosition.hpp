

#include "KUKAInterface.hpp"


namespace iros {
namespace orocos {
namespace pbd {

using namespace std;
using namespace RTT;

class KUKAInterfacePosition: public KUKAInterface {

  public:
	KUKAInterfacePosition(string name);
	virtual ~KUKAInterfacePosition() {}

  private:
	virtual void approach_state();
	virtual void approach_start_state();

  protected:
    virtual void updateHook();

	OutputPort<geometry_msgs::Pose> port_position;
	OutputPort<geometry_msgs::Pose> port_position_ros;
	InputPort<geometry_msgs::Pose> port_position_measured;

	geometry_msgs::Twist stiffness, damping;

	bool check_start_state;
};

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */
