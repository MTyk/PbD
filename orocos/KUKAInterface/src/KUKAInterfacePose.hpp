

#include "KUKAInterfacePosition.hpp"

namespace iros {
namespace orocos {
namespace pbd {

using namespace std;
using namespace RTT;

class KUKAInterfacePose: public KUKAInterfacePosition {

  public:
	KUKAInterfacePose(string name);
	virtual ~KUKAInterfacePose() {}

  protected:
    virtual void updateHook();

  private:
	virtual void approach_state();
	virtual void approach_start_state();

	double quatNorm(double x, double y, double z, double w);
	void quatNorm(geometry_msgs::Quaternion& q);

};

} /* namespace pbd */
} /* namespace orocos */
} /* namespace iros */
