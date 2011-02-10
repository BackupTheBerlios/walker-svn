#ifndef __WALKER_YARD_CONTROL_CHAIN_PD_CONTROL_H__
#define __WALKER_YARD_CONTROL_CHAIN_PD_CONTROL_H__

#include "Control.h"

namespace ctrl {
namespace chain {

class PDControl :
    public Control
{
public:
    PDControl(const loose_timer_ptr& timer);
    ~PDControl();

    // Override Control
	void loadConfig(const std::string& configFile);

    double getTimeInterval() const { return timeInterval; }

private:
	// Override ChainControl
    void drawDebugInfo();

    // Override PhysicsControl
	void   acquire_safe();
	void   unacquire_safe();
    double pre_sync();

private:
	// settings
    std::string     pdConfig;

    // flow
    bool            needRestart;
	double          startTime;
    double          lastUpdate;
    double          timeInterval;

    // PD control
    ublas::vector<physics::real>    mass;
    ublas::matrix<physics::real>    Kd;
    ublas::vector<physics::real>    targetVelocity;
    ublas::matrix<physics::real>    Kp;
    ublas::vector<physics::real>    targetPosition;
};

typedef boost::intrusive_ptr<PDControl>         pd_control_ptr;
typedef boost::intrusive_ptr<const PDControl>   const_pd_control_ptr;

} // namespace chain
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_PD_CONTROL_H__
