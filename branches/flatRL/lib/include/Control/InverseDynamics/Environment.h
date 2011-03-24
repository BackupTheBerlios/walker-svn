#ifndef __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_ENVIRONMENT_H___
#define __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_ENVIRONMENT_H___

#include "../PhysicsEnvironment.h"

namespace ctrl {
namespace id {

class Environment :
    public PhysicsEnvironment
{
template<typename T> friend class ctrl::environment_wrapper;
public:
    Environment(const PhysicsEnvironment::DESC& desc);

    void toggleEpisodic(bool toggle) {episodic = toggle;}

    // Override Environment
    void reset();
    void makeReward();

    // get
    float getReward() const { return reward; }

private:
    bool   terminal;
    bool   episodic;
    float  reward;
    float  averagePos;
    float  averageVel;
};

typedef boost::intrusive_ptr<Environment>          environment_ptr;
typedef boost::intrusive_ptr<const Environment>    const_environment_ptr;

} // namespace id
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_ENVIRONMENT_H___
