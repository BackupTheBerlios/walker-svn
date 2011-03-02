#ifndef __WALKER_YARD_CONTROL_ENVIRONMENT_H___
#define __WALKER_YARD_CONTROL_ENVIRONMENT_H___

#include <slon/Utility/referenced.hpp>
#include <boost/intrusive_ptr.hpp>

namespace ctrl {

class Environment :
    public slon::Referenced
{
public:
    enum STATE
    {
        READY,
        WAITING_STATE,
        WAITING_ACTION,
        WAITING_REWARD
    };

public:
    // Implement Environment
    virtual STATE getState() const = 0;

    virtual void makeState() = 0;
    virtual void makeAction() = 0;
    virtual void makeReward() = 0;

    virtual ~Environment() {}
};

typedef boost::intrusive_ptr<Environment>       environment_ptr;
typedef boost::intrusive_ptr<const Environment> const_environment_ptr;
    
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_ENVIRONMENT_H___
