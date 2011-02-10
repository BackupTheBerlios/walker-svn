#ifndef __WALKER_YARD_CONTROL_CHAIN_HEURISTIC_CONTROL_H___
#define __WALKER_YARD_CONTROL_CHAIN_HEURISTIC_CONTROL_H___

#include "Environment.h"

namespace ctrl {
namespace chain {

class HeuristicControl :
    public slon::Referenced
{
public:
    HeuristicControl(const ChainEnvironment* env);

    void makeAction();

    template<typename OutIter>
    void queryAction(OutIter out) const
    {
        const_cast<HeuristicControl*>(this)->makeAction();
        for(size_t i = 0; i<env->getActionSize(); ++i, ++out)
        {
            (*out) = motorForces[i] / env->maxForce;
        }
    }

private:
    // control target
    const_chain_environment_ptr env;

    // action
    std::vector<float> motorForces;
};

typedef boost::intrusive_ptr<HeuristicControl>          heuristic_control_ptr;
typedef boost::intrusive_ptr<const HeuristicControl>    const_heuristic_control_ptr;

} // namespace chain
} // namesapce ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_HEURISTIC_CONTROL_H___