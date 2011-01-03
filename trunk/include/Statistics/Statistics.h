#ifndef __WALKER_YARD_STATISTICS_STATISTICS_H__
#define __WALKER_YARD_STATISTICS_STATISTICS_H__

#include <slon/Utility/referenced.hpp>

namespace stats {

template<class Observed>
class Statistics :
    public slon::Referenced
{
public:
    typedef Observed observed;

public:
    /** Gather some information from RL */
    virtual void gather(const observed&) = 0;

    virtual ~Statistics() {}
};

} // namespace stat

#endif // __WALKER_YARD_STATISTICS_RL_STATISTICS_H__
