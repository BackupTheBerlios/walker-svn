#include "Control/Utility/LooseTimer.h"
#include <algorithm>
#include <boost/thread/locks.hpp>
#include <sgl/Math/Base.hpp>
#include <sgl/Math/Utility.hpp>

namespace 
{
    typedef boost::shared_lock<boost::shared_mutex>             shared_lock;
    typedef boost::unique_lock<boost::shared_mutex>             unique_lock;
    typedef boost::upgrade_lock<boost::shared_mutex>            upgrade_lock;
    typedef boost::upgrade_to_unique_lock<boost::shared_mutex>  upgrade_to_unique_lock;
}

namespace ctrl {

using namespace slon;

LooseTimer::LooseTimer(double tolerance_) :
    pause(false),
	realtime(false),
    time(0.0),
    timeScale(1.0),
	tolerance(tolerance_)
{
    unique_lock lock(mutex);

    StartStopTimer* startStopTimer = new StartStopTimer();
    startStopTimer->start();
    deltaTimer = delta_timer(startStopTimer);
}

bool LooseTimer::myIsBounded() const
{
	return !barriers.empty();
}

double LooseTimer::myGetMaxTime() const
{
    return myIsBounded() ? barriers.back() : time;
}

void LooseTimer::mySetTime(double time_)
{
    time = time_;
    if ( myIsBounded() ) {
        time = std::min(myGetMaxTime(), time);
    }
}

bool LooseTimer::isPaused() const
{ 
    shared_lock lock(mutex);
    return pause; 
}

void LooseTimer::togglePause(bool toggle)
{ 
    unique_lock lock(mutex);
    pause = toggle; 
}

bool LooseTimer::isRealtime() const		    
{ 
    shared_lock lock(mutex);
    return realtime; 
}

void LooseTimer::toggleRealtime(bool toggle) 
{ 
    unique_lock lock(mutex);
    realtime = toggle; 
}

double LooseTimer::getTime() const 
{
    upgrade_lock lock(mutex);

    double dt = deltaTimer();
    if (!pause) 
	{
        upgrade_to_unique_lock ulock(lock);
		if (realtime) {
            time = myGetMaxTime();
		}
		else {
            const_cast<LooseTimer*>(this)->mySetTime(time + dt * timeScale);
		}
    }

    return time;
}

void LooseTimer::setTime(double time)
{
    unique_lock lock(mutex);
    mySetTime(time);
}

void LooseTimer::setTimeScale(double timeScale_)
{
    unique_lock lock(mutex);
    timeScale = std::max(0.0, timeScale_);
}

bool LooseTimer::isBounded() const
{
    shared_lock lock(mutex);
	return myIsBounded();
}

double LooseTimer::getMaxTime() const   
{ 
    shared_lock lock(mutex);
    return myGetMaxTime();
}

double LooseTimer::getTimeScale() const
{
    shared_lock lock(mutex);
    return timeScale;
}

double LooseTimer::getTolerance() const
{
    shared_lock lock(mutex);
    return tolerance;
}

void LooseTimer::addBarrier(double barrier)
{
    unique_lock lock(mutex);
	barriers.push_back(barrier);
	std::push_heap(barriers.begin(), barriers.end());
}

void LooseTimer::removeBarrier(double barrier)
{
    unique_lock lock(mutex);
    double_vector::iterator iter = std::remove(barriers.begin(), barriers.end(), barrier);
    assert( iter != barriers.end() );
	barriers.erase(iter);
}

} // namespace ctrl

