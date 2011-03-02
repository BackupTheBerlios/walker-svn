#include "Control/PhysicsControl.h"
#include <boost/thread/locks.hpp>
#include <slon/Physics/PhysicsManager.h>

namespace ctrl {
   
PhysicsControl::PhysicsControl(const loose_timer_ptr& timer_, bool multithreaded_)
:   isAcquired(false)
,	multithreaded(multithreaded_)
,	timer(timer_)
{
}

PhysicsControl::~PhysicsControl()
{
    if (isAcquired) {
        unacquire();
    }
}

void PhysicsControl::setTargetModel(const scene::node_ptr& targetModel_)
{
    if (isAcquired) 
    {
        unacquire();
        targetModel = targetModel_;
        acquire();
    }
    else {
        targetModel = targetModel_;
    }
}

void PhysicsControl::setPhysicsModel(const physics::physics_model_ptr& physicsModel_)
{
    if (isAcquired) 
    {
        unacquire();
        physicsModel = physicsModel_;
        acquire();
    }
    else {
        physicsModel = physicsModel_;
    }
}

void PhysicsControl::acquire()
{
    boost::lock_guard<boost::mutex> lock(syncMutex);

    time = timer->getTime();
    acquire_safe();
    isAcquired = true;
	if (multithreaded)
	{
		boost::thread thread( boost::bind(&PhysicsControl::run, this) );
		logicThread.swap(thread);
	}

    preConnection  = physics::currentPhysicsManager().connectPreFrameCallback( boost::bind(&PhysicsControl::prePhysicsCallback, this) );
    postConnection = physics::currentPhysicsManager().connectPostFrameCallback( boost::bind(&PhysicsControl::postPhysicsCallback, this) );
}

void PhysicsControl::unacquire()
{
    {
        boost::lock_guard<boost::mutex> lock(syncMutex);
        if (!isAcquired) {
            return;
        }
		
        isAcquired = false;
        unacquire_safe();
    }
    logicThread.join();
    preConnection.disconnect();
    postConnection.disconnect();

    preTimerBarrier.reset();
    postTimerBarrier.reset();
}

void PhysicsControl::loadConfig(const std::string& fileName)
{
}

void PhysicsControl::prePhysicsCallback()
{
    boost::lock_guard<boost::mutex> lock(syncMutex);

    time = timer->getTime();
    if ( isAcquired && (!preTimerBarrier || (time + timer->getTolerance() > preTimerBarrier.value())) ) 
	{
        double dt = pre_sync();
        preTimerBarrier.reset();
		if (dt >= 0) 
		{
			TimeBarrier tb(timer, time + dt);
			preTimerBarrier.swap(tb);
		}
	}
}

void PhysicsControl::postPhysicsCallback()
{
    boost::lock_guard<boost::mutex> lock(syncMutex);

    time = timer->getTime();
    if ( isAcquired && (!postTimerBarrier || (time + timer->getTolerance() > postTimerBarrier.value())) ) 
	{
        double dt = post_sync();
		postTimerBarrier.reset();
		if (dt >= 0) 
		{
			TimeBarrier tb(timer, time + dt);
			postTimerBarrier.swap(tb);
		}
	}
}

} // namespace ctrl
