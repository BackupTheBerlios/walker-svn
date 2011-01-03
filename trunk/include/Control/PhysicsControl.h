#ifndef __WALKER_YARD_PHYSICS_CONTROL_H__
#define __WALKER_YARD_PHYSICS_CONTROL_H__

#include "Control.h"
#include "Utility/LooseTimer.h"
#include <boost/intrusive_ptr.hpp>
#include <boost/signals/connection.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <slon/Physics/PhysicsModel.h>

namespace ctrl {

/** Double threaded physics control. Separates logic in different thread. */
class PhysicsControl :
    public Control
{
public:
    PhysicsControl(const loose_timer_ptr& timer, bool multithreaded = true);

    // Override Control
    void acquire();
    void unacquire();
    void loadConfig(const std::string& fileName);

    /** Setup visual model for control. Calls unacquire_safe, then acquire_safe. */
    virtual void setTargetModel(const scene::node_ptr& targetModel);

    /** Setup physics model for control. Calls unacquire_safe, then acquire_safe. */
    virtual void setPhysicsModel(const physics::physics_model_ptr& physicsModel);

    /** Check wether control acquired */
    virtual bool acquired() const { return isAcquired; }

    virtual ~PhysicsControl();

protected:
    /** Override this function to prepare control. Called from main thread, don't interfere with physics and render. */
    virtual void acquire_safe() {}

    /** Override this function to disable control. Called from main thread, don't interfere with physics and render. */
    virtual void unacquire_safe() {}

    /** Override this function to perform synchronization logic. Called from main thread, don't interfere with physics and render. 
	 * @return return delta time to next sync call, or -1.0 if indifferent.
	 */
    virtual double pre_sync() { return -1.0; }
	
    /** Override this function to perform synchronization logic. Called from main thread, don't interfere with physics and render. 
	 * @return return delta time to next sync call, or -1.0 if indifferent.
	 */
	virtual double post_sync() { return -1.0; }

    /** Perform controller logic here. */
	virtual void run() {}

private:
    // accept from main thread
    void prePhysicsCallback();
    void postPhysicsCallback();

    // accept from physics thread
    // void accept_physics_thread();

private:
    bool                        isAcquired;
    bool                        needReacquire;
	bool						multithreaded;
    boost::thread               logicThread;
    boost::signals::connection  preConnection;
    boost::signals::connection  postConnection;
	TimeBarrier					preTimerBarrier;
	TimeBarrier					postTimerBarrier;

protected:
    boost::mutex                syncMutex;
    scene::node_ptr             targetModel;
    physics::physics_model_ptr  physicsModel;
	loose_timer_ptr				timer;
    double                      time;
};

typedef boost::intrusive_ptr<PhysicsControl>		physics_control_ptr;
typedef boost::intrusive_ptr<const PhysicsControl>	const_physics_control_ptr;

} // namespace ctrl

#endif // __WALKER_YARD_PHYSICS_CONTROL_H__
