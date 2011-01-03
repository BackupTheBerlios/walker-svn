#ifndef __WALKER_YARD_CONTROL_PHYSICS_ENVIRONMENT_H___
#define __WALKER_YARD_CONTROL_PHYSICS_ENVIRONMENT_H___

#define NOMINMAX
#include "../SlonEngine.h"
#include "../Utility/Math.h"
#include "Environment.h"
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <set>
#include <sgl/Math/Containers.hpp>
#include <sgl/Math/Quaternion.hpp>
#include <slon/Physics/Constraint.h>
#include <slon/Physics/DynamicsWorld.h>
#include <slon/Physics/RigidBody.h>

namespace ctrl {

template<typename T> class rl_wrapper;

class PhysicsEnvironment :
    public Environment,
    public boost::noncopyable
{
template<typename T> friend class environment_wrapper;
public:
    typedef std::vector<physics::rigid_body_ptr>    rigid_body_vector;
    typedef std::vector<physics::Motor*>            motor_vector;
    typedef std::vector<physics::constraint_ptr>    constraint_vector;

    struct DESC
    {
        float                   maxForce;
        float                   maxVelocity;
        bool                    freeJoints;
        unsigned                numRigidBodies;
        physics::RigidBody**    rigidBodies;
        unsigned                numConstraints;
        physics::Constraint**   constraints;
    };

    enum CONTROL_TYPE
    {
        CONTROL_FORCE,
        CONTROL_VELOCITY
    };

private:
    typedef std::pair<const physics::CollisionObject*, 
                      const physics::CollisionObject*>  contact_pair;
    typedef std::set<contact_pair>                      contact_set;

    typedef boost::signals::scoped_connection                       scoped_connection;
    typedef boost::shared_ptr<boost::signals::scoped_connection>    connection_ptr;
    typedef std::vector<connection_ptr>                             connection_vector;

public:
    PhysicsEnvironment(const DESC& desc);

    // Override Environment
    void reset();
    void makeState();
    void makeAction();
    void setControlType(CONTROL_TYPE controlType);

    // get
    CONTROL_TYPE getControlType() const  { return controlType; }
    STATE        getState() const        { return state; }
    size_t       getStateSize() const    { return stateSize; }
    size_t       getActionSize() const   { return actionSize; }
    float        getMaxForce() const     { return maxForce; }
    float        getMaxVelocity() const  { return maxVelocity; }

private:
    void handleAppearingContact(const physics::Contact& c);
    void handleDissappearingContact(const physics::Contact& c);

protected:
    rigid_body_vector           rigidBodies;
    constraint_vector           constraints;
    motor_vector                motors;
    contact_set                 activeContacts;
    connection_vector           contactConnections;

    // Sync
    boost::mutex                stateMutex;
    boost::mutex                actionMutex;
    boost::mutex                rewardMutex;
    boost::mutex                contactUpdateMutex;
    boost::condition_variable   stateReadyCondition;
    boost::condition_variable   actionReadyCondition;
    boost::condition_variable   rewardReadyCondition;

    // RL
    size_t                      stateSize;
    size_t                      actionSize;
    STATE                       state;

    unsigned                    numDynamicRigidBodies;
    float                       centerHeight;
    float                       initialCenterHeight;
    math::Vector3f              initialMassCenter;
    float                       maxVelocity;
    float                       maxForce;
    bool                        freeJoints;
    double                      updateTime;

    // state
    math::Quaternionf           rotation;
    math::Vector3f              angVelocity;

    // action
    CONTROL_TYPE                controlType;

public:
    ublas::vector<float>        position;
    ublas::vector<float>        velocity;
    ublas::vector<float>        targetForce;
    ublas::vector<float>        targetVelocity;
};

typedef boost::intrusive_ptr<PhysicsEnvironment>          physics_environment_ptr;
typedef boost::intrusive_ptr<const PhysicsEnvironment>    const_physics_environment_ptr;

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_PHYSICS_ENVIRONMENT_H___
