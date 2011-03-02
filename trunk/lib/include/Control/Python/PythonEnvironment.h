#ifndef __WALKER_YARD_CONTROL_PYTHON_PYTHON_ENVIRONMENT_H___
#define __WALKER_YARD_CONTROL_PYTHON_PYTHON_ENVIRONMENT_H___

#define NOMINMAX
#include "../Environment.h"
#include <sgl/Math/Containers.hpp>
#include <sgl/Math/Quaternion.hpp>
#include <slon/Physics/Constraint.h>
#include <slon/Physics/DynamicsWorld.h>
#include <slon/Physics/RigidBody.h>
#//include <boost/python.hpp>
#include <boost/thread.hpp>
#include <set>

namespace ctrl {

using namespace slon;

class PythonEnvironment :
    public Environment
{
template<typename T> friend class rl_wrapper;
public:
    typedef std::vector<physics::rigid_body_ptr>        rigid_body_vector;
    typedef std::vector<physics::RotationalMotor*>      rotational_motor_vector;
    typedef std::vector<physics::constraint_ptr>        constraint_vector;

    struct DESC
    {
        float                   maxForce;
        float                   maxVelocity;
        unsigned                numRigidBodies;
        physics::RigidBody**    rigidBodies;
        unsigned                numConstraints;
        physics::Constraint**   constraints;
    };

private:
    typedef std::pair<const physics::CollisionObject*, 
                      const physics::CollisionObject*>  contact_pair;
    typedef std::set<contact_pair>                      contact_set;

    typedef boost::signals::scoped_connection                       scoped_connection;
    typedef boost::shared_ptr<boost::signals::scoped_connection>    connection_ptr;
    typedef std::vector<connection_ptr>                             connection_vector;

private:
    // noncpyable
    PythonEnvironment(const PythonEnvironment&);
    PythonEnvironment& operator = (const PythonEnvironment&);

public:
    PythonEnvironment(const DESC& desc);

    // load python implementation
    void loadImpl(const std::string& pyFileName, const std::string& pyObject);

    // Override Environment
    void reset();
    void makeState();
    void makeAction();
    void makeReward();

    // get
    STATE   getState() const        { return state; }
    size_t  getStateSize() const    { return stateSize; }
    size_t  getActionSize() const   { return actionSize; }
    float   getMaxForce() const     { return maxForce; }
    float   getMaxVelocity() const  { return maxVelocity; }
    float   getReward() const       { return reward; }

private:
    void handleAppearingContact(const physics::Contact& c);
    void handleDissappearingContact(const physics::Contact& c);

private:
    // impl
    // boost::python::object       impl;

    // 
    rigid_body_vector           rigidBodies;
    constraint_vector           constraints;
    rotational_motor_vector     motors;
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

    float                       centerHeight;
    float                       initialCenterHeight;
    float                       maxVelocity;
    float                       maxForce;
    double                      updateTime;

    // state
    bool                        terminal;
    math::Quaternionf           rotation;
    math::Vector3f              angVelocity;
    math::vector_of_vector2f    constraintAngles;
    math::vector_of_vector2f    constraintVelocities;

    // action
    std::vector<float>          motorForces;
    std::vector<float>          motorVelocities;

    // reward
    float                       reward;
};

template<>
class rl_wrapper<PythonEnvironment>
{
public:
    typedef PythonEnvironment environment_type;

public:
    rl_wrapper(environment_type& env_) 
    :   env(env_) 
    {}

    // Implement Environment
    size_t state_size()  const { return env.stateSize; }
    size_t action_size() const { return env.actionSize; }

    template<typename InIterator>
    float perform_action(InIterator firstPin, InIterator endPin)
    {
        size_t i = 0;
        for (InIterator pin = firstPin; pin != endPin; ++pin, ++i)
        {
            float force = float(*pin);
            if (force > 1.0f) {
                force = 1.0f;
            }
            else if (force < -1.0f) {
                force = -1.0f;
            }

            env.motorForces[i]      = fabs(force * env.maxForce);
            env.motorVelocities[i]  = force * env.maxVelocity;
        }

        // wait for action
        {
            boost::unique_lock<boost::mutex> stateLock(env.stateMutex);
            env.state = Environment::WAITING_ACTION;
            env.actionReadyCondition.wait( stateLock, boost::bind(boost::bind(&environment_type::state, &env) == Environment::READY) );
        }

        // wait for reward
        {
            boost::unique_lock<boost::mutex> stateLock(env.stateMutex);
            env.state = Environment::WAITING_REWARD;
            env.rewardReadyCondition.wait( stateLock, boost::bind(boost::bind(&environment_type::state, &env) == Environment::READY) );
        }

        return env.reward;
    }

    template<typename OutIterator>
    bool query_state(OutIterator stateOut)
    {
        using namespace math;

        // wait for state
        {
            boost::unique_lock<boost::mutex> lock(env.stateMutex);
            env.state = Environment::WAITING_STATE;
            env.stateReadyCondition.wait( lock, boost::bind(boost::bind(&environment_type::state, &env) == Environment::READY) );
        }

        //// copy base body orientation
        //{
        //    *stateOut++ = rotation.x;  
        //    *stateOut++ = rotation.y;
        //    *stateOut++ = rotation.z;

        //    *stateOut++ = angVelocity.x;
        //    *stateOut++ = angVelocity.y;
        //    *stateOut++ = angVelocity.z;
        //}

        // copy state
        for (size_t i = 0; i<env.constraintAngles.size(); ++i) 
        {
            Vector3f lowLimit  = env.constraints[i]->getStateDesc().angularLimits[0];
            Vector3f highLimit = env.constraints[i]->getStateDesc().angularLimits[1];
            *stateOut++        = 2.0f * (env.constraintAngles[i].x - lowLimit.x) / (highLimit.x - lowLimit.x) - 1.0f;
            *stateOut++        = 2.0f * (env.constraintAngles[i].y - lowLimit.y) / (highLimit.y - lowLimit.y) - 1.0f;

            //*stateOut++ = 2.0f * std::min(std::max(constraintVelocities[i].x, -maxVelocity), maxVelocity) / maxVelocity - 1.0f;
            //*stateOut++ = 2.0f * std::min(std::max(constraintVelocities[i].y, -maxVelocity), maxVelocity) / maxVelocity - 1.0f;
        }

        return env.terminal;
    }

private:
    environment_type& env;
};
    
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_PYTHON_PYTHON_ENVIRONMENT_H___
