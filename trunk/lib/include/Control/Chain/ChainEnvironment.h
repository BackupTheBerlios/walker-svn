#ifndef __WALKER_YARD_CONTROL_CHAIN_CHAIN_ENVIRONMENT_H___
#define __WALKER_YARD_CONTROL_CHAIN_CHAIN_ENVIRONMENT_H___

#define NOMINMAX
#include "../Environment.h"
#include <sgl/Math/Containers.hpp>
#include <sgl/Math/Quaternion.hpp>
#include <slon/Physics/Constraint.h>
#include <slon/Physics/DynamicsWorld.h>
#include <slon/Physics/RigidBody.h>
#include <boost/thread.hpp>
#include <set>

namespace ctrl {

using namespace slon;

class ChainEnvironment :
    public Environment
{
template<typename T> friend class rl_wrapper;
friend class HeuristicControl;
public:
    typedef std::vector<physics::rigid_body_ptr>        rigid_body_vector;
    typedef std::vector<physics::RotationalMotor*>      rotational_motor_vector;
    typedef std::vector<physics::constraint_ptr>        constraint_vector;

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

private:
    typedef std::pair<const physics::CollisionObject*, 
                      const physics::CollisionObject*>  contact_pair;
    typedef std::set<contact_pair>                      contact_set;

    typedef boost::signals::scoped_connection                       scoped_connection;
    typedef boost::shared_ptr<boost::signals::scoped_connection>    connection_ptr;
    typedef std::vector<connection_ptr>                             connection_vector;

private:
    // noncpyable
    ChainEnvironment(const ChainEnvironment&);
    ChainEnvironment& operator = (const ChainEnvironment&);

public:
    ChainEnvironment(const DESC& desc);

    void toggleEpisodic(bool toggle) {episodic = toggle;}
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

    unsigned                    numDynamicRigidBodies;
    float                       centerHeight;
    float                       initialCenterHeight;
    math::Vector3f              initialMassCenter;
    float                       maxVelocity;
    float                       maxForce;
    bool                        freeJoints;
    double                      updateTime;

    // state
    bool                        firstStep;
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

    float                       averagePos;
    float                       averageVel;
    bool                        episodic;
};

typedef boost::intrusive_ptr<ChainEnvironment>          chain_environment_ptr;
typedef boost::intrusive_ptr<const ChainEnvironment>    const_chain_environment_ptr;

template<>
class rl_wrapper<ChainEnvironment>
{
public:
    typedef ChainEnvironment environment_type;

public:
    rl_wrapper(environment_type& env_) 
    :   env(env_) 
    {}

    // Implement Environment
    size_t state_size()  const { return env.stateSize; }
    size_t action_size() const { return env.actionSize; }

    template<typename T>
    T sgn(T val) 
    {
        if (val > T(0)) return T(1);
        if (val < T(0)) return T(-1);
        return T(0);
    }

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

            if (env.freeJoints)
            {
                env.motorForces[i]      = fabs(force * env.maxForce);
                env.motorVelocities[i]  = sgn(force) * env.maxVelocity;
            }
            else
            {
                env.motorForces[i]      = env.maxForce;
                env.motorVelocities[i]  = force * env.maxVelocity;
            }
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

            //*stateOut++ = 2.0f * std::min(std::max(env.constraintVelocities[i].x, -env.maxVelocity), env.maxVelocity) / env.maxVelocity - 1.0f;
            //*stateOut++ = 2.0f * std::min(std::max(env.constraintVelocities[i].y, -env.maxVelocity), env.maxVelocity) / env.maxVelocity - 1.0f;
        }

        return env.terminal;
    }

private:
    environment_type& env;
};
    
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_CHAIN_ENVIRONMENT_H___
