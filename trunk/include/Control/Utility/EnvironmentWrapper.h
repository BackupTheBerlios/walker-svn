#ifndef __WALKER_YARD_CONTROL_UTILITY_RL_ENVIRONMENT_WRAPPER_H___
#define __WALKER_YARD_CONTROL_UTILITY_RL_ENVIRONMENT_WRAPPER_H___


namespace ctrl
{

template<typename Environment>
class environment_wrapper
{
public:
    typedef Environment                         environment_type;
    typedef boost::intrusive_ptr<Environment>   environment_ptr;

public:
    environment_wrapper(environment_ptr env_) 
    :   env(env_) 
    {}

    // Implement Environment
    size_t state_size()  const { return env->stateSize; }
    size_t action_size() const { return env->actionSize; }

    template<typename T>
    T sgn(T val) 
    {
        if (val > T(0)) return T(1);
        if (val < T(0)) return T(-1);
        return T(0);
    }

    template<typename InIterator>
    float perform_action(InIterator firstPin, InIterator endPin, bool nowait = false)
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

            env->targetVelocity[i] = force * env->maxVelocity;
        }

        // wait for action
		if (nowait) {
			env->makeAction();
		}
		else 
        {
            boost::unique_lock<boost::mutex> stateLock(env->stateMutex);
            env->state = Environment::WAITING_ACTION;
            env->actionReadyCondition.wait( stateLock, boost::bind(boost::bind(&environment_type::state, env) == Environment::READY) );
        }

        // wait for reward
		if (nowait) {
			env->makeReward();
		}
		else 
        {
            boost::unique_lock<boost::mutex> stateLock(env->stateMutex);
            env->state = Environment::WAITING_REWARD;
            env->rewardReadyCondition.wait( stateLock, boost::bind(boost::bind(&environment_type::state, env) == Environment::READY) );
        }

		env->state = Environment::READY;
        return env->reward;
    }

    template<typename OutIterator>
    bool query_state(OutIterator stateOut, bool nowait = false)
    {
        using namespace math;

        // wait for state
		if (nowait) {
			env->makeState();
		}
		else 
        {
            boost::unique_lock<boost::mutex> lock(env->stateMutex);
            env->state = Environment::WAITING_STATE;
            env->stateReadyCondition.wait( lock, boost::bind(boost::bind(&environment_type::state, env) == Environment::READY) );
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
        for (size_t i = 0; i<env->motors.size(); ++i) 
        {
            float hi = env->motors[i]->getHiLimit();
            float lo = env->motors[i]->getLoLimit();
            if (hi > lo) {
                *stateOut++ = 2.0f * (env->position[i] - lo) / (hi - lo) - 1.0f;
            }
            else {
                *stateOut++ = env->position[i] / math::PI - 1.0f;
            }

            //*stateOut++ = 2.0f * std::min(std::max(env->constraintVelocities[i].x, -env->maxVelocity), env->maxVelocity) / env->maxVelocity - 1.0f;
            //*stateOut++ = 2.0f * std::min(std::max(env->constraintVelocities[i].y, -env->maxVelocity), env->maxVelocity) / env->maxVelocity - 1.0f;
        }

        return env->terminal;
    }

private:
    environment_ptr env;
};

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_UTILITY_RL_ENVIRONMENT_WRAPPER_H___