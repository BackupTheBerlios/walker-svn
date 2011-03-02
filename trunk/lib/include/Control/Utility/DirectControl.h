#ifndef __WALKER_YARD_CONTROL_CHAIN_DIRECT_CONTROL_H__
#define __WALKER_YARD_CONTROL_CHAIN_DIRECT_CONTROL_H__

#include "../PhysicsControl.h"
#include "EnvironmentWrapper.h"

namespace ctrl {

template<typename Environment,
         typename ControlFunction>
class DirectControl :
    public PhysicsControl
{
private:
    template<typename T>
    struct transfer
    {
        typedef const T& type;
    };

    template<typename T>
    struct transfer<const T&>
    {
        typedef const T& type;
    };

    template<typename T>
    struct transfer<T&>
    {
        typedef T& type;
    };

public:
	typedef ControlFunction	                    control_function;
    typedef boost::intrusive_ptr<Environment>   environment_ptr;
	typedef environment_wrapper<Environment>    environment_wrapper_type;

public:
    DirectControl(const loose_timer_ptr&                    timer_,
				  const environment_ptr&                    environment_,
                  typename transfer<control_function>::type controlFunction_,
                  double                                    interval_ = -1.0)
	:	PhysicsControl(timer_, false)
	,	environmentWrapper(environment_)
	,	controlFunction(controlFunction_)
    ,   interval(interval_)
	{
    }

    ~DirectControl()
    {
        if ( acquired() ) {
            unacquire();
        }
    }

public:
    void setControlFunction(typename transfer<control_function>::type cf)
    {
        controlFunction = cf;
    }

    typename transfer<control_function>::type getControlFunction() const 
    {
        return controlFunction;
    }

    void setEnvironment(const environment_ptr& env)
    {
        environmentWrapper = environment_type(environment);
    }

    const environment_ptr& getEnvironment() const
    {
        return environment;
    }

private:
    // Override PhysicsControl
    double pre_sync()
	{
        std::vector<float> state( environmentWrapper.state_size() );
        std::vector<float> action( environmentWrapper.action_size() );

        environmentWrapper.query_state( state.begin(), true );
        controlFunction.compute( state.begin(), state.end(), action.begin() );
        environmentWrapper.perform_action( action.begin(), action.end(), true );

		return interval;
	}

private:
    double                      interval;
    environment_wrapper_type	environmentWrapper;
    control_function	        controlFunction;
};

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_DIRECT_CONTROL_H__
