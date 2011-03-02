#ifndef __WALKER_YARD_REINFORCEMENT_LEARNING_HPP___
#define __WALKER_YARD_REINFORCEMENT_LEARNING_HPP___

#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <vector>

namespace learn
{

template < typename ValueType,
           typename Environment,
           typename ValueFunction,
           typename BestActionFunction >
class reinforcement_learning 
{
public:
    typedef ValueType                       value_type;
    typedef Environment                     environment_type;
    typedef ValueFunction                   value_function_type;
    typedef BestActionFunction              best_action_function_type;

private:
    static  boost::mt19937                  rng;

private:
    typedef std::vector<ValueType>  state_vector;
    typedef std::vector<ValueType>  action_vector;

    typedef boost::variate_generator< boost::mt19937, 
                                      boost::normal_distribution<value_type> >  normal_variate_generator;

public:
    reinforcement_learning( environment_type             _environment,
                            value_function_type          _valueFunction,
                            best_action_function_type    _bestAction) :
        environment(_environment),
        valueFunction(_valueFunction),
        bestAction(_bestAction),
        stateSize( environment.state_size() ),
        actionSize( environment.action_size() ),
        lastAction(actionSize),
        currentState(stateSize),
        lastState(stateSize),
        alpha(0.1),
        gamma(0.999),
        beta(0.1),
        normal_sampler( rng, boost::normal_distribution<ValueType>(0, 0.1) )
    {
        //T sigma = 1.0;
        //boost::mt19937 rng( static_cast<unsigned>( std::time(0) ));
        //boost::normal_distribution<T> norm_dist(0, sigma);
        //normal_sampler = normal_variate_generator(rng, norm_dist);
        //normal_sampler = std::vector<normal_variate_generator_ptr>(actionMap->getNumPins(), normal_variate_generator_ptr());
        //for (int i = 0; i < normal_sampler.size(); ++i)
        //{
        //    normal_sampler[i] = normal_variate_generator_ptr(new normal_variate_generator(rng, norm_dist));
        //}
        // TODO initialize valueFunction and bestAction
		numEpisodes = 0;
    }

    void new_episode()
    {
        numActions   = 0;
		numEpisodes += 1;
		if (numEpisodes > 300) 
			normal_sampler = normal_variate_generator( rng, boost::normal_distribution<ValueType>(0, 0.1) );

        environment.query_state( lastState.begin() );
        lastValue = valueFunction.compute( lastState.begin(), 
                                           lastState.end() );

        // select first action
        bestAction.compute( lastState.begin(), 
                            lastState.end(), 
                            lastAction.begin() );

        for (size_t i = 0; i < actionSize; ++i)
        {
            //lastAction[i] += (*normal_sampler[i])();
            lastAction[i] += normal_sampler();
        }
        
        lastReward = environment.perform_action( lastAction.begin(), 
                                                 lastAction.end() );
    }

    bool update()
    {
        bool terminal = environment.query_state( currentState.begin() );
        if (terminal)
        {
            // V_{t+1}(s_t) <- r_t
            valueFunction.update( lastState.begin(), 
                                  lastState.end(), 
                                  lastReward, 
                                  beta );
            return true;
        }
        else  // nonterminal state
        {
            // V_{t+1}(s_t) <- r_t + gamma*V_t(s_{t+1})
            ValueType value = lastReward
                            + gamma * valueFunction.compute( currentState.begin(), currentState.end() );

            valueFunction.update( lastState.begin(), 
                                  lastState.end(), 
                                  value,
                                  beta );
            value = valueFunction.compute( lastState.begin(), lastState.end() );

            if (value > lastValue)
            {
                // update best action
                bestAction.update( lastState.begin(), 
                                   lastState.end(), 
                                   lastAction.begin(), 
                                   lastAction.end(), 
                                   alpha );
            }
            lastValue = valueFunction.compute( currentState.begin(), currentState.end() );
        }

        // select new action
        bestAction.compute( currentState.begin(), 
                            currentState.end(), 
                            lastAction.begin() );
        
        // sum with variation
        for (size_t i = 0; i < actionSize; ++i)
        {
            //lastAction[i] += (*normal_sampler[i])();
            lastAction[i] += normal_sampler();
        }
        lastState.swap(currentState);
        lastReward = environment.perform_action( lastAction.begin(), 
                                                 lastAction.end() );
        ++numActions;

        return false;
    }

    size_t                      get_num_episodes() const            { return numEpisodes; }
    size_t                      get_num_actions() const             { return numActions; }
    const value_type*           get_last_state() const              { return &lastState[0]; }
    const value_type*           get_last_action() const             { return &lastAction[0]; }
    value_type                  get_last_reward() const             { return lastReward; }

    environment_type            get_environment() const             { return environment; }
    value_function_type         get_value_function() const          { return valueFunction; }
    best_action_function_type   get_best_action_function() const    { return bestAction; }

private:
    // RL primitives
    environment_type            environment;
    value_function_type         valueFunction;
    best_action_function_type   bestAction;

    // internal 
    size_t                      stateSize, actionSize;
    value_type                  alpha, beta, gamma;
    value_type                  lastReward;
    value_type                  lastValue;
    action_vector               lastAction;
    state_vector                currentState;
    state_vector                lastState;
    //std::vector<normal_variate_generator_ptr>   normal_sampler;
    normal_variate_generator    normal_sampler;

    size_t                      numActions;
public:
	int numEpisodes;
};

template < typename ValueType,
           typename Environment,
           typename ValueFunction,
           typename BestActionFunction >
boost::mt19937 reinforcement_learning<ValueType, Environment, ValueFunction, BestActionFunction>::rng; 

}

#endif  // __WALKER_YARD_REINFORCEMENT_LEARNING_HPP___