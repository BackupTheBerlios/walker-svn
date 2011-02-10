#ifndef __WALKER_YARD_STUB_LEARNING_HPP___
#define __WALKER_YARD_STUB_LEARNING_HPP___

#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <vector>

namespace learn
{

template < typename ValueType,
           typename Environment,
           typename ValueFunction,
           typename BestActionFunction >
class stub_learning 
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
    stub_learning( environment_type             _environment,
                            value_function_type          _valueFunction,
                            best_action_function_type    _bestAction) :
        environment(_environment),
        valueFunction(_valueFunction),
        bestAction(_bestAction),
        normal_sampler( rng, boost::normal_distribution<ValueType>(0, 0.2) ),
        currentState(environment.state_size())
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
		numEpisodes += 1;

        std::vector<ValueType> action(environment.action_size(), 1.0);
        environment.perform_action( action.begin(), 
                                                 action.end() );
    }

    bool update()
    {
        bool terminal = environment.query_state( currentState.begin() );
        if (terminal)
        {
            return true;
        }
        else  // nonterminal state
        {
 
            std::vector<ValueType> action(environment.action_size(), 1.0);
            environment.perform_action( action.begin(), 
                                                 action.end() );

            return false;
        }
    }

    environment_type            get_environment()           { return environment; }
    value_function_type         get_value_function()        { return valueFunction; }
    best_action_function_type   get_best_action_function()  { return bestAction; }

private:
    // RL primitives
    environment_type            environment;
    value_function_type         valueFunction;
    best_action_function_type   bestAction;

    normal_variate_generator    normal_sampler;
    std::vector<ValueType>      currentState;
public:
	int numEpisodes;
};

template < typename ValueType,
           typename Environment,
           typename ValueFunction,
           typename BestActionFunction >
boost::mt19937 stub_learning<ValueType, Environment, ValueFunction, BestActionFunction>::rng; 

}

#endif  // __WALKER_YARD_STUB_LEARNING_HPP___