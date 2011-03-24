#ifndef __WALKER_YARD_PLAIN_DIRECT_RL_HPP___
#define __WALKER_YARD_PLAIN_DIRECT_RL_HPP___

#include <boost/random.hpp>
#include <boost/numeric/ublas/matrix.hpp>


namespace learn {

namespace ublas = boost::numeric::ublas;

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
class plain_direct_rl
{
public:
    typedef ValueType           value_type;
    typedef Environment         environment_type;
    typedef ActionFunction      action_function_type;

private:
    static  boost::mt19937      rng;

private:
    typedef boost::variate_generator< boost::mt19937, 
                                      boost::normal_distribution<value_type> >  normal_variate_generator;
    typedef boost::variate_generator< boost::mt19937, 
                                      boost::uniform_real<value_type> >         uniform_variate_generator;
    
    typedef ublas::matrix<value_type>   matrix_type;
    typedef ublas::vector<value_type>   vector_type;
public:
    plain_direct_rl( environment_type        _environment,
                           action_function_type    _bestAction);
    vector_type new_episode(vector_type state, value_type reward);
    vector_type update(vector_type state, value_type reward);

    size_t                      get_num_episodes() const        { return 0; }
    size_t                      get_num_actions() const         { return 0; }
    const value_type*           get_last_state() const          { return &lastState[0]; }
    const value_type*           get_last_action() const         { return &lastAction[0]; }
    value_type                  get_last_reward() const         { return lastReward; }

    environment_type            get_environment() const          { return environment; }
    action_function_type        get_best_action_function() const { return bestAction; }

private:
    matrix_type mult(const matrix_type& mat, const vector_type& vec)
    {
        assert( vec.size() == mat.size1() );

        matrix_type res(mat);
        for (size_t i = 0; i < mat.size1(); ++i)
        {
            for (size_t j = 0; j < mat.size2(); ++j)
            {
                res(i, j) *= vec(i);
            }
        }
        return res;
    }

public:
    value_type  sigma;
    value_type  beta;
    value_type  gamma;

private:
    // RL primitives
    environment_type            environment;
    action_function_type        bestAction;

    // internal 
    size_t                      stateSize, actionSize;
    value_type                  lastReward;
    vector_type                 lastState;
    vector_type                 lastAction;
    vector_type                 lastBestAction;
    vector_type                 currentState;
    matrix_type                 Z;
    matrix_type                 theta;
    normal_variate_generator    normal_sampler;
    uniform_variate_generator   uniform_sampler;
   bool    debug;
};


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
plain_direct_rl<ValueType, Environment, ActionFunction>::plain_direct_rl( Environment       _environment,
                                                                                      ActionFunction    _bestAction)
:   environment(_environment)
,   bestAction(_bestAction)
,   stateSize( environment.state_size() )
,   actionSize( environment.action_size() )
,   lastState(stateSize)
,   lastAction(actionSize)
,   lastBestAction(actionSize)
,   currentState(stateSize)
,   Z(_bestAction.num_components(), _bestAction.num_params()/_bestAction.num_components())
,   theta(_bestAction.num_components(), _bestAction.num_params()/_bestAction.num_components())
,   normal_sampler( rng, boost::normal_distribution<value_type>( 0, value_type(0.1) ) )
,   uniform_sampler( rng, boost::uniform_real<value_type>(-1.0, 1.0) )
,   sigma(0.05f)
,   beta(0.9f)
,   gamma(0.002f)
{
    debug = false;
    std::fill(theta.data().begin(), theta.data().end(), value_type(0));
}


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
vector_type plain_direct_rl<ValueType, Environment, ActionFunction>::new_episode(vector_type state, value_type reward)
{
    if (episode_number >= 1)
    {
        matrix_type delta;
        // ... compute delta
        matrix_type gradient(bestAction.num_components(), bestAction.num_params()/bestAction.num_components());
        bestAction.get_gradient(lastState.begin(), lastState.end(), gradient.data().begin()); // lastState?
        delta = (1/(sigma*sigma))*mult(gradient, lastAction - lastBestAction);
        Z = beta*Z + delta;
        bestAction.get_params(theta.data().begin());
        theta += gamma*reward*Z;
        bestAction.set_params(theta.data().begin(), theta.data().end());
    }
    Z = matrix_type( bestAction.num_components(),
                     bestAction.num_params() / bestAction.num_components() );
    std::fill(Z.data().begin(), Z.data().end(), value_type(0));
    //environment.query_state( lastState.begin() );
    
    
    // select first action
    bestAction.compute( state.begin(), 
                        state.end(), 
                        lastBestAction.begin() );
    for (size_t i = 0; i < actionSize; ++i)
    {
        //lastAction[i] += (*normal_sampler[i])();
        lastAction[i] = lastBestAction[i] + normal_sampler();
    }

    lastReward = environment.perform_action( lastAction.begin(),
                                             lastAction.end() );
    //debug = true;
    
    ++episode_number;
    return lastAction;
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
vector_type plain_direct_rl<ValueType, Environment, ActionFunction>::update(vector_type state, value_type reward)
{
    matrix_type delta;
    // ... compute delta
    matrix_type gradient(bestAction.num_components(), bestAction.num_params()/bestAction.num_components());
    bestAction.get_gradient(lastState.begin(), lastState.end(), gradient.data().begin()); // lastState?
    delta = (1/(sigma*sigma))*mult(gradient, lastAction - lastBestAction);
    Z = beta*Z + delta;
    bestAction.get_params(theta.data().begin());
    theta += gamma*reward*Z;
    bestAction.set_params(theta.data().begin(), theta.data().end());

    //debug = false;
    if (terminal)
    {
        return true;
    }
    // select new action
    bestAction.compute( state.begin(), 
                        state.end(), 
                        lastBestAction.begin() );
      
    // sum with variation
    for (size_t i = 0; i < actionSize; ++i)
    {
        //lastBestAction[i] = std::max(std::min(lastBestAction[i], value_type(1.0)), value_type(-1.0));
        //lastAction[i] += (*normal_sampler[i])();
        lastAction[i] = lastBestAction[i] + normal_sampler();
        //lastAction[i] = std::max(std::min(lastAction[i], 1.0f), -1.0f);
    }

    /*
            value_type distance = value_type();
        for (size_t i = 0; i<currentState.size(); ++i) {
            distance += sqrt( (currentState[i] - lastState[i]) * (currentState[i] - lastState[i]) );
        }

        if (distance < 0.01)
        {
            std::cerr << "random action" << std::endl;
            for (size_t i = 0; i < actionSize; ++i)
            {
                //lastAction[i] += (*normal_sampler[i])();
                lastAction[i] = uniform_sampler();
            }
        }
*/
    lastState.swap(state);
    lastReward = environment.perform_action( lastAction.begin(), 
                                             lastAction.end() );
    gamma *= 0.99f;
    if (gamma < 0.0005f) gamma = 0.0005f;
    return lastAction;
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
boost::mt19937 plain_direct_rl<ValueType, Environment, ActionFunction>::rng; 

} // namespace learn

#endif // __WALKER_YARD_PLAIN_DIRECT_RL_HPP___