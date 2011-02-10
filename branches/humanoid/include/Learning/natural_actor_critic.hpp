#ifndef __WALKER_YARD_PLAIN_DIRECT_LEARNING_HPP___
#define __WALKER_YARD_PLAIN_DIRECT_LEARNING_HPP___

#include <list>
#include <boost/random.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace learn {

namespace ublas = boost::numeric::ublas;

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
class natural_actor_critic
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
    natural_actor_critic( environment_type        _environment,
                           action_function_type    _bestAction);
    void new_episode();
    bool update();

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

private:
    // RL primitives
    environment_type            environment;
    action_function_type        bestAction;

    // internal
    std::list<vector_type>      stateTrack;
    std::list<value_type>       rewards;
    std::list<vector_type>      actionTrack;

    size_t                      stateSize, actionSize;
    vector_type                 currentState;
    matrix_type                 theta;
    normal_variate_generator    normal_sampler;
    uniform_variate_generator   uniform_sampler;
    value_type                  sigma, beta, gamma;
};


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
natural_actor_critic<ValueType, Environment, ActionFunction>::natural_actor_critic( Environment       _environment,
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
,   sigma(0.1f)
,   beta(0.9f)
,   gamma(0.01f)
{
    std::fill(theta.data().begin(), theta.data().end(), value_type(0));
}


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
void natural_actor_critic<ValueType, Environment, ActionFunction>::new_episode()
{
    
    // select first action
    bestAction.compute( lastState.begin(), 
                        lastState.end(), 
                        lastBestAction.begin() );
    for (size_t i = 0; i < actionSize; ++i)
    {
        //lastAction[i] += (*normal_sampler[i])();
        lastAction[i] = lastBestAction[i] + normal_sampler();
    }

    lastReward = environment.perform_action( lastAction.begin(),
                                             lastAction.end() );
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
bool natural_actor_critic<ValueType, Environment, ActionFunction>::update()
{
    bool terminal = environment.query_state( currentState.begin() );


    stateTrack.push_back(currentState);
    
    if (terminal)
    {
        return true;
    }
    // select new action
    bestAction.compute( currentState.begin(), 
                        currentState.end(), 
                        lastBestAction.begin() );
      
    // sum with variation
    for (size_t i = 0; i < actionSize; ++i)
    {
        lastAction[i] = lastBestAction[i] + normal_sampler();
    }


    lastState.swap(currentState);
    value_type reward = environment.perform_action( lastAction.begin(), 
                                             lastAction.end() );
    
    actionTrack.push_back(lastAction);
    rewards.push_back(reward);
    return false;
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
boost::mt19937 natural_actor_critic<ValueType, Environment, ActionFunction>::rng; 

} // namespace learn

#endif // __WALKER_YARD_PLAIN_DIRECT_LEARNING_HPP___