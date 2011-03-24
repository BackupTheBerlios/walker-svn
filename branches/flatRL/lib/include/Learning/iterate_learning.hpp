#ifndef __WALKER_YARD_PLAIN_ITERATE_LEARNING_HPP___
#define __WALKER_YARD_PLAIN_ITERATE_LEARNING_HPP___

#include <boost/random.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace learn {

namespace ublas = boost::numeric::ublas;

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
class iterate_learning
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
    iterate_learning( environment_type        _environment,
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
    value_type                  sigma, beta, gamma;
    int                         iter;
    int                         numEpisodes;
};


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
iterate_learning<ValueType, Environment, ActionFunction>::iterate_learning( Environment       _environment,
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
,   beta(0.95f)
,   gamma(0.00002f)
,   iter(1)
,   numEpisodes(0)
{
    std::fill(theta.data().begin(), theta.data().end(), value_type(0));
}


template < typename ValueType,
           typename Environment,
           typename ActionFunction >
void iterate_learning<ValueType, Environment, ActionFunction>::new_episode()
{
    Z = matrix_type( bestAction.num_components(),
                     bestAction.num_params() / bestAction.num_components() );
    std::fill(Z.data().begin(), Z.data().end(), value_type(0));
    environment.query_state( lastState.begin() );
    
    
    // select first action
    bestAction.compute( lastState.begin(), 
                        lastState.end(), 
                        lastBestAction.begin() );
    for (size_t i = 0; i < actionSize; ++i)
    {
        //lastAction[i] += (*normal_sampler[i])();
        lastAction[i] = lastBestAction[i] + normal_sampler();
    }
    numEpisodes += 1;
    if (!(numEpisodes %5 )) 
        iter = !iter;
    lastReward = environment.perform_action( lastAction.begin(),
                                             lastAction.end() );
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
bool iterate_learning<ValueType, Environment, ActionFunction>::update()
{
    bool terminal = environment.query_state( currentState.begin() );
    matrix_type delta;
    // ... compute delta
    matrix_type gradient(bestAction.num_components(), bestAction.num_params()/bestAction.num_components());
    bestAction.get_gradient(lastState.begin(), lastState.end(), gradient.data().begin()); // lastState?
    delta = (1/(sigma*sigma))*mult(gradient, lastAction - lastBestAction);
    Z = beta*Z + delta;
    for (int i = iter*bestAction.num_components()/2; i < (iter+1)*bestAction.num_components()/2; ++i)
    {
        for (int j = 0; j < bestAction.num_params()/bestAction.num_components(); ++j)
        {
            Z(i, j) = 0;
        }
    }
    bestAction.get_params(theta.data().begin());
    theta += gamma*lastReward*Z;
    bestAction.set_params(theta.data().begin(), theta.data().end());

    if (terminal)
    {
        return true;
    }
    // select new action
    bestAction.compute( currentState.begin(), 
                        currentState.end(), 
                        lastBestAction.begin() );
      
    // sum with variation
    //for (int i = (1-iter)*bestAction.num_components()/2; i < (2-iter)*bestAction.num_components()/2; ++i)
    for (int i = 0; i < lastAction.size(); ++i)
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
    lastState.swap(currentState);
    lastReward = environment.perform_action( lastAction.begin(), 
                                             lastAction.end() );
    gamma *= 0.999;
    if (gamma < 0.000001) gamma = 0.000001;
    return false;
}

template < typename ValueType,
           typename Environment,
           typename ActionFunction >
boost::mt19937 iterate_learning<ValueType, Environment, ActionFunction>::rng; 

} // namespace learn

#endif // __WALKER_YARD_PLAIN_ITERATE_LEARNING_HPP___