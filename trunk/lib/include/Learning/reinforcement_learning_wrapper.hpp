#ifndef __WALKER_YARD_REINFORCEMENT_LEARNING_WRAPPER_HPP___
#define __WALKER_YARD_REINFORCEMENT_LEARNING_WRAPPER_HPP___

namespace learn {

template <typename ValueType>
class abstract_reinforcement_learning
{
public:
    typedef ValueType         value_type;

public:
    virtual void new_episode() = 0;
    virtual bool update() = 0;

    virtual size_t            get_num_episodes() const = 0;
    virtual size_t            get_num_actions() const = 0;
    virtual const value_type* get_last_state() const = 0;
    virtual const value_type* get_last_action() const = 0;
    virtual value_type        get_last_reward() const = 0;

    virtual ~abstract_reinforcement_learning() {}
};

template <typename ReinforcementLearning>
class reinforcement_learning_wrapper :
    public abstract_reinforcement_learning<typename ReinforcementLearning::value_type>
{
public:
    typedef ReinforcementLearning                                       reinforcement_learning_type;
    typedef typename reinforcement_learning_type::value_type            value_type;
    typedef typename reinforcement_learning_type::environment_type      environment_type;
    typedef typename reinforcement_learning_type::action_function_type  action_function_type;

public:
    reinforcement_learning_wrapper(reinforcement_learning_type rl_) :
        rl(rl_)
    {}

    void new_episode()  { rl.new_episode(); }
    bool update()       { return rl.update(); }

    size_t                  get_num_episodes() const    { return rl.get_num_episodes(); }
    size_t                  get_num_actions() const     { return rl.get_num_actions(); }
    const value_type*       get_last_state() const      { return rl.get_last_state(); }
    const value_type*       get_last_action() const     { return rl.get_last_action(); }
    value_type              get_last_reward() const     { return rl.get_last_reward(); }

    environment_type        get_environment() const          { return rl.get_environment(); }
    action_function_type    get_best_action_function() const { return rl.get_best_action_function(); }

private:
    reinforcement_learning_type rl;
};

template <typename RL>
inline reinforcement_learning_wrapper<RL> make_rl_wrapper(RL rl)
{
    return reinforcement_learning_wrapper<RL>(rl);
}

template <typename RL>
inline reinforcement_learning_wrapper<RL>* make_new_rl_wrapper(RL rl)
{
    return new reinforcement_learning_wrapper<RL>(rl);
}

} // namespace learn

#endif // __WALKER_YARD_REINFORCEMENT_LEARNING_WRAPPER_HPP___