#ifndef __WALKER_YARD_STATISTICS_VALUE_FUNCTION_PLOT_H__
#define __WALKER_YARD_STATISTICS_VALUE_FUNCTION_PLOT_H__

#include "../Plot/LinePlot.h"
#include "Statistics.h"

namespace stats {

/** Plots RL data */
template<class ReinforcementLearning>
class ValueFunctionPlot :
    public Statistics<ReinforcementLearning>
{
public:
    typedef ReinforcementLearning                           reinforcement_learning;
    typedef typename reinforcement_learning::value_type     value_type;

    struct DESC
    {
        plot::line_plot_2d_ptr  valueFunctionPlotter;
        bool                    restartWithNewEpisode;

        explicit DESC(plot::LinePlot2D* valueFunctionPlotter_  = 0,
                      bool              restartWithNewEpisode_ = false) :
            valueFunctionPlotter(valueFunctionPlotter_),
            restartWithNewEpisode(restartWithNewEpisode_)
        {}
    };

public:
    ValueFunctionPlot(const DESC& desc_) :
        desc(desc_),
        lastEpisode(0),
        lastAction(0),
        numActions(0)
    {
        assert(desc.valueFunctionPlotter);
    }

    void gather(const reinforcement_learning& rl)
    {
        size_t  episode = rl.get_num_episodes();
        size_t  action  = rl.get_num_actions();
        
        if (episode == lastEpisode) {
            lastAction = action;
        }
        else 
        {
            if (desc.restartWithNewEpisode) 
            {
                desc.valueFunctionPlotter->plotEnd();
                desc.valueFunctionPlotter->plotBegin();
            }
            else 
            {
                numActions += lastAction;
            }
        }
        lastEpisode = episode;

        if (!desc.restartWithNewEpisode) {
            action += numActions;
        }

        if ( !desc.valueFunctionPlotter->isPlotting() ) {
            desc.valueFunctionPlotter->plotBegin();
        }

        size_t              stateSize = rl.get_environment().state_size();
        const value_type*   lastState = rl.get_last_state();
        value_type          value     = rl.get_value_function().compute(lastState, lastState + stateSize);
        value_type          argument  = value_type(action);
        desc.valueFunctionPlotter->plotPoint( math::Vector2f(argument, value) );
    }

private:
    DESC    desc;
    size_t  lastEpisode;
    size_t  lastAction;
    size_t  numActions;
};
    
} // namespace stat

#endif // __WALKER_YARD_STATISTICS_VALUE_FUNCTION_PLOT_H__
