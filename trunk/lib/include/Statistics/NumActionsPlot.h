#ifndef __WALKER_YARD_STATISTICS_NUM_ACTIONS_PLOT_H__
#define __WALKER_YARD_STATISTICS_NUM_ACTIONS_PLOT_H__

#include "../Plot/LinePlot.h"
#include "Statistics.h"

namespace stats {

/** Plots RL data */
template<class ReinforcementLearning>
class NumActionsPlot :
    public Statistics<ReinforcementLearning>
{
public:
    typedef ReinforcementLearning                           reinforcement_learning;
    typedef typename reinforcement_learning::value_type     value_type;

    struct DESC
    {
        plot::line_plot_2d_ptr  plotter;

        explicit DESC(plot::LinePlot2D* plotter_ = 0) :
            plotter(plotter_)
        {}
    };

public:
    NumActionsPlot(const DESC& desc_) :
        desc(desc_),
        lastEpisode(0),
        lastAction(0)
    {
        assert(desc.plotter);
    }

    ~NumActionsPlot()
    {
        assert(desc.plotter);
        desc.plotter->plotEnd();
    }

    void gather(const reinforcement_learning& rl)
    {
        assert(desc.plotter);

        size_t  episode = rl.get_num_episodes();
        size_t  action  = rl.get_num_actions();
        
        if ( !desc.plotter->isPlotting() ) {
            desc.plotter->plotBegin();
        }

        if (episode == lastEpisode) {
            lastAction = action;
        }
        else {
            desc.plotter->plotPoint( math::Vector2f( float(lastEpisode), float(lastAction) ) );
        }
        lastEpisode = episode;
    }

private:
    DESC    desc;
    size_t  lastEpisode;
    size_t  lastAction;
};
    
} // namespace stat

#endif // __WALKER_YARD_STATISTICS_NUM_ACTIONS_PLOT_H__
