#ifndef __WALKER_YARD_CONTROL_CHAIN_FLAT_RL_CONTROL_H__
#define __WALKER_YARD_CONTROL_CHAIN_FLAT_RL_CONTROL_H__

#include "Control.h"

namespace ctrl {
namespace chain {

class FlatRLControl :
    public Control
{
public:
    enum RL_FUNCTION_TYPE
    {
        FT_NEURAL_NETWORK,
        FT_RADIAL_BASIS,
        FT_LINEAR,
        FT_MIXED
    };
    public:
    // reinforcement learning
    typedef learn::neural_network< float, 
                                   learn::generic_function<float> >         generic_neural_network_type;
    typedef learn::neural_network< float, 
                                   learn::hyperbolic_tangent<float> >       htangent_neural_network_type;
    typedef learn::scalar_neural_network<generic_neural_network_type>       scalar_neural_network_type;

    typedef learn::linear_function< float,
                                    learn::generic_multi_function<float> >  linear_radial_basis_function_type;
    typedef learn::vector_function< float,
                                    scalar_neural_network_type >            vector_neural_network_function_type;
    typedef learn::vector_function< float,
                                    linear_radial_basis_function_type >     vector_linear_radial_basis_function_type;
    typedef learn::radial_basis_function<float>                             radial_basis_function_type;
    typedef learn::vector_function< float,
                                    radial_basis_function_type >            vector_radial_basis_function_type;

    //typedef radial_basis_function_type                  scalar_function_type;
    //typedef linear_radial_basis_function_type           scalar_function_type;
    typedef scalar_neural_network_type                  scalar_function_type;

    //typedef vector_radial_basis_function_type           vector_function_type;
    //typedef vector_linear_radial_basis_function_type    vector_function_type;
    typedef htangent_neural_network_type                vector_function_type;
    //typedef vector_neural_network_function_type         vector_function_type;

    enum RL_TYPE
    {
        RL_TD_LAMBDA,
        RL_DIRECT
    };

    typedef learn::abstract_reinforcement_learning<float>       reinforcement_learning;
    typedef boost::shared_ptr<reinforcement_learning>           reinforcement_learning_ptr;

    // statistics
    typedef stats::Statistics<reinforcement_learning>           rl_statistics;
    typedef boost::intrusive_ptr<rl_statistics>                 rl_statistics_ptr;
    typedef std::vector<rl_statistics_ptr>                      rl_statistics_vector;
/*	
    typedef stats::ValueFunctionPlot<reinforcement_learning>    value_function_plot;
	typedef stats::NumActionsPlot<reinforcement_learning>       num_actions_plot;
*/

public:
    FlatRLControl(const loose_timer_ptr& timer);
    ~FlatRLControl();

    // Override Control
	void loadConfig(const std::string& configFile);

    double getTimeInterval() const { return timeInterval; }

private:
	// Override ChainControl
    void drawDebugInfo();

    // Override PhysicsControl
	void   acquire_safe();
	void   unacquire_safe();
    double pre_sync();

private:

	// settings
    RL_TYPE         rlType;
    std::string     rlConfig;
    std::string     pdConfig;
	bool			dumpFuncsOnExit;
	bool			loadFuncsOnStartup;
	std::string		scalarFunctionDump;
	std::string		vectorFunctionDump;
	bool			autosave;
	double			autosaveTime;
    bool            episodic;
    double          episodeLength;

    // flow
    bool            needRestart;
    bool            complete;
	double			startTime;
	double			lastAutosave;
    double          lastUpdate;
    double          timeInterval;

    // RL control
    scalar_function_type        scalarFunction;
    vector_function_type        vectorFunction;
    reinforcement_learning_ptr  chainLearner;

    // statistics
    rl_statistics_vector        statistics;
    plot::draw_line_plot_2d_ptr valueFunctionPlotter;
    plot::draw_line_plot_2d_ptr numActionsPlotter;
    rl_statistics_ptr           valueFunctionPlot;
    rl_statistics_ptr           numActionsPlot;

    // PD control
    ublas::vector<physics::real>    mass;
    ublas::matrix<physics::real>    Kd;
    ublas::vector<physics::real>    targetVelocity;
    ublas::matrix<physics::real>    Kp;
    ublas::vector<physics::real>    targetPosition;
};

typedef boost::intrusive_ptr<FlatRLControl>         flat_rl_control_ptr;
typedef boost::intrusive_ptr<const FlatRLControl>   const_flat_rl_control_ptr;

} // namespace chain
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_FLAT_RL_CONTROL_H__
