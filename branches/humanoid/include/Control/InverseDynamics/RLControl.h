#ifndef __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_RL_CONTROL_H__
#define __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_RL_CONTROL_H__

#include "../../Learning/linear_function.hpp"
#include "../../Learning/neural_network.hpp"
#include "../../Learning/radial_basis_function.hpp"
#include "../../Learning/reinforcement_learning_wrapper.hpp"
#include "../../Learning/scalar_neural_network.hpp"
#include "../../Learning/vector_function.hpp"
#include "../../Plot/DrawLinePlot.h"
#include "../../Statistics/NumActionsPlot.h"
#include "../../Statistics/ValueFunctionPlot.h"
#include "../Utility/DirectControl.h"
#include "Control.h"

namespace ctrl {
namespace id {

class RLControl :
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
    typedef environment_wrapper<environment_type>   rl_environment_type;

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

	typedef DirectControl<environment_type, vector_function_type>   direct_control_type;

public:
    RLControl(const loose_timer_ptr& timer);
    ~RLControl();

    /** Attach RL statistics to the control. */
    void addRLStatistics(rl_statistics* statistics);

    /** Remove RL statistics from the control. */
    bool removeRLStatistics(rl_statistics* statistics);

    // Override Control
	void loadConfig(const std::string& configFile);

	rl_environment_type getEnvironmentWrapper() const { return rl_environment_type(environment); }

	const vector_function_type& getActionFunction() const { return vectorFunction; }

    double getTimeInterval() const { return timeInterval; }

private:
	// Override ChainControl
    void initialize();

    // Override PhysicsControl
	void   acquire_safe();
	void   unacquire_safe();
    double pre_sync();
    void   run();


	void dumpFunctions();
	void loadFunctions();

    void initFunctions(radial_basis_function_type&    scalarFunction,
                       htangent_neural_network_type&  vectorFunction,
                       size_t                         stateSize,
                       size_t                         actionSize);

    void initFunctions(scalar_neural_network_type&    scalarFunction,
                       htangent_neural_network_type&  vectorFunction,
                       size_t                         stateSize,
                       size_t                         actionSize);

    void initFunctions(radial_basis_function_type&        scalarFunction,
                       vector_radial_basis_function_type& vectorFunction,
                       size_t                             stateSize,
                       size_t                             actionSize);

    void initFunctions(linear_radial_basis_function_type&         scalarFunction,
                       vector_linear_radial_basis_function_type&  vectorFunction,
                       size_t                                     stateSize,
                       size_t                                     actionSize);

    void initFunctions(scalar_neural_network_type&            scalarFunction,
                       vector_neural_network_function_type&   vectorFunction,
                       size_t                                 stateSize,
                       size_t                                 actionSize);
private:
	// settings
    RL_TYPE         rlType;
    std::string     rlConfig;
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

    // sync
    boost::mutex                sceneReadyMutex;
    boost::condition_variable   sceneReadyCondition;

    // control
    scalar_function_type        scalarFunction;
    vector_function_type        vectorFunction;
    reinforcement_learning_ptr  chainLearner;

    // statistics
    rl_statistics_vector        statistics;
    plot::draw_line_plot_2d_ptr valueFunctionPlotter;
    plot::draw_line_plot_2d_ptr numActionsPlotter;
    rl_statistics_ptr           valueFunctionPlot;
    rl_statistics_ptr           numActionsPlot;
};

typedef boost::intrusive_ptr<RLControl>         rl_control_ptr;
typedef boost::intrusive_ptr<const RLControl>   const_rl_control_ptr;

} // nmaespace id
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_RL_CONTROL_H__
