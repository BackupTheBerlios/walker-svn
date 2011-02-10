#ifndef __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__
#define __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__

#include "../../Learning/linear_function.hpp"
#include "../../Learning/neural_network.hpp"
#include "../../Learning/radial_basis_function.hpp"
#include "../../Learning/reinforcement_learning_wrapper.hpp"
#include "../../Learning/scalar_neural_network.hpp"
#include "../../Learning/vector_function.hpp"
#include "../../Plot/DrawLinePlot.h"
#include "../../Statistics/NumActionsPlot.h"
#include "../../Statistics/ValueFunctionPlot.h"
#include "../Control.h"
#include "../Utility/LooseTimer.h"
#include "ChainEnvironment.h"
#include "HeuristicControl.h"
#include <boost/property_tree/ptree.hpp>
#include <slon/Physics/PhysicsManager.h>
#include <slon/Realm/Object/CompoundObject.h>
#include <slon/Realm/Object/EntityObject.h>

namespace ctrl {

class ChainControl :
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
    typedef ChainEnvironment                environment_type;
    typedef rl_wrapper<ChainEnvironment>    rl_environment_type;

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
    typedef linear_radial_basis_function_type           scalar_function_type;
    //typedef scalar_neural_network_type                  scalar_function_type;

    //typedef vector_radial_basis_function_type           vector_function_type;
    typedef vector_linear_radial_basis_function_type    vector_function_type;
    //typedef htangent_neural_network_type                vector_function_type;
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
    // physics
    typedef sgl::vector<physics::RigidBody::state_desc,
                        sgl::aligned_allocator<physics::RigidBody::state_desc> >    rigid_body_desc_vector;

    typedef sgl::vector<physics::Constraint::state_desc,
                        sgl::aligned_allocator<physics::Constraint::state_desc> >   constraint_desc_vector;

public:
    ChainControl();
    ~ChainControl();

    /** Attach RL statistics to the control. */
    void addRLStatistics(rl_statistics* statistics);

    /** Remove RL statistics from the control. */
    bool removeRLStatistics(rl_statistics* statistics);

    // Override central
	void loadConfig(const std::string& configFile);
    void loadTargetModel(const std::string& modelFile);
    
    void toggleHeuristic(bool toggle);
    void toggleLearning(bool toggle);
    void togglePause(bool toggle);
    void toggleRealtime(bool toggle);
    void toggleDebugDraw(bool toggle);
    void toggleDraw(bool toggle);
    void setTimeScale(float timeScale);

    bool  isHeuristicUsed() const   { return useHeuristic; }
    bool  isLearning() const        { return learning; }
    bool  isPaused() const          { return pause; }
    bool  isRealtime() const        { return realtime; }
    bool  isDebugDrawing() const    { return debugDraw; }
    bool  isDrawing() const;
    float getSimulationTime() const { return (float)looseTimer->getTime(); }
    float getTimeScale() const      { return (float)looseTimer->getTimeScale(); }

    void beginSimulation(bool fromStart = true);
    void endSimulation();
    environment_type* getEnvironment() { return environment.get(); }

private:
    void prepareScene();
    void syncWithPhysics();
    void run();
    void drawDebugInfo();
	
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
    void ChainControl::initFunctions(scalar_neural_network_type&    scalarFunction,
                       vector_neural_network_function_type&  vectorFunction,
                       size_t                         stateSize,
                       size_t                         actionSize);
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
    bool            freeJoints;
    bool            randomStartup;
    double          episodeLength;
    float           maxForce;
    float           maxVelocity;
	
    bool            debugDraw;
    float           debugDrawForceScale;

    // flow
    bool            learning;
    bool            useHeuristic;
    bool            simulation;
    bool            pause;
    bool            realtime;
    bool            preparingScene;
	double			startTime;
	double			lastAutosave;
    double          lastUpdate;
    double          timeInterval;

    boost::thread                               updateThread;
    boost::intrusive_ptr<LooseTimer>            looseTimer;
    physics::PhysicsManager::connection_type    syncConnection;
    physics::PhysicsManager::connection_type    drawConnection;
    boost::mutex                                episodeMutex;
    boost::mutex                                sceneReadyMutex;
    boost::condition_variable                   sceneReadyCondition;

    // scene
    realm::compound_object_ptr                  modelObject;
    realm::entity_object_ptr                    debugObject;
    graphics::debug_mesh_ptr                    debugMesh;
    sgl::ref_ptr<sgl::Font>                     debugFont;
    rigid_body_desc_vector                      rigidBodiesInitialDescs;
    constraint_desc_vector                      constraintsInitialDescs;

    // control
    chain_environment_ptr                       environment;
    const_heuristic_control_ptr                 heuristicControl;
    scalar_function_type                        scalarFunction;
    vector_function_type                        vectorFunction;
    reinforcement_learning_ptr                  chainLearner;

    // statistics
    rl_statistics_vector                        statistics;
    plot::draw_line_plot_2d_ptr                 valueFunctionPlotter;
    plot::draw_line_plot_2d_ptr                 numActionsPlotter;
    rl_statistics_ptr                           valueFunctionPlot;
    rl_statistics_ptr                           numActionsPlot;
};

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__
