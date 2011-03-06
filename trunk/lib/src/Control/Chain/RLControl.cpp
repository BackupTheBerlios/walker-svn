#include "Control/Chain/RLControl.h"
#include "Learning/config.hpp"
#include "Learning/serialization.hpp"
#include "Learning/plain_direct_learning.hpp"
#include "Learning/td_lambda_learning.hpp"
#include "Plot/DumpLinePlot.h"
#include "Utility/PhysicsDebug.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/locks.hpp>
#include <ctime>
#include <slon/Engine.h>
#include <slon/Scene/Geode.h>
#include <slon/Database/Collada/Collada.h>
#include <slon/Graphics/Common.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawCommon.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawPhysics.h>
#include <slon/Input/KeyboardHandler.h>
#include <slon/Physics/PhysicsManager.h>
#include <slon/Realm/Object/CompoundObject.h>
#include <slon/Realm/Object/EntityObject.h>
#include <slon/Realm/World.h>
#undef CreateFont

namespace {

    using namespace ctrl;
    using namespace chain;
	
    boost::mt19937 rng;

    RLControl::RL_TYPE getRLType(const std::string& str)
    {
        if (str == "DirectRL") {
            return RLControl::RL_DIRECT;
        }
        else if (str == "TDLambda") {
            return RLControl::RL_TD_LAMBDA;
        }
        
        throw std::runtime_error("Invalid RL type");
    }

} // anonymous namespace

namespace ctrl {
namespace chain {

using namespace boost::numeric::ublas;

RLControl::RLControl(const loose_timer_ptr& timer)
:	Control(timer)
,   dumpFuncsOnExit(false)
,	needRestart(false)
,   complete(false)
,	lastUpdate(0.0)
,	timeInterval(0.05)
{
}

RLControl::~RLControl()
{
    if ( acquired() ) {
        unacquire();
    }

	if (dumpFuncsOnExit) {
		dumpFunctions();
	}
}

void RLControl::dumpFunctions()
{
	learn::io::serialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::serialize(scalarFunctionDump.c_str(), scalarFunction);
}

void RLControl::loadFunctions()
{
	learn::io::deserialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::deserialize(scalarFunctionDump.c_str(), scalarFunction);
}

void RLControl::initFunctions(radial_basis_function_type&    scalarFunction,
                                 htangent_neural_network_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    boost::variate_generator< boost::mt19937&, 
                              boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(-1.0f, 1.0f));

#ifdef DEBUG
    const size_t numSamples = 2500;
#else
    const size_t numSamples = 2500;
#endif

    matrix<float> sample(numSamples, stateSize);
    std::generate(sample.begin1(), sample.end1(), generator);

    scalarFunction = RLControl::radial_basis_function_type( stateSize, 
                                                               sample.size1() * sample.size2(), 
                                                               0.34f, 
                                                               -10000, 
                                                               10000, 
                                                               sample );

    learn::make_cascade_neural_network(vectorFunction, stateSize, actionSize, 2, true);
    learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>(-0.2f, 0.2f, 0.1f) );
}

void RLControl::initFunctions(scalar_neural_network_type&    scalarFunction,
                                 htangent_neural_network_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    generic_neural_network_type network;
    learn::make_cascade_neural_network(network, stateSize, 1, 3, true);
    learn::distribute_weights( network, learn::uniform_weight_distributor<float>() );

    // fill up functions
    learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 3, make_generic_function_with_derivative( learn::linear<float>() ) );
    
    scalarFunction = scalar_neural_network_type(network);
    learn::make_cascade_neural_network(vectorFunction, stateSize, actionSize, 3, true);
    learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>() );
}

void RLControl::initFunctions(scalar_neural_network_type&    scalarFunction,
                                 vector_neural_network_function_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    generic_neural_network_type network;
    learn::make_cascade_neural_network(network, stateSize, 1, 3, true);
    learn::distribute_weights( network, learn::uniform_weight_distributor<float>() );

    // fill up functions
    learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 3, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    
    scalarFunction = scalar_neural_network_type(network);
    vectorFunction = vector_neural_network_function_type(actionSize, scalarFunction);
}

void RLControl::initFunctions(radial_basis_function_type&        scalarFunction,
                                 vector_radial_basis_function_type& vectorFunction,
                                 size_t                             stateSize,
                                 size_t                             actionSize)
{
    boost::variate_generator< boost::mt19937&, 
                              boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(-1.0f, 1.0f));

#ifdef DEBUG
    const size_t numSamples = 2500;
#else
    const size_t numSamples = 2500;
#endif

    matrix<float> sample(numSamples, stateSize);
    std::generate(sample.begin1(), sample.end1(), generator);
    /*
    std::vector< std::vector<float> > sample;
    //for (int i = 0; i<numSamples; ++i) {
    //    std::generate(sample[i].begin(), sample[i].end(), generator);
    //}
    for (float x1 = -1.4f; x1 < 1.4f; x1 += 0.2f)
    {
        for (float x2 = -1.4f; x2 < 1.4f; x2 += 0.2f)
        {
           // for (float x3 = -1.1f; x3 < 1.1f; x3 += 0.3f)
            {
                //for (float x4 = -1.1f; x4 < 1.1f; x4 += 0.3f)
                {
                    sample.push_back(std::vector<float>(stateSize));
                    sample.back()[0] = x1;
                    sample.back()[1] = x2;
                    //sample.back()[2] = x3;
                    //sample.back()[3] = x4;
                }
            }
        }
    }
    */

    scalarFunction = radial_basis_function_type(stateSize, sample.size1() * sample.size2(), 0.1f, -10000, 10000, sample);
    vectorFunction = vector_radial_basis_function_type(actionSize, learn::radial_basis_function<float>( stateSize, 
                                                                                                        sample.size1() * sample.size2(), 
                                                                                                        0.1f, 
                                                                                                        -1.0f, 
                                                                                                        1.0f, 
                                                                                                        sample ));
}

void RLControl::initFunctions(linear_radial_basis_function_type&         scalarFunction,
                                 vector_linear_radial_basis_function_type&  vectorFunction,
                                 size_t                                     stateSize,
                                 size_t                                     actionSize)
{
    scalarFunction = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 

    linear_radial_basis_function_type lin_func = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 
    vectorFunction = vector_linear_radial_basis_function_type(actionSize, lin_func);
}

void RLControl::loadConfig(const std::string& configFile)
{
    Control::loadConfig(configFile);

	boost::property_tree::ptree properties;
	boost::property_tree::read_ini(configFile, properties);
	
	dumpFuncsOnExit     = properties.get("DumpFuncsOnExit", false);
	loadFuncsOnStartup  = properties.get("LoadFuncsOnStartup", false);
	scalarFunctionDump  = properties.get("ScalarFunctionDump", "Data/Dump/scalar_function_dump.txt");
	vectorFunctionDump  = properties.get("VectorFunctionDump", "Data/Dump/vector_function_dump.txt");
	autosave            = properties.get("Autosave", false);
	autosaveTime        = properties.get("AutosaveTime", 60.0);
    episodic            = properties.get("Episodic", false);
    episodeLength       = properties.get("EpisodeLength", 0.0);
    rlType              = getRLType( properties.get("Learner", "DirectRL") );
    rlConfig            = properties.get("LearnerConfig", "");
}

void RLControl::initialize()
{
	Control::initialize();

    size_t stateSize  = environment->getStateSize();
    size_t actionSize = environment->getActionSize();

	if (loadFuncsOnStartup)
	{
		try
		{
			loadFunctions();
		}
		catch (std::exception&)
		{
			initFunctions(scalarFunction, vectorFunction, stateSize, actionSize); // overloaded function for different function types
		}
	}
	else {
		initFunctions(scalarFunction, vectorFunction, stateSize, actionSize); // overloaded function for different function types
	}

    environment->toggleEpisodic(episodic);
    environment->setControlType(PhysicsEnvironment::CONTROL_VELOCITY);
    switch (rlType)
    {
        case RL_TD_LAMBDA:
        {
            typedef learn::td_lambda_learning< float,
                                                rl_environment_type, 
                                                scalar_function_type&, 
                                                vector_function_type& >	    reinforcement_learning;
                
            reinforcement_learning rl(rl_environment_type(environment), scalarFunction, vectorFunction);
            //learn::init_from_ini(rl, rlConfig);
            chainLearner.reset( learn::make_new_rl_wrapper(rl) );
            break;
        }

        case RL_DIRECT:
        {	        
            typedef learn::plain_direct_learning< float,
                                                    rl_environment_type, 
                                                    vector_function_type& >   reinforcement_learning;

            reinforcement_learning rl(rl_environment_type(environment), vectorFunction);
            learn::init_from_ini(rl, rlConfig);
            chainLearner.reset( learn::make_new_rl_wrapper(rl) );
            break;
        }

    default:
        assert(!"can't get here");
        break;
    }

    // create statistics
    {
 /*	    std::string dateTimeStr;
        {
            time_t curTime;
            time(&curTime);
            tm* lt = localtime(&curTime);

            std::ostringstream ss;
            ss << 1900 + lt->tm_year << "_" << lt->tm_mon << "_" << lt->tm_mday
               << "_" << lt->tm_hour << "_" << lt->tm_min << "_" << lt->tm_sec;

            dateTimeStr = ss.str();
        }
	
        stats::Statistics<reinforcement_learning>* statistics = 0;
	    if (rlType == RL_TD_LAMBDA)
        {
            std::ofstream* plotOutput = new std::ofstream( ("Data/Plots/ValueFunction_" + dateTimeStr + ".txt").c_str() );
            assert( plotOutput->is_open() );

            value_function_plot::DESC desc;
            desc.valueFunctionPlotter.reset( new plot::DumpLinePlot2D( boost::shared_ptr<std::ostream>(plotOutput) ) );
            desc.restartWithNewEpisode = false;

            statistics = new value_function_plot(desc);
        }
        addRLStatistics(statistics);

        {
            std::ofstream* plotOutput = new std::ofstream( ("Data/Plots/NumActions_" + dateTimeStr + ".txt").c_str() );
            assert( plotOutput->is_open() );

            num_actions_plot::DESC desc;
            desc.plotter.reset( new plot::DumpLinePlot2D( boost::shared_ptr<std::ostream>(plotOutput) ) );

            statistics = new num_actions_plot(desc);
        }
        addRLStatistics(statistics);

        if (rlType == RL_TD_LAMBDA)
        {
            plot::DrawLinePlot2D* plotter = 0;
            {
                plot::DrawLinePlot2D::DESC desc;
                desc.axisColor  = math::Vector4f(1.0f, 1.0f, 0.0f, 1.0f);
                desc.lineColor  = math::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
                desc.autoBounds = true;
                desc.bufferSize = 100;

                valueFunctionPlotter.reset( new plot::DrawLinePlot2D(desc) );
                valueFunctionPlotter->setRegion( math::Vector2i(10, 10), math::Vector2i(200, 200) );
            }

            value_function_plot::DESC desc;
            desc.valueFunctionPlotter.reset( valueFunctionPlotter.get() );
            desc.restartWithNewEpisode = false;

            valueFunctionPlot.reset( new value_function_plot(desc) );
        }

        {
            plot::DrawLinePlot2D* plotter = 0;
            {
                plot::DrawLinePlot2D::DESC desc;
                desc.axisColor  = math::Vector4f(1.0f, 1.0f, 0.0f, 1.0f);
                desc.lineColor  = math::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
                desc.autoBounds = true;
                desc.bufferSize = 100;

                numActionsPlotter.reset( new plot::DrawLinePlot2D(desc) );
                numActionsPlotter->setRegion( math::Vector2i(220, 10), math::Vector2i(200, 200) );
            }

            num_actions_plot::DESC desc;
            desc.plotter.reset( numActionsPlotter.get() );

            numActionsPlot.reset( new num_actions_plot(desc) );
        }
*/
    }
}

void RLControl::acquire_safe()
{
	Control::acquire_safe();
	needRestart = true;
    environment->setControlType(ctrl::PhysicsEnvironment::CONTROL_VELOCITY);
}

void RLControl::unacquire_safe()
{
	Control::unacquire_safe();
	while (!complete)
	{
		double dt = pre_sync();
		if (dt > 0.0) {
			timer->setTime(timer->getTime() + dt);
		}
	}
}

double RLControl::pre_sync()
{
    if (needRestart)
    {
        acquire_safe();
        environment->reset();

        // notify learning thread
        {
            boost::lock_guard<boost::mutex> lock(sceneReadyMutex);
            needRestart = false;
        }
        sceneReadyCondition.notify_one();

        return physics::currentPhysicsManager().getDynamicsWorld()->getStateDesc().fixedTimeStep + math::EPS_6f;
    }

    switch ( environment->getState() )
    {
        case Environment::WAITING_STATE:
        {
            // fill out state
            environment->makeState();
            return 0.0;
        }

        case Environment::WAITING_ACTION:
        {
            // perform delayed action in physics thread and unlock timer
            lastUpdate = timer->getTime();
            environment->makeAction();
            return timeInterval;
        }

        case Environment::WAITING_REWARD:
        {
            // make reward at the end of the update interval
            environment->makeReward();
            return 0.0;
        }

        default:
            break; // do nothing
    }

	return 0.0;
}

void RLControl::drawDebugInfo()
{
    using namespace graphics::debug;
    using namespace physics;

    if (debugDraw)
	{
        Control::drawDebugInfo();

		size_t          stateSize = rl_environment_type(environment).state_size();
		const float*    lastState = chainLearner->get_last_state();
		float           value     = scalarFunction.compute(lastState, lastState + stateSize);
		float           force     = 0.0f;

		// compute force sum
		for (PhysicsModel::constraint_iterator iter  = physicsModel->firstConstraint();
											   iter != physicsModel->endConstraint();
											   ++iter) 
		{
			for (int i = 0; i<3; ++i)
			{
                const physics::Motor* motor = (*iter)->getMotor( physics::Motor::TYPE(physics::Motor::MOTOR_X_ROT + i) );
				if (motor) {
					force += (float)motor->getForce();
				}
			}
		}

		std::ostringstream ss;
        if (rlType == RL_TD_LAMBDA) {
		    ss << "Value function: " << value << std::endl;
        }

		ss << "Episode number: " << chainLearner->get_num_episodes() << std::endl
		   << "Action number: " << chainLearner->get_num_actions() << std::endl
		   << "Applied force: " << force << std::endl
		   << "Reward: " << environment->getReward();

		(*debugMesh) << color(1.0f, 1.0f, 0.0f, 1.0f)
					 << transform( math::make_translation(10.0f, 120.0f, 0.0f) )
					 << font( debugFont.get() )
					 << text( ss.str() );
	}
}

void RLControl::run()
{
	startTime    = Engine::Instance()->getSimulationTimer().getTime();
	lastAutosave = startTime;

    // recreate scene if needed
    complete = false;
    while ( acquired() )
    {
        // run
        chainLearner->new_episode();
        bool   terminalState = false;
        double episodeStart  = time;
        while ( !terminalState && acquired() )
        {
            terminalState = chainLearner->update();

            // restart if episode time is elapsed
            if (episodeLength > 0.0 && time - episodeStart > episodeLength) {
                terminalState = true;
            }
				
			// do autosave
			if (autosave)
			{
				if (time - lastAutosave > autosaveTime) 
				{
					lastAutosave = time;
					dumpFunctions();
				}
			}

            // gather statistics
            std::for_each( boost::make_indirect_iterator( statistics.begin() ), 
                           boost::make_indirect_iterator( statistics.end() ), 
                           boost::bind(&rl_statistics::gather, _1, boost::ref(*chainLearner)) );
        }

        if ( terminalState && acquired() )
        {
            boost::unique_lock<boost::mutex> lock(sceneReadyMutex);
            needRestart = true;
            sceneReadyCondition.wait(lock, boost::bind(&RLControl::needRestart, this) == false);
        }
    }

    complete = true;
}

void RLControl::addRLStatistics(rl_statistics* stat)
{
    statistics.push_back(stat);
}

bool RLControl::removeRLStatistics(rl_statistics* stat)
{
    rl_statistics_vector::iterator iter = std::find(statistics.begin(), statistics.end(), stat);
    if ( iter != statistics.end() )
    {
        std::swap( *iter, statistics.back() );
        statistics.pop_back();
        return true;
    }

    return false;
}

} // namespace chain
} // namespace ctrl
