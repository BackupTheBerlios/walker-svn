#include "Control/Chain/ChainControl.h"
#include "Learning/config.hpp"
#include "Learning/serialization.hpp"
#include "Learning/plain_direct_learning.hpp"
#include "Learning/td_lambda_learning.hpp"
#include "Plot/DumpLinePlot.h"
#include "Utility/PhysicsDebug.h"
#include <boost/iterator/indirect_iterator.hpp>
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

    static boost::mt19937 rng;

    ChainControl::RL_TYPE getRLType(const std::string& str)
    {
        if (str == "DirectRL") {
            return ChainControl::RL_DIRECT;
        }
        else if (str == "TDLambda") {
            return ChainControl::RL_TD_LAMBDA;
        }
        
        throw std::runtime_error("Invalid RL type");
    }

} // anonymous namespace

namespace ctrl {

using namespace boost::numeric::ublas;

ChainControl::ChainControl() :
    learning(true),
    simulation(false),
    pause(false),
    realtime(false),
    debugDraw(false),
    debugDrawForceScale(3.0f),
    lastUpdate(0.0),
    timeInterval(0.05)
{
}

ChainControl::~ChainControl()
{
	if (dumpFuncsOnExit) {
		dumpFunctions();
	}
}

void ChainControl::dumpFunctions()
{
	learn::io::serialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::serialize(scalarFunctionDump.c_str(), scalarFunction);
}

void ChainControl::loadFunctions()
{
	learn::io::deserialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::deserialize(scalarFunctionDump.c_str(), scalarFunction);
}

void ChainControl::initFunctions(radial_basis_function_type&    scalarFunction,
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

    scalarFunction = ChainControl::radial_basis_function_type( stateSize, 
                                                               sample.size1() * sample.size2(), 
                                                               0.34f, 
                                                               -10000, 
                                                               10000, 
                                                               sample );

    learn::make_cascade_neural_network(vectorFunction, stateSize, actionSize, 2, true);
    learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>(-0.2f, 0.2f, 0.1f) );
}

void ChainControl::initFunctions(scalar_neural_network_type&    scalarFunction,
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

void ChainControl::initFunctions(scalar_neural_network_type&    scalarFunction,
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

void ChainControl::initFunctions(radial_basis_function_type&        scalarFunction,
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

void ChainControl::initFunctions(linear_radial_basis_function_type&         scalarFunction,
                                 vector_linear_radial_basis_function_type&  vectorFunction,
                                 size_t                                     stateSize,
                                 size_t                                     actionSize)
{
    scalarFunction = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 

    linear_radial_basis_function_type lin_func = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 
    vectorFunction = vector_linear_radial_basis_function_type(actionSize, lin_func);
}

void ChainControl::loadConfig(const std::string& configFile)
{
	boost::property_tree::ptree properties;
	boost::property_tree::read_ini(configFile, properties);
	
	dumpFuncsOnExit     = properties.get("DumpFuncsOnExit", false);
	loadFuncsOnStartup  = properties.get("LoadFuncsOnStartup", false);
	scalarFunctionDump  = properties.get("ScalarFunctionDump", "Data/Dump/scalar_function_dump.txt");
	vectorFunctionDump  = properties.get("VectorFunctionDump", "Data/Dump/vector_function_dump.txt");
	autosave            = properties.get("Autosave", false);
	autosaveTime        = properties.get("AutosaveTime", 60.0);
    useHeuristic        = properties.get("UseHeuristic", false);
    episodic            = properties.get("Episodic", false);
    freeJoints          = properties.get("FreeJoints", true);
    randomStartup       = properties.get("RandomStartup", false);
    episodeLength       = properties.get("EpisodeLength", 0.0);
    maxForce            = properties.get("MaxForce", 20.0f);
    maxVelocity         = properties.get("MaxVelocity", 10.0f);
    rlType              = getRLType( properties.get("Learner", "DirectRL") );
    rlConfig            = properties.get("LearnerConfig", "");
}

void ChainControl::loadTargetModel(const std::string& modelFile)
{
    // load model
    physics::PhysicsManager& physicsManager = physics::currentPhysicsManager();

    {
        database::library_ptr  library       = database::loadLibrary(modelFile);
        scene::Node *          graphicsModel = library->getVisualScenes().front().second.get();
        physics::PhysicsModel* physicsModel  = library->getPhysicsScenes().front().second.get();

        modelObject.reset( new realm::CompoundObject(graphicsModel, true, physicsModel) );
        realm::currentWorld()->add( modelObject.get() );

        // insert physics shape debug meshes
        //PhysicsDebugVisitor visitor;
        //visitor.traverse(*graphicsModel);
    }

    // copy descs
    {
        physics::PhysicsModel* physicsModel = modelObject->getPhysicsModel();

        rigidBodiesInitialDescs.resize( std::distance( physicsModel->firstRigidBody(), physicsModel->endRigidBody() ) );
        std::transform( physicsModel->firstRigidBody(), 
                        physicsModel->endRigidBody(), 
                        rigidBodiesInitialDescs.begin(),
                        boost::bind(&physics::RigidBody::getStateDesc, _1) );

        constraintsInitialDescs.resize( std::distance( physicsModel->firstConstraint(), physicsModel->endConstraint() ) );
        std::transform( physicsModel->firstConstraint(), 
                        physicsModel->endConstraint(), 
                        constraintsInitialDescs.begin(),
                        boost::bind(&physics::Constraint::getStateDesc, _1) );

        // create Environment
        {
            ChainEnvironment::DESC              desc;
            std::vector<physics::RigidBody*>    rigidBodies;
            std::vector<physics::Constraint*>   constraints;

            std::transform( physicsModel->firstRigidBody(),
                            physicsModel->endRigidBody(),
                            std::back_inserter(rigidBodies),
                            boost::mem_fn(&physics::rigid_body_ptr::get) );
            
            std::transform( physicsModel->firstConstraint(),
                            physicsModel->endConstraint(),
                            std::back_inserter(constraints),
                            boost::mem_fn(&physics::constraint_ptr::get) );

            desc.maxForce       = maxForce;
            desc.maxVelocity    = maxVelocity;
            desc.rigidBodies    = &rigidBodies[0];
            desc.numRigidBodies = rigidBodies.size();
            desc.constraints    = &constraints[0];
            desc.numConstraints = constraints.size();
            desc.freeJoints     = freeJoints;
        
            environment.reset(new ChainEnvironment(desc));
            environment->toggleEpisodic(episodic);
            heuristicControl.reset(new HeuristicControl(environment.get()));
        }

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

        switch (rlType)
        {
            case RL_TD_LAMBDA:
            {
                typedef learn::td_lambda_learning< float,
                                                   rl_environment_type, 
                                                   scalar_function_type&, 
                                                   vector_function_type& >	    reinforcement_learning;
                
                reinforcement_learning rl(rl_environment_type(*environment), scalarFunction, vectorFunction);
                //learn::init_from_ini(rl, rlConfig);
                chainLearner.reset( learn::make_new_rl_wrapper(rl) );
                break;
            }

            case RL_DIRECT:
            {	        
                typedef learn::plain_direct_learning< float,
                                                      rl_environment_type, 
                                                      vector_function_type& >   reinforcement_learning;

                reinforcement_learning rl(rl_environment_type(*environment), vectorFunction);
                learn::init_from_ini(rl, rlConfig);
                chainLearner.reset( learn::make_new_rl_wrapper(rl) );
                break;
            }

        default:
            assert(!"can't get here");
            break;
        }

        looseTimer.reset( new LooseTimer() );
        looseTimer->togglePause(pause || realtime);
        looseTimer->toggleBounds(true);
    }

    // create statistics
    {
        std::string dateTimeStr;
        {
            time_t curTime;
            time(&curTime);
            tm* lt = localtime(&curTime);

            std::ostringstream ss;
            ss << 1900 + lt->tm_year << "_" << lt->tm_mon << "_" << lt->tm_mday
               << "_" << lt->tm_hour << "_" << lt->tm_min << "_" << lt->tm_sec;

            dateTimeStr = ss.str();
        }
/*		
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

    // prepare debug draw objects
    {
        debugMesh.reset(new graphics::DebugMesh);

        // create font
        debugFont.reset( graphics::currentDevice()->CreateFont() );
        {
            sgl::Image* image = graphics::currentDevice()->CreateImage();
            image->LoadFromFile("Data/Fonts/font.png");
            debugFont->SetTexture( image->CreateTexture2D() );
        }

        debugObject.reset( new realm::EntityObject(*debugMesh, false) );
        realm::currentWorld()->add( debugObject.get() );
    }

    toggleDebugDraw(true);
} // load

void ChainControl::beginSimulation(bool fromStart)
{
    physics::PhysicsManager& physicsManager = physics::currentPhysicsManager();
    physicsManager.setTimer( looseTimer.get() );

    // start learning
    simulation = true;
    boost::thread thread( boost::bind(&ChainControl::run, this) );
    updateThread.swap(thread);

    // connect updaters
    syncConnection = physicsManager.connectPreFrameCallback( boost::bind(&ChainControl::syncWithPhysics, this) );
    drawConnection = physicsManager.connectPostFrameCallback( boost::bind(&ChainControl::drawDebugInfo, this) );
}

void ChainControl::toggleHeuristic(bool toggle)
{
    useHeuristic = toggle;

    // restart
    if (simulation)
    {
        endSimulation();
        beginSimulation();
    }
}

void ChainControl::toggleLearning(bool toggle)
{
    if (simulation) 
    {
        endSimulation();
        learning = toggle;
        beginSimulation();
    }
    else {
        learning = toggle;
    }
}

void ChainControl::togglePause(bool toggle)
{
    pause = toggle;
    looseTimer->togglePause(pause || realtime);
}

void ChainControl::toggleRealtime(bool toggle)
{
    // restart
    if (simulation) 
    {
        endSimulation();
        realtime = toggle;
        if (realtime) {
            learning = true;
        }
        looseTimer->togglePause(pause || realtime);
        beginSimulation();
    }
    else 
    {
        realtime = toggle;
        if (realtime) {
            learning = true;
        }
        looseTimer->togglePause(pause || realtime);
    }
}

void ChainControl::toggleDebugDraw(bool toggle)
{
    if (!debugDraw && toggle)
    {
        debugDraw = true;
	#ifdef CHAIN_CONTROL_USE_TD_LAMBDA
        //addRLStatistics( valueFunctionPlot.get() );
        //valueFunctionPlotter->hide(false);
	#endif
        //addRLStatistics( numActionsPlot.get() );
        //numActionsPlotter->hide(false);
    }
    else if (debugDraw && !toggle)
    {
        debugDraw = false;
	#ifdef CHAIN_CONTROL_USE_TD_LAMBDA
        //removeRLStatistics( valueFunctionPlot.get() );
        //valueFunctionPlotter->hide(true);
	#endif
        //removeRLStatistics( numActionsPlot.get() );
        //numActionsPlotter->hide(true);
    }
}

void ChainControl::toggleDraw(bool toggle)
{
    realm::World* world = Engine::Instance()->getWorld();
    if (!toggle)
    {
        if ( debugObject->getWorld() ) {
            world->remove( debugObject.get() );
        }
        if ( modelObject->getWorld() ) {
            world->remove( modelObject.get() );
        }
    }
    else 
    {
        if ( !debugObject->getWorld() ) {
            world->add( debugObject.get() );
        }
        if ( !modelObject->getWorld() ) {
            world->add( modelObject.get() );
        }
    }
}

bool ChainControl::isDrawing() const
{
    return debugObject->getWorld() || modelObject->getWorld();
}

void ChainControl::setTimeScale(float timeScale)
{
    if (timeScale < 0.0f) {
        timeScale = 0.0f;
    }

    looseTimer->setTimeScale(timeScale);
}

void ChainControl::endSimulation()
{
    simulation = false;
    looseTimer->setTime( looseTimer->getMaxTime() ); // simulation must complete

	// wait episode ends
	while ( !episodeMutex.try_lock() ) {
        Engine::Instance()->frame();
	}
    episodeMutex.unlock();

	updateThread.join();
    syncConnection.disconnect();
    drawConnection.disconnect();
}

void ChainControl::prepareScene()
{
    using namespace physics;

    rigid_body_desc_vector rigidBodiesDescs(rigidBodiesInitialDescs);
    constraint_desc_vector constraintsDescs(constraintsInitialDescs);

    // reset rigid bodies
    {
        rigid_body_desc_vector::iterator descIter = rigidBodiesDescs.begin();
        for (PhysicsModel::rigid_body_iterator iter  = modelObject->getPhysicsModel()->firstRigidBody();
                                               iter != modelObject->getPhysicsModel()->endRigidBody(); 
                                               ++iter, ++descIter)
        {
            (*iter)->reset(*descIter);
        }
    }

    // reset constraints
    {
        constraint_desc_vector::iterator descIter = constraintsDescs.begin();
        for (PhysicsModel::constraint_iterator iter  = modelObject->getPhysicsModel()->firstConstraint();
                                               iter != modelObject->getPhysicsModel()->endConstraint(); 
                                               ++iter, ++descIter)
        {
            (*iter)->reset(*descIter);
        }
    }

    if (randomStartup)
    {

        boost::variate_generator< boost::mt19937&, 
                                  boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(0.0f, 1.0f));

        // std axes
        math::Vector3f axes[3] = 
        {
            math::Vector3f(1.0f, 0.0f, 0.0f),
            math::Vector3f(0.0f, 1.0f, 0.0f),
            math::Vector3f(0.0f, 0.0f, 1.0f),
        };

        // rotate rigid bodies around constraints
        for (size_t i = 0; i<constraintsDescs.size(); ++i)
        {
            physics::RigidBody* rbody0 = constraintsInitialDescs[i].rigidBodies[0];
            physics::RigidBody* rbody1 = constraintsInitialDescs[i].rigidBodies[1];

            // find desc by name
            physics::RigidBody::state_desc* desc0 = 0;
            physics::RigidBody::state_desc* desc1 = 0;
            for (size_t j = 0; j<rigidBodiesDescs.size(); ++j) 
            {
                if ( rigidBodiesDescs[j].name == rbody0->getName() ) {
                    desc0 = &rigidBodiesDescs[j];
                }
                else if ( rigidBodiesDescs[j].name == rbody1->getName() ) {
                    desc1 = &rigidBodiesDescs[j];
                }
            }
            assert(desc0 && desc1);

            // get transform of the constraint from bottom cone
            math::Matrix4f transform = desc0->initialTransform * constraintsDescs[i].frames[0]; 
            for (int j = 0; j<3; ++j)
            {
                float angle = constraintsDescs[i].angularLimits[0][j] + generator() * (constraintsDescs[i].angularLimits[1][j] - constraintsDescs[i].angularLimits[0][j]);
                transform  *= math::make_rotation(angle, axes[j]);
            }

            desc1->initialTransform = transform * math::invert(constraintsDescs[i].frames[1]);
        }

        // reset rigid bodies
        {
            rigid_body_desc_vector::iterator descIter = rigidBodiesDescs.begin();

            for (PhysicsModel::rigid_body_iterator iter  = modelObject->getPhysicsModel()->firstRigidBody();
                                                   iter != modelObject->getPhysicsModel()->endRigidBody(); 
                                                   ++iter, ++descIter)
            {
                (*iter)->reset(*descIter);
            }
        }
    }
}

void ChainControl::syncWithPhysics()
{
    if (preparingScene)
    {
        prepareScene();
        environment->reset();

        // notify learning thread
        {
            boost::lock_guard<boost::mutex> sceneReadyLock(sceneReadyMutex);
            preparingScene = false;
        }
        sceneReadyCondition.notify_one();

        // to make sure one simulation step is executed
        lastUpdate = looseTimer->getTime() + physics::currentPhysicsManager().getDynamicsWorld()->getStateDesc().fixedTimeStep + math::EPS_6f;
        looseTimer->setTimeBounds(0.0, lastUpdate);
        looseTimer->setTime(lastUpdate);
        
        return;
    }

    switch ( environment->getState() )
    {
        case ChainEnvironment::WAITING_STATE:
        {
            // filluot state
            environment->makeState();
            break;
        }

        case ChainEnvironment::WAITING_ACTION:
        {
            // perform delayed action in physics thread and unlock timer
            lastUpdate = looseTimer->getTime();
            environment->makeAction();
            looseTimer->setTimeBounds(0.0, lastUpdate + timeInterval);
            if (realtime) {
                looseTimer->setTime(lastUpdate + timeInterval);
            }
            break;
        }

        case ChainEnvironment::WAITING_REWARD:
        {
            // make reward at the end of the update interval
            if ( looseTimer->atBounds() ) {
                 environment->makeReward();
            }
            break;
        }

        default:
            break; // do nothing
    }
}

void ChainControl::drawDebugInfo()
{
	if ( debugDraw && debugObject->getWorld() ) 
	{
		using namespace physics;
		using namespace graphics::debug;

		const math::Vector3f axes[] =
		{
			math::Vector3f( 0.0f, 1.0f, 0.0f),
			math::Vector3f(-1.0f, 0.0f, 0.0f)
		};

		debugMesh->clear();

		// draw rigid bodies
		for (PhysicsModel::rigid_body_iterator iter  = modelObject->getPhysicsModel()->firstRigidBody();
											   iter != modelObject->getPhysicsModel()->endRigidBody(); 
											   ++iter)
		{
			if ( RigidBody::AS_SLEEPING == (*iter)->getActivationState() 
				 || RigidBody::AS_DISABLE_SIMULATION == (*iter)->getActivationState() ) 
			{
				(*debugMesh) << color(0.5f, 0.5f, 0.5f, 1.0f);
			}
			else {
				(*debugMesh) << color(1.0f, 0.0f, 0.0f, 1.0f);
			}

			(*debugMesh) << transform( (*iter)->getTransform() )
						 << static_cast<const physics::ConeShape&>( *(*iter)->getCollisionShape() );
		}

		// draw forces
		(*debugMesh) << color(1.0f, 0.0f, 0.0f, 1.0f);
		for (PhysicsModel::constraint_iterator iter  = modelObject->getPhysicsModel()->firstConstraint();
											   iter != modelObject->getPhysicsModel()->endConstraint(); 
											   ++iter)
		{
			const physics::Constraint& constraint = (**iter);

			math::Vector3f force(0.0f, 0.0f, 0.0f);
			for (int i = 0; i<2; ++i)
			{
				const physics::RotationalMotor* motor = constraint.getRotationalMotor(i);
				if ( motor && motor->isActivated() ) {
					force += axes[i] * motor->getTargetVelocity();
				}
			}

			force *= debugDrawForceScale;
			force /= environment->getMaxVelocity();

			math::Vector3f origin0 = math::get_translation( constraint.getRigidBodyA()->getTransform() * constraint.getStateDesc().frames[0]  );
			math::Vector3f origin1 = math::get_translation( constraint.getRigidBodyA()->getTransform() );
			math::Vector3f origin2 = math::get_translation( constraint.getRigidBodyB()->getTransform() );
			math::Vector3f tip     = math::xyz( constraint.getRigidBodyA()->getTransform() * constraint.getStateDesc().frames[0] * math::make_vec(force, 1.0f) );
			(*debugMesh) << transform( math::make_identity<float, 4>() )
						 << line(origin0, tip)
						 << line(origin1, tip) 
						 << line(origin2, tip)
						 << line(origin0, origin1)
						 << line(origin0, origin2);
		}

		{
			size_t          stateSize = rl_environment_type(*environment).state_size();
			const float*    lastState = chainLearner->get_last_state();
			float           value     = scalarFunction.compute(lastState, lastState + stateSize);
			float           force     = 0.0f;

			// compute force sum
			for (PhysicsModel::constraint_iterator iter  = modelObject->getPhysicsModel()->firstConstraint();
												   iter != modelObject->getPhysicsModel()->endConstraint();
												   ++iter) 
			{
				for (int i = 0; i<3; ++i)
				{
					if ( physics::RotationalMotor* motor = (*iter)->getRotationalMotor(i) ) {
						force += fabs( motor->getMaxForce() );
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
						 << transform( math::make_translation(10.0f, 100.0f, 0.0f) )
						 << font( debugFont.get() )
						 << text( ss.str() );
		}
	}
}

void ChainControl::run()
{
	startTime    = Engine::Instance()->getSimulationTimer().getTime();
	lastAutosave = startTime;

    // recreate scene if needed
    while (simulation)
    {
        // prepare scene
        episodeMutex.lock();
        {
            if (simulation)
            {
                boost::unique_lock<boost::mutex> sceneReadyLock(sceneReadyMutex);
                preparingScene = true;
                sceneReadyCondition.wait(sceneReadyLock, boost::bind(&ChainControl::preparingScene, this) == false);
            }
            
            // start episode
            chainLearner->new_episode();
        }
        episodeMutex.unlock();

        // run
        bool terminalState = false;
        if (useHeuristic)
        {
            while (!terminalState && simulation)
            {
                boost::lock_guard<boost::mutex> lock(episodeMutex);
                {
                    rl_environment_type env(*environment);
                    std::vector<float> state( env.state_size() );
                    std::vector<float> action( env.action_size() );

                    terminalState = env.query_state( state.begin() );
                    heuristicControl->queryAction( action.begin() );
                    env.perform_action( action.begin(), action.end() );
                }
            }
        }
        else if (learning)
        {
            double episodeStart = looseTimer->getTime();
            while (!terminalState && simulation)
            {
                boost::lock_guard<boost::mutex> lock(episodeMutex);
                {
                    if (!pause) {
                        terminalState = chainLearner->update();
		            }

                    // restart if episode time is elapsed
                    if (episodeLength > 0.0 && looseTimer->getTime() - episodeStart > episodeLength) {
                        terminalState = true;
                    }
    				
				    // do autosave
				    if (autosave)
				    {
					    double time = Engine::Instance()->getSimulationTimer().getTime();
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
            }
        }
        else
        {
            while (simulation)
            {
                boost::lock_guard<boost::mutex> lock(episodeMutex);
                {
                    rl_environment_type env(*environment);
                    std::vector<float> state( env.state_size() );
                    std::vector<float> action( env.action_size() );

                    terminalState = env.query_state( state.begin() );
                    vectorFunction.compute( state.begin(), state.end(), action.begin() );
                    env.perform_action( action.begin(), action.end() );
                }
            }
        }
    }
}

void ChainControl::addRLStatistics(rl_statistics* stat)
{
    statistics.push_back(stat);
}

bool ChainControl::removeRLStatistics(rl_statistics* stat)
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

} // namespace ctrl
