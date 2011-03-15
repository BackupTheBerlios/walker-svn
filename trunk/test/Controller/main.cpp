#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <slon/Engine.h>
#include <slon/Graphics/Renderer/FixedPipelineRenderer.h>
#include <slon/Graphics/Renderer/ForwardRenderer.h>
#include <slon/Graphics/Renderable/SkyBox.h>
#include <slon/Scene/Camera/LookAtCamera.h>
#include <slon/Scene/Light/DirectionalLight.h>
#include <slon/Utility/Plot/gnuplot.h>
#include "Control/Chain/PDControl.h"
#include "Control/Chain/RLControl.h"

#define BOOST_TEST_MODULE ChainControllerTest
#include <boost/test/unit_test.hpp>

using namespace slon;

ctrl::loose_timer_ptr       timer(new ctrl::LooseTimer);
ctrl::chain::pd_control_ptr pdControl(new ctrl::chain::PDControl(timer));
ctrl::chain::pd_control_ptr pdGcControl(new ctrl::chain::PDControl(timer));
ctrl::chain::rl_control_ptr rlControl(new ctrl::chain::RLControl(timer));
realm::object_ptr           chain;

Engine* InitializeEngine()
{
    Engine* engine = Engine::Instance();
	engine->init();

    // initialize logging
    log::currentLogManager().redirectOutput("", "log.txt");

    // initialize graphics
    graphics::GraphicsManager& graphicsManager = engine->getGraphicsManager();
    sgl::rectangle             viewport(0, 0, 800, 600);
    graphicsManager.setVideoMode(viewport.width, viewport.height, 32, false, false, 1);
                
    //graphics::FFPRendererDesc rendererDesc;
    //rendererDesc.useDebugRender = true;
    graphics::ForwardRendererDesc rendererDesc;
    rendererDesc.useDepthPass   = true;
    rendererDesc.makeDepthMap   = false;
    rendererDesc.useDebugRender = true;
    graphicsManager.initRenderer(rendererDesc);

    // Create camera
    scene::LookAtCamera* camera = new scene::LookAtCamera();
    camera->setViewport(viewport);
    camera->setProjectionMatrix( math::make_perspective( 0.7853982f,
                                                         static_cast<float>(viewport.width) / viewport.height,
                                                         0.1f,
                                                         500.0f ) );
    graphicsManager.addCamera(camera);

    // setup states
    camera->setPosition( math::Vector3f(0.0f, 0.85f, -2.0f) );
    camera->setDirection( math::Vector3f(0.0f, 0.0f, 1.0f) - camera->getPosition() );
    camera->setUp( math::Vector3f(0.0f, 1.0f, 0.0f) );

    // create light
    scene::DirectionalLight* light = new scene::DirectionalLight();
    light->setColor( math::Vector4f(0.8f, 0.8f, 0.8f, 1.0f) );
    light->setAmbient(0.3f);
    light->setIntensity(0.5f);
    light->setDirection( math::Vector3f(1.5f, -0.5f, 0.85f) );

    realm::currentWorld().add(light, false);

    // initialize physics
    physics::PhysicsManager& physicsManager = physics::currentPhysicsManager();
    physics::DynamicsWorld::state_desc dynamicsWorldDesc;
	{
		dynamicsWorldDesc.gravity       = math::Vector3r(0.0, physics::real(-9.8), 0.0);
		dynamicsWorldDesc.fixedTimeStep = physics::real(0.005);
	}
	physicsManager.initDynamicsWorld(dynamicsWorldDesc);
    physicsManager.getDynamicsWorld()->setMaxNumSubSteps(10000); // to make sure simulation is correct

	// timer
    timer->togglePause(true);
    timer->setTimeScale(1000.0); // simulate as fast as can
	physicsManager.setTimer( timer.get() );

    return engine;
}

void InitializeScene(const std::string& fileName)
{
    boost::property_tree::ptree properties;
    boost::property_tree::read_ini(fileName, properties);

    std::string physicsModelFile  = properties.get("PhysicsModel", "");
    std::string graphicsModelFile = properties.get("GraphicsModel", "");
    std::string pdConfigFile      = properties.get("PDConfigFile", "");
    std::string pdGcConfigFile    = properties.get("PDGCConfigFile", "");

    // setup controller
    scene::node_ptr             graphicsModel;
    physics::physics_model_ptr  physicsModel;
    if (graphicsModelFile == physicsModelFile)
    {
	    database::library_ptr library = database::loadLibrary(graphicsModelFile);

        if ( !library->getVisualScenes().empty() ) {
	        graphicsModel = library->getVisualScenes().front().second;
        }

        if ( !library->getPhysicsScenes().empty() ) {
	        physicsModel = library->getPhysicsScenes().front().second;
        }
    }
    else
    {
	    database::library_ptr library = database::loadLibrary(graphicsModelFile);

        if ( !library->getVisualScenes().empty() ) {
	        graphicsModel = library->getVisualScenes().front().second;
        }

        physicsModel = database::loadPhysicsScene(physicsModelFile);
    }
    pdControl->setTargetModel(graphicsModel);
    pdControl->setPhysicsModel(physicsModel);
    pdControl->loadConfig(pdConfigFile);

    pdGcControl->setTargetModel(graphicsModel);
    pdGcControl->setPhysicsModel(physicsModel);
    pdGcControl->loadConfig(pdGcConfigFile);

    if (chain) {
        realm::currentWorld().remove(chain.get());
    }
    chain.reset( realm::currentWorld().add(graphicsModel.get(), true, physicsModel.get()) );
}

// get distance between initial center of mass and current
physics::Vector3r GetCOM(const ctrl::PhysicsEnvironment& env)
{
    physics::Vector3r COM(0);
    physics::real     mass(0);
    for (size_t i = 0; i<env.rigidBodies.size(); ++i) 
    {
        mass += env.rigidBodies[i]->getMass();
        COM  += env.rigidBodies[i]->getMass() * math::get_translation( env.rigidBodies[i]->getTransform() );
    }
    COM /= mass;

    return COM;
}

std::string PrintTable(size_t nRows, const double* data0, const float* data1, const float* data2, const float* data3)
{
    std::ostringstream dataOs;
    for (size_t i = 0; i<nRows; ++i) {
        dataOs << data0[i] << " " << data1[i] << " " << data2[i] << " " << data3[i] << std::endl;
    }

    return dataOs.str();
}

BOOST_AUTO_TEST_CASE(chain_controller_test)
{
    const size_t      num_tests = 4;
    const std::string test_configs[num_tests] =
    {
        "Data/Config/ChainTest_2.ini",
        "Data/Config/ChainTest_3.ini",
        "Data/Config/ChainTest_4.ini",
        "Data/Config/ChainTest_5.ini"
    };
    
    Engine* engine = InitializeEngine();
    for (size_t i = 0; i<num_tests; ++i)
    {
        InitializeScene(test_configs[i]);
        slon::gnuplot plot;

        // check balance maintainance
        {
            const double        balance_time      = 5.0;
            const physics::real time_step         = physics::real(0.1);
            const physics::real balance_threshold = physics::real(0.1);
            const size_t        num_steps         = size_t(physics::real(balance_time) / time_step + physics::real(0.1));

            std::vector<double>        timeAxis(num_steps);
            std::vector<physics::real> pdBalanceAxis(num_steps);
            std::vector<physics::real> pdGcBalanceAxis(num_steps);
            std::vector<physics::real> rlBalanceAxis(num_steps, 0);

            // PD
            ctrl::TimeBarrier tb(timer, 0.0);
            timer->setTime(0.0);
            pdControl->acquire();
            engine->frame();
            {
                timer->togglePause(false);

                physics::Vector3r initialCOM = GetCOM( *pdControl->getEnvironment() );
                for (size_t i = 0; i<num_steps; ++i)
                {
                    timeAxis[i] = timer->getTime();
                    pdBalanceAxis[i] = math::length(GetCOM( *pdControl->getEnvironment() ) - initialCOM);
                    BOOST_CHECK(pdBalanceAxis[i] < balance_threshold);
                    tb.swap( ctrl::TimeBarrier(timer, timeAxis[i] + time_step) );
                    while ( timer->getTime() < timeAxis[i] + time_step ) {
                        engine->frame();
                    }
                }

                timer->togglePause(true);
            }
            pdControl->unacquire();

            // PD with gc
            tb.swap( ctrl::TimeBarrier(timer, 0.0) );
            timer->setTime(0.0);
            pdGcControl->acquire();
            engine->frame();
            {
                timer->togglePause(false);

                physics::Vector3r initialCOM = GetCOM( *pdControl->getEnvironment() );
                for (size_t i = 0; i<num_steps; ++i)
                {
                    timeAxis[i] = timer->getTime();
                    pdGcBalanceAxis[i] = math::length(GetCOM( *pdControl->getEnvironment() ) - initialCOM);
                    BOOST_CHECK(pdGcBalanceAxis[i] < balance_threshold);
                    tb.swap( ctrl::TimeBarrier(timer, timeAxis[i] + time_step) );
                    while ( timer->getTime() < timeAxis[i] + time_step ) {
                        engine->frame();
                    }
                }

                timer->togglePause(true);
            }
            pdGcControl->unacquire();

            // RL

            // plot data        
            plot.define_macro_quoted("OUTPUT", std::string("Data/Plots/BalanceMaintainance_") + boost::lexical_cast<std::string>(i + 2) + ".png");
            plot.define_macro_quoted("TITLE", "Ballance maintenance");
            plot.set_data_source(0, PrintTable(num_steps, &timeAxis[0], &pdBalanceAxis[0], &pdGcBalanceAxis[0], &rlBalanceAxis[0]));
            plot.execute("Data/Plots/ChainTest.plot");
        }
            
        // check balance restoration
        {
            const double        balance_time      = (i + 1) * 7.0;
            const physics::real time_step         = physics::real(0.1);
            const physics::real balance_threshold = physics::real(0.1);
            const size_t        num_steps         = size_t(physics::real(balance_time) / time_step + physics::real(0.1));

            std::vector<double>        timeAxis(num_steps);
            std::vector<physics::real> pdBalanceAxis(num_steps);
            std::vector<physics::real> pdGcBalanceAxis(num_steps);
            std::vector<physics::real> rlBalanceAxis(num_steps, 0);

            // PD
            ctrl::TimeBarrier tb(timer, 0.0);
            timer->setTime(0.0);
            {
                pdControl->acquire();
                physics::Vector3r initialCOM = GetCOM( *pdControl->getEnvironment() );
                
                pdControl->bendMax();
                engine->frame();
                timer->togglePause(false);

                for (size_t i = 0; i<num_steps; ++i)
                {
                    timeAxis[i] = timer->getTime();
                    pdBalanceAxis[i] = math::length(GetCOM( *pdControl->getEnvironment() ) - initialCOM);
                    tb.swap( ctrl::TimeBarrier(timer, timeAxis[i] + time_step) );
                    while ( timer->getTime() < timeAxis[i] + time_step ) {
                        engine->frame();
                    }
                }
                BOOST_CHECK(pdBalanceAxis[num_steps - 1] < balance_threshold);

                timer->togglePause(true);
            }
            pdControl->unacquire();

            // PD with GC
            tb.swap( ctrl::TimeBarrier(timer, 0.0) );
            timer->setTime(0.0);
            {
                pdGcControl->acquire();
                physics::Vector3r initialCOM = GetCOM( *pdControl->getEnvironment() );
                
                pdControl->bendMax();
                engine->frame();
                timer->togglePause(false);

                for (size_t i = 0; i<num_steps; ++i)
                {
                    timeAxis[i] = timer->getTime();
                    pdGcBalanceAxis[i] = math::length(GetCOM( *pdControl->getEnvironment() ) - initialCOM);
                    tb.swap( ctrl::TimeBarrier(timer, timeAxis[i] + time_step) );
                    while ( timer->getTime() < timeAxis[i] + time_step ) {
                        engine->frame();
                    }
                }
                BOOST_CHECK(pdGcBalanceAxis[num_steps - 1] < balance_threshold);

                timer->togglePause(true);
            }
            pdGcControl->unacquire();

            // cut plot
            const double        balance_maintain_time  = 1.5;
            const size_t        balance_maintain_steps = size_t(balance_maintain_time / time_step);
            const physics::real small_threshold        = 0.05;

            size_t steps = 0;
            for (size_t j = 0; j<num_steps; ++j)
            {
                if ( fabs(pdBalanceAxis[j]) < small_threshold
                     && fabs(pdGcBalanceAxis[j]) < small_threshold
                     && fabs(rlBalanceAxis[j]) < small_threshold )
                {
                    ++steps;
                }
                else {
                    steps = 0;
                }

                if (steps == balance_maintain_steps) 
                {
                    timeAxis.resize(j);
                    pdBalanceAxis.resize(j);
                    pdGcBalanceAxis.resize(j);
                    rlBalanceAxis.resize(j);
                    break;
                }
            }

            // plot data
            plot.define_macro_quoted("OUTPUT", std::string("Data/Plots/BalanceRestoration_") + boost::lexical_cast<std::string>(i + 2) + ".png");
            plot.define_macro_quoted("TITLE", "Ballance restoration");
            plot.set_data_source(0, PrintTable(timeAxis.size(), &timeAxis[0], &pdBalanceAxis[0], &pdGcBalanceAxis[0], &rlBalanceAxis[0]));
            plot.execute("Data/Plots/ChainTest.plot");
        }
    }
}
