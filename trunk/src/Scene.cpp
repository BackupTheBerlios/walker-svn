#include "Scene.h"
#include "Utility/PhysicsDebug.h"
#include <boost/bind.hpp>
#include <iomanip>
#include <slon/Engine.h>
#include <slon/Database/Collada/Collada.h>
#include <slon/Graphics/Common.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawCommon.h>
#include <slon/Graphics/Renderable/Mesh.h>
#include <slon/Graphics/Renderable/SkyBox.h>
#include <slon/Graphics/Renderer/FixedPipelineRenderer.h>
#include <slon/Graphics/Renderer/ForwardRenderer.h>
#include <slon/Physics/PhysicsModel.h>
#include <slon/Realm/Location/DbvtLocation.h>
#include <slon/Realm/Object/EntityObject.h>
#include <slon/Realm/World/ScalableWorld.h>
#include <slon/Scene/Light/DirectionalLight.h>
#include <slon/Scene/Group.h>
#include <sstream>
#undef CreateFont

namespace {

    void printSceneGraph(const scene::Node& node, const std::string& indent = "")
    {
        using namespace scene;

        std::cout << indent << node.getName() << std::endl;
        if ( const Group* group = dynamic_cast<const Group*>(&node) )
        {
            for ( Group::const_node_iterator iter = group->firstChild();
                                             iter != group->endChild();
                                             ++iter )
            {
                printSceneGraph(**iter, indent + "  ");
            }
        }
    }

    scene::LookAtCamera* createMainCamera(const sgl::rectangle& viewport)
    {
        // create camera
        scene::LookAtCamera* camera = new scene::LookAtCamera();
        camera->setViewport(viewport);
        camera->setProjectionMatrix( math::make_perspective( 0.7853982f,
                                                             static_cast<float>(viewport.width) / viewport.height,
                                                             0.1f,
                                                             500.0f ) );
        return camera;
    }

} // anonymous namespace

bool userAction = false;
math::Vector3f userTorque;

Scene::Scene(const DESC& desc)
:	fps(0.0f)
,   time(0.0f)
,   dt(0.0f)
,   drawHelp(true)
,   needToTakeScreenshot(false)
{
    Engine* engine = Engine::Instance();

    // setup graphics
    {
        using namespace slon::scene;

        // setup loggers
        {
            log::currentLogManager().redirectOutput("database", "database_log.txt");
            log::currentLogManager().redirectOutput("graphics", "graphics_log.txt");
        }

        graphics::GraphicsManager& graphicsManager = engine->getGraphicsManager();
        graphicsManager.setVideoMode(desc.windowWidth, desc.windowHeight, 32, desc.fullscreen, false, desc.multisample);
		screenSize = math::Vector2ui(desc.windowWidth, desc.windowHeight);
#ifdef WIN32
        FreeConsole();
#endif
        // create renderer
        graphics::Renderer* renderer;
        if (!desc.ffp)
        {
            try
            {
                graphics::ForwardRendererDesc desc;
                desc.useDepthPass   = true;
                desc.makeDepthMap   = false;
                desc.useDebugRender = true;
                renderer = graphicsManager.initRenderer(desc);
            }
            catch(slon_error&)
            {
                graphics::FFPRendererDesc desc;
                desc.useDebugRender = true;
                renderer = graphicsManager.initRenderer(desc);
            }
        }
        else
        {
            graphics::FFPRendererDesc desc;
            desc.useDebugRender = true;
            renderer = graphicsManager.initRenderer(desc);
        }

        // create world
        world.reset(new realm::ScalableWorld);
        engine->setWorld( world.get() );

        // Create skybox
        graphics::SkyBox* skyBox = new graphics::SkyBox();
        {
            const std::string SKY_BOX_MAPS[6] =
            {
                "Data/SkyBox/thunder_west.jpg",
                "Data/SkyBox/thunder_east.jpg",
                "Data/SkyBox/thunder_up.jpg",
                "Data/SkyBox/thunder_down.jpg",
                "Data/SkyBox/thunder_south.jpg",
                "Data/SkyBox/thunder_north.jpg"
            };
            skyBox->MakeFromSideTextures(SKY_BOX_MAPS);
        }
        skyboxObject.reset( new realm::EntityObject(*skyBox, false) );
        world->add( skyboxObject.get() );

        // create scene
        {
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
			timer.reset(new ctrl::LooseTimer);
            timer->togglePause(true);
			physicsManager.setTimer( timer.get() );

            // create base control
			controlType = desc.control.type;
			{
                scene::node_ptr             graphicsModel;
                physics::physics_model_ptr  physicsModel;

                if ( std::string(desc.control.graphicsModel) == desc.control.physicsModel )
                {
				    database::library_ptr library = database::loadLibrary(desc.control.graphicsModel);

                    if ( !library->getVisualScenes().empty() ) {
				        graphicsModel = library->getVisualScenes().front().second;
                    }

                    if ( !library->getPhysicsScenes().empty() ) {
				        physicsModel = library->getPhysicsScenes().front().second;
                    }
                }
                else
                {
				    database::library_ptr library = database::loadLibrary(desc.control.graphicsModel);

                    if ( !library->getVisualScenes().empty() ) {
				        graphicsModel = library->getVisualScenes().front().second;
                    }

                    physicsModel = database::loadPhysicsScene(desc.control.physicsModel);
                }

                if ( scene::MatrixTransform* transform = dynamic_cast<scene::MatrixTransform*>(graphicsModel.get()) ) {
                    transform->setTransform(desc.control.transform);
                }

				modelObject.reset( new realm::CompoundObject(graphicsModel.get(), true, physicsModel.get()) );
				realm::currentWorld()->add( modelObject.get() );

                if ( scene::MatrixTransform* transform = dynamic_cast<scene::MatrixTransform*>(graphicsModel.get()) ) {
                    transform->setTransform( math::make_identity<float, 4>() );
                }

    	        switch (controlType)
                {
                    case CHAIN_CONTROL:
                    {
                        {
                            chain_rl_control* lc = new chain_rl_control(timer);
				            lc->loadConfig(desc.control.config);
				            lc->setTargetModel(graphicsModel);
				            lc->setPhysicsModel(physicsModel);
                            controls.push_back( ctrl::physics_control_ptr(lc) );

                            chain_direct_control* dc = new chain_direct_control( timer, 
                                                                                 lc->getEnvironment(), 
                                                                                 lc->getActionFunction(),
                                                                                 lc->getTimeInterval() );
                            controls.push_back( ctrl::physics_control_ptr(dc) );
                        }

                        {
                            chain_pd_control* control = new chain_pd_control(timer);
				            control->loadConfig(desc.control.config);
				            control->setTargetModel(graphicsModel);
				            control->setPhysicsModel(physicsModel);
                            controls.push_back( ctrl::physics_control_ptr(control) );
                        }

                        controlIndex = 2;

                        break;
                    }

                    case INVERSE_DYNAMICS_CONTROL:
                    {
                        //learningControl.reset( new id_control(timer) );
                        break;
                    }

                    case HUMANOID_CONTROL:
                    {
                        human_control* control = new human_control(timer);
			            control->loadConfig(desc.control.config);
			            control->setTargetModel(graphicsModel);
			            control->setPhysicsModel(physicsModel);
                        controls.push_back( ctrl::physics_control_ptr(control) );

                        controlIndex = 0;

                        break;
                    }

                    default:
                        break;
                }
			}

            // create scene
            {
                database::library_ptr  library       = database::loadLibrary(desc.arenaFile);
                scene::Node*           graphicsModel = library->getVisualScenes().front().second.get();
                physics::PhysicsModel* physicsModel  = library->getPhysicsScenes().front().second.get();

                arenaObject.reset( new realm::CompoundObject(graphicsModel, false, physicsModel) );
                world->add( arenaObject.get() );
            }

            // create cube for throwing
            {
                database::library_ptr  library       = database::loadLibrary("Data/Models/cube.dae");
                scene::Node*           graphicsModel = library->getVisualScenes().front().second.get();
                physics::PhysicsModel* physicsModel  = library->getPhysicsScenes().front().second.get();

                cubeObject.reset( new realm::CompoundObject(graphicsModel, true, physicsModel) );
                world->add( cubeObject.get() );
            }

            // create light
            scene::DirectionalLight* light = new DirectionalLight();
            light->setColor( math::Vector4f(0.8f, 0.8f, 0.8f, 1.0f) );
            light->setAmbient(0.3f);
            light->setIntensity(0.5f);
            light->setDirection( math::Vector3f(-1.5f, -0.5f, 0.85f) );

            world->add( new realm::EntityObject(*light, false) );
        }

        // Create camera
        {
            sgl::rectangle viewport(0, 0, desc.windowWidth, desc.windowHeight);
            camera.reset( createMainCamera(viewport) );
            graphicsManager.addCamera( camera.get() );
        }

        // setup states
        camera->setPosition( math::Vector3f(1.5f, 2.0f, 1.5f) );
        camera->setDirection( math::Vector3f(0.0f, 1.0f, 0.0f) - camera->getPosition() );
        camera->setUp( math::Vector3f(0.0f, 1.0f, 0.0f) );

        // create debug mesh
        debugMesh.reset(new graphics::DebugMesh);
        world->add( new realm::EntityObject(*debugMesh, false) );

        // create font
        debugFont.reset( graphics::currentDevice()->CreateFont() );
        {
            sgl::Image* image = graphics::currentDevice()->CreateImage();
            image->LoadFromFile("Data/Fonts/font.png");
            debugFont->SetTexture( image->CreateTexture2D() );
        }

		// create physics debug mesh
		//PhysicsDebugVisitor vis;
		//vis.traverse(*modelObject->getRoot());
		//vis.traverse(*arenaObject->getRoot());
		//vis.traverse(*cubeObject->getRoot());

        // attach signals
        graphicsManager.connectPostFrameRenderCallback( boost::bind(&Scene::OnPostRender, this) );
    }

    // setup input
    {
        using boost::bind;
        using namespace slon::input;

        InputManager& inputManager = engine->getInputManager();
        inputManager.showCursor(true);

        keyboardHandler.reset( new KeyboardHandler() );
        inputManager.addInputHandler( keyboardHandler.get() );

        keyboardHandler->connectKeyPressEventHandler( input::KEY_F1,        bind(&Scene::toggleHelpText,    this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_F9,        bind(&Scene::takeScreenShot,    this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_ESCAPE,    bind(&Scene::stopDemo,          this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_SPACE,     bind(&Scene::restart,           this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_p,         bind(&Scene::switchPause,       this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_r,         bind(&Scene::switchRealtime,    this) );
        keyboardHandler->connectKeyPressEventHandler( input::KEY_l,         bind(&Scene::switchLearning,    this) );

        keyboardHandler->connectKeyDownHandler( input::KEY_w,   bind(&Scene::flyCamera, this, math::Vector3f( 0.0f,  0.0f,  1.0f)) );
        keyboardHandler->connectKeyDownHandler( input::KEY_s,   bind(&Scene::flyCamera, this, math::Vector3f( 0.0f,  0.0f, -1.0f)) );
        keyboardHandler->connectKeyDownHandler( input::KEY_d,   bind(&Scene::flyCamera, this, math::Vector3f( 1.0f,  0.0f,  0.0f)) );
        keyboardHandler->connectKeyDownHandler( input::KEY_a,   bind(&Scene::flyCamera, this, math::Vector3f(-1.0f,  0.0f,  0.0f)) );
        keyboardHandler->connectKeyDownHandler( input::KEY_q,   bind(&Scene::flyCamera, this, math::Vector3f( 0.0f,  1.0f,  0.0f)) );
        keyboardHandler->connectKeyDownHandler( input::KEY_e,   bind(&Scene::flyCamera, this, math::Vector3f( 0.0f, -1.0f,  0.0f)) );

        keyboardHandler->connectKeyDownHandler( input::KEY_UP,      bind(&Scene::modifyTimeScale, this,  0.5f) );
        keyboardHandler->connectKeyDownHandler( input::KEY_DOWN,    bind(&Scene::modifyTimeScale, this, -0.5f) );

        // setup mouse input
        mouseHandler.reset( new MouseHandler() );
        mouseHandler->connectMouseButtonDownEventHandler( MBUTTON_LEFT, boost::bind(&Scene::toggleRotation, this, true) );
        mouseHandler->connectMouseButtonUpEventHandler( MBUTTON_LEFT, boost::bind(&Scene::toggleRotation, this, false) );
        mouseHandler->connectMouseButtonDownEventHandler( MBUTTON_RIGHT, boost::bind(&Scene::throwCube, this) );
        inputManager.addInputHandler( mouseHandler.get() );
    }

    controls[controlIndex]->acquire();

    Engine::DESC eDesc;
    eDesc.multithreaded = false;
    eDesc.grabInput     = false;
    timer->togglePause(false);
    engine->run(eDesc);
}

void Scene::toggleHelpText()   { drawHelp = !drawHelp; } // F1
// Such a mess, because need to take screenshot after scene rendering
void Scene::takeScreenShot()   { needToTakeScreenshot = true; } // F9
void Scene::stopDemo()         { Engine::Instance()->stop(); } // ESC

void Scene::toggleRotation(bool toggle)
{
    if ( toggle && !mouseMotionConnection.connected() ) {
        mouseMotionConnection = mouseHandler->connectRelativeMouseMotionEventHandler( bind(&Scene::turnCamera, this, _1, _2) );
    }
    else if ( !toggle && mouseMotionConnection.connected() ) {
        mouseMotionConnection.disconnect();
    }
}

void Scene::showScene()
{
    if ( !cubeObject->getWorld() )
    {
        world->add( cubeObject.get() );
        world->add( arenaObject.get() );
        world->add( modelObject.get() );
        world->add( skyboxObject.get() );
    }
}

void Scene::hideScene()
{
    if ( cubeObject->getWorld() )
    {
        world->remove( cubeObject.get() );
        world->remove( arenaObject.get() );
        world->remove( modelObject.get() );
        world->remove( skyboxObject.get() );
    }
}

void Scene::switchRealtime()   
{
    timer->toggleRealtime( !timer->isRealtime() ); 
	 
    realm::World* world = Engine::Instance()->getWorld();
    if ( timer->isRealtime() ) {
        hideScene();
    }
    else {
        showScene();
    }
}

void Scene::switchLearning()
{
    controls[controlIndex]->unacquire();
    if ( ++controlIndex >= controls.size() ) {
        controlIndex = 0;
    }
    controls[controlIndex]->acquire();
    /*
    if (controlIndex == 0)
    {
	    if ()
	    {
		    control->unacquire();
		    control = learningControl;
		    control->acquire();
		    controlBehaviour = LEARNING_CONTROL;
	    }
	    else if (controlIndex ==  
	    {
		    control->unacquire();
            timer->toggleRealtime(false);
            showScene();

		    switch (controlType)
            {
                case CHAIN_CONTROL:
                {
                    chain_control& lc = static_cast<chain_control&>(*learningControl);
                    control.reset( new chain_control::direct_control_type( timer, 
                                                                           lc.getEnvironment(), 
                                                                           lc.getActionFunction(),
                                                                           lc.getTimeInterval() ) );
                }
                break;

                case INVERSE_DYNAMICS_CONTROL:
                {
                    id_control& lc = static_cast<id_control&>(*learningControl);
                    control.reset( new id_control::direct_control_type( timer, 
                                                                        lc.getEnvironment(), 
                                                                        lc.getActionFunction(),
                                                                        lc.getTimeInterval() ) );
                }
                break;

            }

            control->setTargetModel( scene::node_ptr(modelObject->getRoot()) );
            control->setPhysicsModel( modelObject->getPhysicsModel() );
		    control->acquire();
		    controlBehaviour = DIRECT_CONTROL;
	    }
    }
    */
}

void Scene::switchPause()  
{ 
	timer->togglePause( !timer->isPaused() );
}

void Scene::restart()
{
    controls[controlIndex]->unacquire();
    controls[controlIndex]->acquire();
}

void Scene::throwCube()
{
    if ( cubeObject->getWorld() ) {
        world->remove( cubeObject.get() );
    }

    // throw rigid body
    physics::RigidBody& rigidBody = **cubeObject->getPhysicsModel()->firstRigidBody();
    physics::RigidBody::state_desc desc = rigidBody.getStateDesc();
	{
		desc.transform      = physics::Matrix4r( math::make_translation(camera->getPosition().x, camera->getPosition().y, camera->getPosition().z) );
		desc.linearVelocity = physics::Vector3r( camera->getDirection() * 10.0f );
	}
	rigidBody.reset(desc);

    world->add( cubeObject.get() );
}

void Scene::flyCamera(const math::Vector3f& speed)    
{ 
    camera->moveForward(speed.z * dt); 
    camera->moveRight(speed.x * dt); 
    camera->moveUp(speed.y * dt); 
}

void Scene::turnCamera(int xrel, int yrel) // on mouse motion
{
    const float bottom_threshold = -0.9f;
    const float top_threshold    =  0.5f;

    math::Vector2f rel = math::Vector2f( -static_cast<float>(xrel),
										  static_cast<float>(yrel) );
    camera->turnAroundAxis( -rel.x * 0.002f, math::Vector3f(0.0f, 1.0f, 0.0f) );
    camera->turnPitch(rel.y * 0.002f);

    math::Vector3f direction = camera->getDirection();
    if (direction.y < bottom_threshold)
    {
        direction.y = bottom_threshold;
        camera->setDirection( normalize(direction) );
    }
    else if (direction.y > top_threshold)
    {
        direction.y = top_threshold;
        camera->setDirection( normalize(direction) );
    }
}

void Scene::modifyTimeScale(float modifier)
{
	timer->setTimeScale( timer->getTimeScale() + float(modifier * dt) );
}

void Scene::renderCommon()
{
    using namespace std;
	using namespace graphics;
	using namespace graphics::debug;

    // settings
    const math::Vector2i fpsTextSize(10, 12);
    const math::Vector2i helpTextSize(10, 12);
    const math::Vector2i modeTextSize(16, 18);
    const math::Vector2i realtimeTextSize0(30, 36);
    const math::Vector2i realtimeTextSize1(20, 24);

    // calculate fps using frame render time
    const float fpsAdaptSpeed = 0.01f;
    fps = fps * (1.0f - fpsAdaptSpeed) + fpsAdaptSpeed / dt;

    debugMesh->clear();

    // draw fps
    {
        ostringstream ss;
        ss << "FPS: " 
           << std::fixed
           << std::setprecision(1)
           << fps 
           << endl;

        (*debugMesh) << font( debugFont.get() )
                     << text_size(fpsTextSize)
                     << debug::transform( math::make_translation( (screenSize.x - ss.str().length() * fpsTextSize.x) / 2.0f, 10.0f, 0.0f) )
                     << color(1.0f, 1.0f, 1.0f, 1.0f)
                     << text( ss.str() );
    }

    // draw simulation time
    {
        ostringstream ss;
        ss << "simulation time: " 
           << std::fixed
           << std::setprecision(1)
           << timer->getTime() << endl
           << "time scale: ";

        if ( timer->isRealtime() ) {
            ss << "max\n";
        }
        else 
        {
            ss << std::fixed
               << std::setprecision(2)
               << timer->getTimeScale() << endl;
        }

        (*debugMesh) << font( debugFont.get() )
                     << text_size(helpTextSize)
					 << debug::transform( math::make_translation(screenSize.x - 160.0f, 10.0f, 0.0f) )
                     << color(1.0f, 1.0f, 1.0f, 1.0f)
                     << text( ss.str() );
    }

    // draw help text
    if (drawHelp)
    {
        ostringstream ss;
        ss << "Toggle help: F1\n"
           << "Toggle realtime: R\n"
           << "Toggle pause: P\n"
           << "Toggle learning: L\n"
           << "Toggle heuristic: H\n"
           << "Restart simulation: space\n"
           << "Simulation speed: up,down\n"
           << "Moving: w,s,a,d,q,e\n";

        (*debugMesh) << font( debugFont.get() )
                     << text_size(helpTextSize)
					 << debug::transform( math::make_translation(10.0f, 10.0f, 0.0f) )
                     << color(1.0f, 1.0f, 1.0f, 1.0f)
                     << text( ss.str() );
    }

    // draw other info
    {
        ostringstream ss;
        ss << "Mode: ";
/*
        switch (controlBehaviour)
        {
        case DIRECT_CONTROL:
            ss << "Simulation";
            break;

        case LEARNING_CONTROL:
            ss << "Learning";
            break;

        default:
            assert(!"Can't get here");
        }
*/
        (*debugMesh) << font( debugFont.get() )
                     << text_size(modeTextSize)
			         << debug::transform( math::make_translation((screenSize.x - ss.str().length() * modeTextSize.x) / 2.0f, 25.0f, 0.0f) )
                     << color(1.0f, 0.0f, 0.0f, 1.0f)
                     << text( ss.str() );
    }

    if ( timer->isRealtime() )
    {
        (*debugMesh) << font( debugFont.get() )
                     << color(1.0f, 1.0f, 1.0f, 1.0f)
                     << text_size(realtimeTextSize0)
			         << debug::transform( math::make_translation(screenSize.x / 2.0f - 200.0f, screenSize.y / 2.0f - 200.0f, 0.0f) )
                     << text("Learning in progress...")
                     << text_size(realtimeTextSize1)
			         << debug::transform( math::make_translation(screenSize.x / 2.0f - 140.0f, screenSize.y / 2.0f - 150.0f, 0.0f) )
                     << text("Press R to view scene");
    }
}
	
void Scene::toggleEngine(const math::Vector3f& torque)
{
	userAction = true;
	userTorque = torque;
}

void Scene::OnPostRender()
{
    using namespace std;

    // render gui
    dt   = float(Engine::Instance()->getSimulationTimer().getTime() - time);
    time = float(Engine::Instance()->getSimulationTimer().getTime());
    renderCommon();

    // here we can take screenshot
    if ( needToTakeScreenshot )
    {
        // get time
        static int screenNum = 0;

        ostringstream numSS;
        numSS << screenNum++;
        string name = string("screen_") + numSS.str() + ".bmp";

        // screenshot
        sgl::ref_ptr<sgl::Image> image( graphics::currentDevice()->CreateImage() );
        //currentDevice()->TakeScreenshot( image.get() );
        //image->SaveToFile( name.c_str() );;

        needToTakeScreenshot = false;
    }
}
