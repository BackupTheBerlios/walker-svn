#ifndef __WALKER_YARD_SCENE_H__
#define __WALKER_YARD_SCENE_H__

#include "Control/Chain/PDControl.h"
#include "Control/Chain/RLControl.h"
#include "Control/Humanoid/Control.h"
#include "Control/PhysicsControl.h"
#include "Control/InverseDynamics/RLControl.h"
#include <sgl/Font.h>
#include <slon/Graphics/Renderable/DebugMesh.h>
#include <slon/Input/KeyboardHandler.h>
#include <slon/Input/MouseHandler.h>
#include <slon/Realm/World.h>
#include <slon/Realm/Object/CompoundObject.h>
#include <slon/Realm/Object/EntityObject.h>
#include <slon/Scene/Camera/LookAtCamera.h>
#include <vector>
#undef CreateFont

using namespace slon;

class Scene
{
private:
    typedef ctrl::id::RLControl     id_control;

    typedef ctrl::chain::RLControl                  chain_rl_control;
    typedef ctrl::chain::PDControl                  chain_pd_control;
    typedef chain_rl_control::direct_control_type   chain_direct_control;
    typedef ctrl::human::Control                    human_control;

    typedef std::vector<ctrl::physics_control_ptr>  control_vector;

public:
	enum TARGET_CONTROL_TYPE
	{
		CHAIN_CONTROL,
		INVERSE_DYNAMICS_CONTROL,
        HUMANOID_CONTROL
	};

	struct CONTROL_DESC
	{
		const char*			graphicsModel;
		const char*			physicsModel;
		const char*			config;
        math::Matrix4f      transform;
		TARGET_CONTROL_TYPE	type;
	};

	struct DESC
	{ 
		unsigned		windowWidth;
        unsigned		windowHeight;
        bool			fullscreen;
		bool			ffp;
        unsigned		multisample;
        std::string		arenaFile;
		CONTROL_DESC	control;
	};

public:
    Scene(const DESC& desc);

private:
    void toggleHelpText();
    void takeScreenShot();
    void stopDemo();
    void toggleRotation(bool toggle);
    void switchRealtime();
    void switchLearning();
    void switchPause();
    void restart();
    void throwCube();
    void flyCamera(const math::Vector3f& speed);
    void turnCamera(int xrel, int yrel);
    void modifyTimeScale(float modifier);
    void renderCommon();
    void OnPostRender();

    void showScene();
    void hideScene();

	void toggleEngine(const math::Vector3f& torque);

private:
    // entities
    realm::world_ptr            world;
    realm::compound_object_ptr  cubeObject;
    realm::compound_object_ptr  arenaObject;
    realm::compound_object_ptr  modelObject;
    realm::entity_object_ptr    skyboxObject;
    scene::look_at_camera_ptr	camera;
    graphics::debug_mesh_ptr    debugMesh;
    sgl::ref_ptr<sgl::Font>     debugFont;

    // input
    input::keyboard_handler_ptr				keyboardHandler;
    input::mouse_handler_ptr				mouseHandler;
    input::MouseHandler::connection_type    mouseMotionConnection;

    // control
	TARGET_CONTROL_TYPE			    controlType;
    control_vector                  controls;
    size_t                          controlIndex;

	// other
	ctrl::loose_timer_ptr		    timer;

    // settings
    float			fps;
    float			time;
    float			dt;
    bool			drawHelp;
    bool			needToTakeScreenshot;
	math::Vector2ui	screenSize;
};

#endif // __WALKER_YARD_SCENE_H__
