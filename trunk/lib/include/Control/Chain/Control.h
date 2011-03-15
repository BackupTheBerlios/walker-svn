#ifndef __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__
#define __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__

#include "../../Utility/Math.h"
#include "../PhysicsControl.h"
#include "../Utility/LooseTimer.h"
#include "Environment.h"
#include <slon/Graphics/Renderable/DebugMesh.h>
#include <slon/Physics/RigidBody.h>

namespace ctrl {
namespace chain {

class Control :
    public PhysicsControl
{
public:
    typedef Environment environment_type;
	
public:
    // physics
    typedef sgl::vector
	<
		physics::RigidBody::state_desc
#ifndef SLON_ENGINE_USE_DOUBLE_PRECISION_PHYSICS
		, sgl::aligned_allocator<physics::RigidBody::state_desc> 
#endif
	> rigid_body_desc_vector;

    typedef sgl::vector
	<
		physics::Constraint::state_desc
#ifndef SLON_ENGINE_USE_DOUBLE_PRECISION_PHYSICS
		, sgl::aligned_allocator<physics::Constraint::state_desc>
#endif
	> constraint_desc_vector;

public:
    Control(const loose_timer_ptr& timer, bool multithreaded = true);

    // Override Control
	void loadConfig(const std::string& fileName);
    void setTargetModel(const scene::node_ptr& targetModel);
    void setPhysicsModel(const physics::physics_model_ptr& physicsModel);
    
	void toggleDebugDraw(bool toggle);

    environment_type* getEnvironment() { return environment.get(); }

    // Methods for testing and debug
    void bendMax();
    void bendRandom();

protected:
    virtual void initialize();
	virtual void drawDebugInfo();

    // Override PhysicsControl
    void   acquire_safe();
	void   unacquire_safe();
    double post_sync();

    // Misc
    ublas::vector<physics::real> getGravityCompensation() const;

protected:
	// settings
    bool            freeJoints;
    bool            randomStartup;
    bool            gravityCompensation;
    double          episodeLength;
    physics::real   maxForce;
    physics::real   maxVelocity;
	
    bool            debugDraw;
    float           debugDrawForceScale;
	
    // scene
    graphics::debug_mesh_ptr    debugMesh;
    sgl::ref_ptr<sgl::Font>     debugFont;
    rigid_body_desc_vector      rigidBodiesInitialDescs;
    constraint_desc_vector      constraintsInitialDescs;

    // control
    environment_ptr             environment;
};

} // namespace chain
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CHAIN_CONTROL_H__
