#ifndef __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_CONTROL_H__
#define __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_CONTROL_H__

#include "../PhysicsControl.h"
#include "../Utility/LooseTimer.h"
#include "Environment.h"
#include <slon/Graphics/Renderable/DebugMesh.h>
#include <slon/Physics/RigidBody.h>

namespace ctrl {
namespace id {

class Control :
    public ctrl::PhysicsControl
{
public:
    typedef Environment                             environment_type;
    typedef boost::intrusive_ptr<environment_type>  environment_ptr;
	
protected:
    // physics
    typedef sgl::vector<physics::RigidBody::state_desc,
                        sgl::aligned_allocator<physics::RigidBody::state_desc> >    rigid_body_desc_vector;

    typedef sgl::vector<physics::Constraint::state_desc,
                        sgl::aligned_allocator<physics::Constraint::state_desc> >   constraint_desc_vector;

public:
    Control(const loose_timer_ptr& timer, bool multithreaded = true);

    // Override Control
	void loadConfig(const std::string& fileName);
    void setTargetModel(const scene::node_ptr& targetModel);
    void setPhysicsModel(const physics::physics_model_ptr& physicsModel);
    
	void toggleDebugDraw(bool toggle);

    environment_type* getEnvironment() { return environment.get(); }

protected:
    virtual void initialize();
	virtual void drawDebugInfo();

    // Override PhysicsControl
    virtual void   acquire_safe();
	virtual void   unacquire_safe();
    virtual double post_sync();

protected:
	// settings
    bool            freeJoints;
    bool            randomStartup;
    double          episodeLength;
    float           maxForce;
    float           maxVelocity;
	
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

} // namespace id
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_INVERSE_DYNAMICS_CONTROL_H__
