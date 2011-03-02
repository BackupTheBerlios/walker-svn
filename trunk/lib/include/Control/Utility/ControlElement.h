#ifndef __WALKER_YARD_CONTROL_CONTROL_ELEMENT_H__
#define __WALKER_YARD_CONTROL_CONTROL_ELEMENT_H__

//#include <slon/Graphics/Renderable/Debug/EdgeDebugMesh.h>
#include <slon/Physics/RigidBody.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawPhysics.h>
#include <slon/Scene/Switch.h>

//#include "Actuator/JointActuator.h"

namespace ctrl {

using namespace slon;

class ControlElement
{
private:
    static const int geode_index                 = 0;
    static const int rigid_body_debug_mesh_index = 1;
    static const int edge_debug_mesh_index       = 2;

public:
    ControlElement( scene::Geode&       geode,
                    physics::RigidBody& rigidBody );

    //void setActuator(const actuator_ptr& _actuator) { actuator = _actuator; }
    math::Matrix4f getWorldTransform() const;

    void toggleShapeDebugMesh(bool toggle);
    void toggleEdgeDebugMesh(bool toggle);
    void toggleRigidBodyDebugMesh(bool toggle);

    bool getShapeDebugMeshToggle() const;
    bool getEdgeDebugMeshToggle() const;
    bool getRigidBodyDebugMeshToggle() const;

private:
    scene::geode_ptr            geode;
    physics::rigid_body_ptr     rigidBody;
    scene::switch_ptr           switchNode;

    // debug draw
    //graphics::debug::rigid_body_debug_mesh_ptr     rigidBodyDebugMesh;
    //graphics::debug::edge_debug_mesh_ptr              edgeDebugMesh;
    //graphics::debug::rigid_body_debug_mesh_ptr        rigidBodyDebugMesh;

    // actuators
    // joint_actuator_ptr          actuator;
};

} // namesapce ctrl

#endif // __WALKER_YARD_CONTROL_CONTROL_ELEMENT_H__
