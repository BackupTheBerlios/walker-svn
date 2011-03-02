#include "Control/Utility/ControlElement.h"
#include <slon/Physics/RigidBody.h>

namespace {

    using namespace slon;
/*
    void create_shape_debug_mesh( const physics::CollisionShape&    shape,
                                  scene::Group&                     whereToAdd )
    {
        using namespace graphics::debug;

        physics_shape_debug_mesh_ptr debugMesh( new PhysicsShapeDebugMesh(shape) );
        whereToAdd.addChild(*debugMesh);
    }
*/
} // anonymous namespace

namespace ctrl {

ControlElement::ControlElement( scene::Geode&       _geode,
                                physics::RigidBody& _rigidBody ) :
    geode(&_geode),
    rigidBody(&_rigidBody),
    switchNode(new scene::Switch)
{
    // add geode to the zero group
    if ( geode->getParent() )
    {
        geode->getParent()->addChild(*switchNode);
        geode->getParent()->moveChild(*switchNode, *geode);
    }
    else {
        switchNode->addChild(*geode);
    }

    // add rigid body debug mesh
    //rigidBodyDebugMesh.reset( new graphics::debug::RigidBodyDebugMesh(*rigidBody) );
    //switchNode->addChild(*rigidBodyDebugMesh);
    // create_shape_debug_mesh(*physicsBone.getRigidBody()->getCollisionShape(), *switchNode);
}

math::Matrix4f ControlElement::getWorldTransform() const
{
    assert(rigidBody);
    return rigidBody->getTransform();
}

} // namespace ctrl

