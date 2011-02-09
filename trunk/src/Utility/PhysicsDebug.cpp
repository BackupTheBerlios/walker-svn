#include "Utility/PhysicsDebug.h"
#include <slon/Graphics/Renderable/Debug/DebugDrawCommon.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawPhysics.h>
#include <slon/Scene/Physics/RigidBodyTransform.h>

void PhysicsDebugVisitor::visitTransform(scene::Transform& transform)
{
    using namespace scene;
    using namespace graphics::debug;

    RigidBodyTransform* rbTransform = dynamic_cast<RigidBodyTransform*>(&transform);
    if (rbTransform)
    {
        physics::RigidBody*  rigidBody = rbTransform->getRigidBody();
        math::Matrix4f       objTrans  = rigidBody->getTransform();
        graphics::DebugMesh* debugMesh = new graphics::DebugMesh();

        *debugMesh << graphics::debug::transform(objTrans) << *rigidBody->getCollisionShape();
        //rbTransform->addChild(*debugMesh);
    }

    base_type::visitTransform(transform);
}
