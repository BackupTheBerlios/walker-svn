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
        physics::RigidBody*             rigidBody = rbTransform->getRigidBody();
        const physics::CollisionShape*  colShape  = rigidBody->getCollisionShape();
        math::Vector3f                  scaling   = rbTransform->getScaling();
        math::Matrix4f                  objTrans  = math::make_scaling(1.0f / scaling.x, 1.0f / scaling.y, 1.0f / scaling.z);
        graphics::DebugMesh*            debugMesh = new graphics::DebugMesh();
        switch ( colShape->getShapeType() ) 
        {
        case physics::CollisionShape::BOX:
            *debugMesh << graphics::debug::transform(objTrans) << *static_cast<const physics::BoxShape*>(colShape);
            break;

        case physics::CollisionShape::CONE:
            *debugMesh << graphics::debug::transform(objTrans) << *static_cast<const physics::ConeShape*>(colShape);
            break;
        }
        rbTransform->addChild(*debugMesh);
    }

    base_type::visitTransform(transform);
}
