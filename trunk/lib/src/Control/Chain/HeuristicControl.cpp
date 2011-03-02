/*
#include "Control/Chain/HeuristicControl.h"
#include <slon/Physics/PhysicsManager.h>

namespace ctrl {
namespace chain {

HeuristicControl::HeuristicControl(const ChainEnvironment* env_)
:   env(env_)
{
}

void HeuristicControl::makeAction()
{
    motorForces.resize( env->getActionSize() );

    math::Vector3f gravity         = physics::currentPhysicsManager().getDynamicsWorld()->getGravity();
    const float    deviationFactor = 10.0f;

    // find center of mass of the chain
    math::Vector3f footHold = math::get_translation( env->constraints[0]->getRigidBodyA()->getTransform() );
    for (size_t i = 0; i<env->constraints.size(); ++i)
    {
        physics::Constraint* cons = env->constraints[i].get();
        math::Vector3f       vec  = math::get_translation( cons->getRigidBodyB()->getTransform() ) - footHold;

        // 1.0 - cos(vec, gravity)
        float deviation = 1.0f - powf( math::dot(vec, -gravity) / ( math::length(gravity) * math::length(vec) ), deviationFactor );

        // go to constraint space
        vec = math::normalize( math::Vector3f(vec.x, 0.0f, vec.z) );
        vec = math::invert( math::from_matrix( cons->getRigidBodyA()->getTransform() ) ) * vec; // to parent cone space
        vec = math::from_matrix( cons->getStateDesc().frames[0] ) * vec;                        // to constraint space

        motorForces[i * 2]     = -vec.y * deviation * env->maxForce;
        motorForces[i * 2 + 1] =  vec.x * deviation * env->maxForce; 
    }
}

} // namespace chain
} // namespace ctrl
*/