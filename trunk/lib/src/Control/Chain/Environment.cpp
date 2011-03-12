#include "Control/Chain/Environment.h"
#include <slon/Physics/PhysicsManager.h>

namespace ctrl {
namespace chain {

Environment::Environment(const PhysicsEnvironment::DESC& desc) 
:   PhysicsEnvironment(desc)
,   terminal(false)
,   episodic(true)
{
}

void Environment::reset()
{
    PhysicsEnvironment::reset();
/*
    physics::BulletRotationalMotor* btMotor = static_cast<physics::BulletRotationalMotor*>(motors[0]);
    btMotor->getBtMotor()->m_targetVelocity = 10.0f;
    btMotor->getBtMotor()->m_enableMotor    = true;
    btMotor->getBtMotor()->m_maxMotorForce  = 100.0f;
    
    btMotor = static_cast<physics::BulletRotationalMotor*>(motors[1]);
    btMotor->getBtMotor()->m_targetVelocity = 10.0f;
    btMotor->getBtMotor()->m_enableMotor    = true;
    btMotor->getBtMotor()->m_maxMotorForce  = 100.0f;
*/
    // get initial state
    terminal   = false;
    reward     = 0.0;
    averageVel = 1.0;
    averagePos = 1.0;
}

void Environment::makeReward()
{
    physics::Vector3r gravity = physics::currentPhysicsManager().getDynamicsWorld()->getGravity();

    // find center of mass of the chain
    centerHeight = 0.0;
    physics::real     mass(0.0);
    physics::Vector3r massCenter(0.0, 0.0, 0.0);
    physics::Vector3r footHold = math::get_translation( rigidBodies[0]->getTransform() );
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        math::Vector3r center = math::get_translation( rigidBodies[i]->getTransform() );
        centerHeight += center.y / rigidBodies.size();

        if ( math::dot(footHold, gravity) < math::dot(center, gravity) ) {
            footHold = center;
        }

        if ( rigidBodies[i]->getDynamicsType() == physics::RigidBody::DT_DYNAMIC )
        {
            massCenter   += math::get_translation( rigidBodies[i]->getTransform() ) * rigidBodies[i]->getMass();
            mass         += rigidBodies[i]->getMass();
        }
    }
    massCenter /= mass;

    // if two or more contacts with floor, then restart sausage
    
    if ( centerHeight < 0.8f * initialCenterHeight) {
        terminal = true;
    }
    

    reward = centerHeight - initialCenterHeight;
    // calculate mass center deviation
    const physics::real deviationFactor = 2.0;
/*
    reward = 0.0;
    {
        physics::Vector3r vec = massCenter - footHold;

        if ( math::length(vec) < math::EPS_3f ) {
            reward = 0.0;
        }
        else
        {
            // cos(vec, gravity) ^ deviationFactor
            physics::real deviation = powf( math::dot(vec, -gravity) / ( math::length(gravity) * math::length(vec) ), deviationFactor );
            reward = deviation - 1.0;
        }
    }
    */

    //if (episodic)
    //{
        //float posDeviation=0, velDeviation=0;
        //bool fallxhigh = true;
        //bool fallxlow = true;
        //bool fallyhigh = true;
        //bool fallylow = true;
        //for (size_t i = 0; i< constraints.size(); ++i) 
        //{
        //    math::Vector3f lowLimit  = constraints[i]->getStateDesc().angularLimits[0];
        //    math::Vector3f highLimit = constraints[i]->getStateDesc().angularLimits[1];
        //    //motors[i]->getPosition()
        //    posDeviation    += pow(2.0f * (position[i].x - lowLimit.x) / (highLimit.x - lowLimit.x) - 1.0f, 2);
        //    posDeviation    += pow(2.0f * (position[i].y - lowLimit.y) / (highLimit.y - lowLimit.y) - 1.0f, 2);

        //    velDeviation += pow(2.0f * std::min(std::max(velocity[i].x, -maxVelocity), maxVelocity) / maxVelocity - 1.0f,2);
        //    velDeviation += pow(2.0f * std::min(std::max(velocity[i].y, -maxVelocity), maxVelocity) / maxVelocity - 1.0f,2);

        //    fallxhigh = fallxhigh && (highLimit.x - position[i].x < 0.01);
        //    fallxlow  = fallxlow && (position[i].x - lowLimit.x < 0.01);
        //    fallyhigh = fallyhigh && (highLimit.y - position[i].y < 0.01);
        //    fallylow  = fallylow && (position[i].y - lowLimit.y < 0.01);
        //    //fall = fall && ((constraintAngles[i].x - lowLimit.x < 0.01) || (highLimit.x - constraintAngles[i].x < 0.01) ||
        //    //       (constraintAngles[i].y - lowLimit.y < 0.01) || (highLimit.y - constraintAngles[i].y < 0.01));
        //}

        //for (size_t i = 0; i < constraints.size(); ++i)
        //{
        //    math::Vector3f lowLimit  = constraints[i]->getStateDesc().angularLimits[0];
        //    math::Vector3f highLimit = constraints[i]->getStateDesc().angularLimits[1];

        //}
        //posDeviation /= position.size()*2.0f;
        //velDeviation /= position.size()*2.0f;
        //
        //averagePos = posDeviation + 0.9f*averagePos;
        //averageVel = velDeviation + 0.9f*averageVel;
        //float forceNorm = 0;
        //for (size_t i = 0; i < motorForces.size(); ++i)
        //{
        //    forceNorm += motorForces[i]*motorForces[i];
        //}
        ////reward -= 0.001*forceNorm;

        ////reward -= 0.001f*velDeviation;

        //if (averagePos < 0.01 && averageVel < 0.01)
        //{
        //    terminal = true;
        //    reward = 1.0;
        //}
        //if (fallxhigh || fallxlow || fallyhigh || fallylow)
        //{
        //    terminal = true;
        //    reward = -6.0;
        //}


    //}

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    rewardReadyCondition.notify_one();
}

} // namespace chain
} // namespace ctrl
