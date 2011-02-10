#include "Control/Chain/ChainEnvironment.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <slon/Physics/PhysicsManager.h>

extern bool userAction;
extern math::Vector3f userTorque;

namespace ctrl {

ChainEnvironment::ChainEnvironment(const DESC& desc) 
:   maxVelocity(desc.maxVelocity)
,   maxForce(desc.maxForce)
,   freeJoints(desc.freeJoints)
,   terminal(false)
,   state(READY)
{
    std::copy( desc.rigidBodies, desc.rigidBodies + desc.numRigidBodies, std::back_inserter(rigidBodies) );
    std::copy( desc.constraints, desc.constraints + desc.numConstraints, std::back_inserter(constraints) );
    for (size_t i = 0; i<constraints.size(); ++i) 
    {
        motors.push_back( constraints[i]->getRotationalMotor(0) );
        motors.push_back( constraints[i]->getRotationalMotor(1) );
    }

    // 4 floats per constraint: 2 - angles of orientation, 2 - angular velocity
    // 6 floats for first rigid body
    stateSize  = /*6 +*/ constraints.size() * 2;
    actionSize = motors.size();

    // Angular velocities of the frames in the constraint coordinate system for X,Y axices
    constraintVelocities.resize( constraints.size() );
    constraintAngles.resize( constraints.size() );
    motorForces.resize( motors.size() );
    motorVelocities.resize( motors.size() );

    // make connections for handling contacts
    for (size_t i = 0; i<rigidBodies.size(); ++i)
    {
        scoped_connection* connection = new scoped_connection( 
            rigidBodies[i]->connectContactAppearCallback( boost::bind(&ChainEnvironment::handleAppearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );

        connection = new scoped_connection( 
            rigidBodies[i]->connectContactDissapearCallback( boost::bind(&ChainEnvironment::handleDissappearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );
    }

    episodic = true;
    reset();
}

void ChainEnvironment::reset()
{
    // get initial state
    firstStep = true;
    terminal = false;
    std::fill( constraintAngles.begin(), constraintAngles.end(), math::Vector2f(0.0f, 0.0f) );
    std::fill( constraintVelocities.begin(), constraintVelocities.end(), math::Vector2f(0.0f, 0.0f) );
    std::fill( motorForces.begin(), motorForces.end(), 0.0f );

    updateTime = physics::currentPhysicsManager().getTimer()->getTime();

    // find center of mass of the chain
    numDynamicRigidBodies = 0;
    initialCenterHeight   = 0.0f;
    initialMassCenter     = math::Vector3f(0.0f, 0.0f, 0.0f);
    float mass            = 0.0f;
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        initialCenterHeight += math::get_translation( rigidBodies[i]->getTransform() ).y / rigidBodies.size();
        if ( rigidBodies[i]->getDynamicsType() == physics::RigidBody::DT_DYNAMIC )
        {
            numDynamicRigidBodies++;
            initialMassCenter += math::get_translation( rigidBodies[i]->getTransform() ) * rigidBodies[i]->getMass();
            mass       += rigidBodies[i]->getMass();
        }
    }
    initialMassCenter /= mass;
    centerHeight = initialCenterHeight;

    std::for_each( boost::make_indirect_iterator( rigidBodies.begin() ),
                   boost::make_indirect_iterator( rigidBodies.end() ),
                   boost::bind(&physics::RigidBody::setActivationState, _1, physics::RigidBody::AS_DISABLE_DEACTIVATION) );
    averageVel = 1.0;
    averagePos = 1.0;
}

void ChainEnvironment::makeAction()
{
    std::for_each( boost::make_indirect_iterator( rigidBodies.begin() ),
                   boost::make_indirect_iterator( rigidBodies.end() ),
                   boost::bind(&physics::RigidBody::applyImpulse, _1, math::Vector3f(0.01f, 0.01f, 0.01f), math::Vector3f(0.0f, 0.0f, 0.0f)) );

    for (size_t i = 0; i<motors.size(); ++i)
    {
		if (userAction)
		{
			motors[i]->setMaxForce( fabs(userTorque[i]) );
			motors[i]->setTargetVelocity( userTorque[i] );
			motors[i]->activate(fabs(userTorque[i]) > math::EPS_3f);
		}
		else
		{
			motors[i]->setMaxForce( motorForces[i] );
			motors[i]->setTargetVelocity( motorVelocities[i] );
			motors[i]->activate(motorForces[i] > math::EPS_3f);
		}
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    actionReadyCondition.notify_one();
}

void ChainEnvironment::makeState()
{
    using namespace math;

    // query first body angles
    rotation    = from_matrix( rigidBodies[0]->getTransform() );      // matrix -> quaternion
    angVelocity = rigidBodies[0]->getAngularVelocity() / maxVelocity;  

    // query angles
    vector_of_vector2f currentConstrantAngles( constraints.size() );
    for (size_t i = 0; i<constraints.size(); ++i)
    {
        currentConstrantAngles[i].x = constraints[i]->getRotationAngle(0);
        currentConstrantAngles[i].y = constraints[i]->getRotationAngle(1);
    }

    // Recalculate velocities of the axises in the constraint using time derivative
    // Use time derivative
    if (firstStep) 
    {
        constraintAngles.swap(currentConstrantAngles);
        firstStep = false;
    }
    else
    {
        double time     = physics::currentPhysicsManager().getTimer()->getTime();
        double timeStep = physics::currentPhysicsManager().getDynamicsWorld()->getStateDesc().fixedTimeStep;
        if (time - updateTime > timeStep - math::EPS_6f) 
        {
            for (size_t i = 0; i<constraints.size(); ++i) {
                constraintVelocities[i] = currentConstrantAngles[i] - constraintAngles[i];
            }
            constraintAngles.swap(currentConstrantAngles);
        }
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    stateReadyCondition.notify_one();
}

void ChainEnvironment::makeReward()
{
    math::Vector3f gravity = physics::currentPhysicsManager().getDynamicsWorld()->getGravity();

    // find center of mass of the chain
    centerHeight = 0.0f;
    float mass   = 0.0f;
    math::Vector3f massCenter(0.0f, 0.0f, 0.0f);
    math::Vector3f footHold = math::get_translation( rigidBodies[0]->getTransform() );
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        math::Vector3f center = math::get_translation( rigidBodies[i]->getTransform() );
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
    /*
    contactUpdateMutex.lock();
    if (activeContacts.size() >= 2 || centerHeight < 0.5f * initialCenterHeight) {
        terminal = true;
    }
    contactUpdateMutex.unlock();
    */

    //reward = centerHeight - initialCenterHeight;
    // calculate mass center deviation
    const float deviationFactor = 2.0f;

    reward = 0.0f;
    {
        math::Vector3f vec = massCenter - footHold;

        if ( math::length(vec) < math::EPS_3f ) {
            reward = 0.0f;
        }
        else
        {
            // cos(vec, gravity) ^ deviationFactor
            float deviation = powf( math::dot(vec, -gravity) / ( math::length(gravity) * math::length(vec) ), deviationFactor );
            reward          = deviation - 1.0f;
        }
    }

    if (episodic)
    {
        float posDeviation=0, velDeviation=0;
        bool fallxhigh = true;
        bool fallxlow = true;
        bool fallyhigh = true;
        bool fallylow = true;
        for (size_t i = 0; i<constraintAngles.size(); ++i) 
        {
            math::Vector3f lowLimit  = constraints[i]->getStateDesc().angularLimits[0];
            math::Vector3f highLimit = constraints[i]->getStateDesc().angularLimits[1];
            posDeviation    += pow(2.0f * (constraintAngles[i].x - lowLimit.x) / (highLimit.x - lowLimit.x) - 1.0f, 2);
            posDeviation    += pow(2.0f * (constraintAngles[i].y - lowLimit.y) / (highLimit.y - lowLimit.y) - 1.0f, 2);

            velDeviation += pow(2.0f * std::min(std::max(constraintVelocities[i].x, -maxVelocity), maxVelocity) / maxVelocity - 1.0f,2);
            velDeviation += pow(2.0f * std::min(std::max(constraintVelocities[i].y, -maxVelocity), maxVelocity) / maxVelocity - 1.0f,2);

            fallxhigh = fallxhigh && (highLimit.x - constraintAngles[i].x < 0.01);
            fallxlow  = fallxlow && (constraintAngles[i].x - lowLimit.x < 0.01);
            fallyhigh = fallyhigh && (highLimit.y - constraintAngles[i].y < 0.01);
            fallylow  = fallylow && (constraintAngles[i].y - lowLimit.y < 0.01);
            //fall = fall && ((constraintAngles[i].x - lowLimit.x < 0.01) || (highLimit.x - constraintAngles[i].x < 0.01) ||
            //       (constraintAngles[i].y - lowLimit.y < 0.01) || (highLimit.y - constraintAngles[i].y < 0.01));
        }
        posDeviation /= constraintAngles.size()*2.0f;
        velDeviation /= constraintAngles.size()*2.0f;
        
        averagePos = posDeviation + 0.9f*averagePos;
        averageVel = velDeviation + 0.9f*averageVel;
        float forceNorm = 0;
        for (size_t i = 0; i < motorForces.size(); ++i)
        {
            forceNorm += motorForces[i]*motorForces[i];
        }
        //reward -= 0.001*forceNorm;

        reward -= 0.001f*velDeviation;

        if (averagePos < 0.01 && averageVel < 0.01)
        {
            terminal = true;
            reward = 1.0;
        }
        if (fallxhigh || fallxlow || fallyhigh || fallylow)
        {
            terminal = true;
            reward = -6.0;
        }
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    rewardReadyCondition.notify_one();
}

void ChainEnvironment::handleAppearingContact(const physics::Contact& c)
{
    contact_pair cp(c.collisionObjects[0], c.collisionObjects[1]);
    if (cp.first > cp.second) {
        std::swap(cp.first, cp.second);
    }

    bool significant = true;
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        for (size_t j = 0; j<rigidBodies.size(); ++j)
        {
            if (rigidBodies[i].get() == cp.first && rigidBodies[j].get() == cp.second) 
            {
                significant = false;
                break;
            }
        }
    }

    if (significant) 
    {
        contactUpdateMutex.lock();
        activeContacts.insert(cp);
        contactUpdateMutex.unlock();
    }
}

void ChainEnvironment::handleDissappearingContact(const physics::Contact& c)
{
    contact_pair cp(c.collisionObjects[0], c.collisionObjects[1]);
    if (cp.first > cp.second) {
        std::swap(cp.first, cp.second);
    }

    contactUpdateMutex.lock();
    activeContacts.erase(cp);
    contactUpdateMutex.unlock();
}

} // namespace ctrl
