#include "Control/PhysicsEnvironment.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <slon/Physics/PhysicsManager.h>
#include <slon/Physics/ServoMotor.h>
#include <slon/Physics/VelocityMotor.h>

namespace ctrl {

PhysicsEnvironment::PhysicsEnvironment(const DESC& desc) 
:   maxVelocity(desc.maxVelocity)
,   maxForce(desc.maxForce)
,   freeJoints(desc.freeJoints)
,   state(READY)
{
    std::copy( desc.rigidBodies, desc.rigidBodies + desc.numRigidBodies, std::back_inserter(rigidBodies) );
    std::copy( desc.constraints, desc.constraints + desc.numConstraints, std::back_inserter(constraints) );
    setControlType(CONTROL_VELOCITY);

    // make connections for handling contacts
    for (size_t i = 0; i<rigidBodies.size(); ++i)
    {
        scoped_connection* connection = new scoped_connection( 
            rigidBodies[i]->connectContactAppearCallback( boost::bind(&PhysicsEnvironment::handleAppearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );

        connection = new scoped_connection( 
            rigidBodies[i]->connectContactDissapearCallback( boost::bind(&PhysicsEnvironment::handleDissappearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );
    }

    reset();
}

void PhysicsEnvironment::setControlType(CONTROL_TYPE controlType_)
{
    controlType = controlType_;

    motors.clear();
    for (size_t i = 0; i<constraints.size(); ++i) 
    {
        for (int j = 0; j<3; ++j) 
        {
            physics::Motor* motor = 0;
            switch (controlType)
            {
            case CONTROL_FORCE:
                motor = constraints[i]->createServoMotor(physics::Motor::TYPE(physics::Motor::MOTOR_X_ROT + j));
                break;

            case CONTROL_VELOCITY:
                motor = constraints[i]->createVelocityMotor(physics::Motor::TYPE(physics::Motor::MOTOR_X_ROT + j));
                break;

            default:
                assert(!"can't get here");
            }

            if (motor) {
                motors.push_back(motor);
            }
        }
    }

    stateSize  = motors.size();
    actionSize = motors.size();

    reset();
}

void PhysicsEnvironment::reset()
{
    // get initial state
    position       = ublas::zero_vector<float>(actionSize);
    velocity       = ublas::zero_vector<float>(actionSize);
    targetForce    = ublas::zero_vector<float>(actionSize);
    targetVelocity = ublas::zero_vector<float>(actionSize);

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
            mass              += rigidBodies[i]->getMass();
        }
    }
    initialMassCenter /= mass;
    centerHeight = initialCenterHeight;

    std::for_each( boost::make_indirect_iterator( rigidBodies.begin() ),
                   boost::make_indirect_iterator( rigidBodies.end() ),
                   boost::bind(&physics::RigidBody::setActivationState, _1, physics::RigidBody::AS_DISABLE_DEACTIVATION) );
}

void PhysicsEnvironment::makeAction()
{
    std::for_each( boost::make_indirect_iterator( rigidBodies.begin() ),
                   boost::make_indirect_iterator( rigidBodies.end() ),
                   boost::bind(&physics::RigidBody::applyImpulse, _1, math::Vector3f(0.01f, 0.01f, 0.01f), math::Vector3f(0.0f, 0.0f, 0.0f)) );

    switch (controlType)
    {
        case CONTROL_FORCE:
        {    
            for (size_t i = 0; i<motors.size(); ++i) 
            {
                targetForce[i] = targetForce[i] > maxForce ? maxForce : targetForce[i] < -maxForce ? -maxForce : targetForce[i]; // clamped [-maxForce, maxForce]
                static_cast<physics::ServoMotor*>(motors[i])->setTargetForce(targetForce[i]);
            }
            break;
        }

        case CONTROL_VELOCITY:
        {
            for (size_t i = 0; i<motors.size(); ++i) 
            {
                physics::VelocityMotor* motor = static_cast<physics::VelocityMotor*>(motors[i]);
                motor->toggle(true);
                motor->setMaxForce(maxForce);
                motor->setTargetVelocity(targetVelocity[i]);
            }
            break;
        }

        default:
            assert(!"can't get here");
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    actionReadyCondition.notify_one();
}

void PhysicsEnvironment::makeState()
{
    using namespace math;

    // query first body angles
    rotation    = from_matrix( rigidBodies[0]->getTransform() );      // matrix -> quaternion
    angVelocity = rigidBodies[0]->getAngularVelocity() / maxVelocity;  

    // query angles
    for (size_t i = 0; i<motors.size(); ++i) 
    {
        position[i] = motors[i]->getPosition();
        velocity[i] = motors[i]->getVelocity();
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    stateReadyCondition.notify_one();
}

void PhysicsEnvironment::handleAppearingContact(const physics::Contact& c)
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

void PhysicsEnvironment::handleDissappearingContact(const physics::Contact& c)
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
