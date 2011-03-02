#include "Control/Python/PythonEnvironment.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <slon/Physics/PhysicsManager.h>

extern bool userActions;

namespace ctrl {

PythonEnvironment::PythonEnvironment(const DESC& desc) 
:   maxVelocity(desc.maxVelocity)
,   maxForce(desc.maxForce)
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
            rigidBodies[i]->connectContactAppearCallback( boost::bind(&PythonEnvironment::handleAppearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );

        connection = new scoped_connection( 
            rigidBodies[i]->connectContactDissapearCallback( boost::bind(&PythonEnvironment::handleDissappearingContact, this, _1) ) );
        contactConnections.push_back( connection_ptr(connection) );
    }

    reset();
}

void PythonEnvironment::loadImpl(const std::string& pyFileName, const std::string& pyObject)
{
}

void PythonEnvironment::reset()
{
    // get initial state
    terminal = false;
    std::fill( constraintAngles.begin(), constraintAngles.end(), math::Vector2f(0.0f, 0.0f) );
    std::fill( constraintVelocities.begin(), constraintVelocities.end(), math::Vector2f(0.0f, 0.0f) );
    std::fill( motorForces.begin(), motorForces.end(), 0.0f );

    updateTime = physics::currentPhysicsManager().getTimer()->getTime();
    getState();

    // find center of mass of the chain
    initialCenterHeight = 0.0f;
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        initialCenterHeight += math::get_translation( rigidBodies[i]->getTransform() ).y / rigidBodies.size();
    }
    centerHeight = initialCenterHeight;

    std::for_each( boost::make_indirect_iterator( rigidBodies.begin() ),
                   boost::make_indirect_iterator( rigidBodies.end() ),
                   boost::bind(&physics::RigidBody::setActivationState, _1, physics::RigidBody::AS_DISABLE_DEACTIVATION) );
}

void PythonEnvironment::makeAction()
{
    if (!userActions)
    {
        for (size_t i = 0; i<motors.size(); ++i)
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

void PythonEnvironment::makeState()
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
    double time     = physics::currentPhysicsManager().getTimer()->getTime();
    double timeStep = physics::currentPhysicsManager().getDynamicsWorld()->getStateDesc().fixedTimeStep;
    if (time - updateTime > timeStep - math::EPS_6f) 
    {
        for (size_t i = 0; i<constraints.size(); ++i) {
            constraintVelocities[i] = currentConstrantAngles[i] - constraintAngles[i];
        }
        constraintAngles.swap(currentConstrantAngles);
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    stateReadyCondition.notify_one();
}

void PythonEnvironment::makeReward()
{
    // find center of mass of the chain
    centerHeight = 0.0f;
    float mass   = 0.0f;
    math::Vector3f massCenter(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i<rigidBodies.size(); ++i) 
    {
        centerHeight += math::get_translation( rigidBodies[i]->getTransform() ).y / rigidBodies.size();

        if ( rigidBodies[i]->getDynamicsType() == physics::RigidBody::DT_DYNAMIC )
        {
            massCenter   += math::get_translation( rigidBodies[i]->getTransform() ) * rigidBodies[i]->getMass();
            mass         += rigidBodies[i]->getMass();
        }
    }

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
    const float maxReward       = 1.0f;
    const float deviationFactor = 5.0f;

    reward = 0.0f;
    {
        math::Vector3f gravity = physics::currentPhysicsManager().getDynamicsWorld()->getGravity();
        math::Vector3f vec     = massCenter - math::get_translation( rigidBodies[0]->getTransform() );

        float deviation = powf( math::dot(vec, -gravity) / ( math::length(gravity) * math::length(vec) ), deviationFactor );
        reward         += deviation;
    }
    reward -= maxReward;

    if (terminal) {
        reward = -1.5;
    }

    {
        boost::lock_guard<boost::mutex> lock(stateMutex);
        state = READY;
    }
    rewardReadyCondition.notify_one();
}

void PythonEnvironment::handleAppearingContact(const physics::Contact& c)
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

void PythonEnvironment::handleDissappearingContact(const physics::Contact& c)
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
