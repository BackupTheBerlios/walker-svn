#ifndef __WALKER_YARD_ACTUATOR_WORLD_VECTOR_ACTUATOR_H__
#define __WALKER_YARD_ACTUATOR_WORLD_VECTOR_ACTUATOR_H__

#include <boost/shared_ptr.hpp>
#include <slon/Physics/Constraint.h>
#include <slon/Physics/RigidBody.h>

using namespace slon;

class JointActuator
{
public:
    JointActuator( physics::Constraint&  _constraint,
                   const math::Vector3f& _normal );

    void setTorque(const math::Vector3f& torque);

    void bend(float angle, float torque);

    void update();

private:
    physics::constraint_ptr     constraint;
    math::Vector3f              normal;

    // dynamic
    math::Vector3f              bendForce;
};

typedef boost::shared_ptr<JointActuator>    joint_actuator_ptr;

#endif  // __WALKER_YARD_ACTUATOR_WORLD_VECTOR_ACTUATOR_H__
