#ifndef __WALKER_YARD_ACTUATOR_EULER_ACTUATOR_H__
#define __WALKER_YARD_ACTUATOR_EULER_ACTUATOR_H__

#include <SlonEngine/Physics/Constraint.h>
#include <SlonEngine/Physics/RigidBody.h>

using namespace slon;

class EulerActuator
{
public:
    EulerActuator(physics::Constraint *_constraint, physics::RigidBody *_rigidBody);
    void bend(double angle, double force);
    virtual ~EulerActuator();
private:
    physics::Constraint *constraint;
    physics::RigidBody *rigidBody;
};

#endif  // __WALKER_YARD_ACTUATOR_EULER_ACTUATOR_H__