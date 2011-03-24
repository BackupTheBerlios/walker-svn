#ifndef __WALKER_YARD_UTILITY_PHYSICS_DEBUG_H__
#define __WALKER_YARD_UTILITY_PHYSICS_DEBUG_H__

#include <slon/Scene/Visitors/NodeVisitor.h>

using namespace slon;

class PhysicsDebugVisitor :
    public scene::NodeVisitor
{
public:
    typedef scene::NodeVisitor base_type;

public:
    void visitTransform(scene::Transform& transform);
};

#endif // __WALKER_YARD_UTILITY_PHYSICS_DEBUG_H__