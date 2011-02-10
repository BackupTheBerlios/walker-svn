#ifndef __WALKER_YARD_CONTROL_GENERIC_ACTION_MAP_H__
#define __WALKER_YARD_CONTROL_GENERIC_ACTION_MAP_H__

#include <boost/shared_ptr.hpp>

namespace ctrl {

class GenericActionMap
{
public:
    typedef boost::shared_array<float>  float_array;

    struct engine
    {
        void_function   engineFunction;
        size_t          numPins;
        float_array     pins;
    };

    typedef std::vector<engine_ptr>     engine_vector;

public:
    template<typename engine_iterator>
    GenericActionMap(const engine_iterator& firstEngine, const engine_iterator& endEngine)
    {
        for ( engine_iterator engineIt = firstEngine; 
                              engineIt != endEngine; 
                              ++engineIt )
        {
            switch ( engineIt->getType() )
            {
            case JOINT_ACTUATOR:
                add_engine( boost::shared_static_cast<JointActuator>(engineIt) );
                break;

            default:
                assert(!"Unknown engine");
                break;
            }
        }
    }

    // Override ActionMap
    size_t  getNumEngines() const = 0;
    size_t  getNumPins(size_t engine) const = 0;
    void    toggleEngine(size_t engine, const float* firstPin, const float* endPin) = 0;

private:
    void    add_engine(const joint_actuator_ptr& jointActuator);    
};

typedef boost::shared_ptr<ActionMap>    action_map_ptr;
    
} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_GENERIC_ACTION_MAP_H__
