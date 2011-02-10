#ifndef __WALKER_YARD_CONTROL_CENTRAL_H__
#define __WALKER_YARD_CONTROL_CENTRAL_H__

#include <boost/intrusive_ptr.hpp>
#include <slon/Scene/Node.h>
#include <string>
#include "SlonEngine.h"

namespace ctrl {

class Control :
    public slon::Referenced
{
public:
	/** Load control config */
	virtual void loadConfig(const std::string& fileName) = 0;

    /** Setup model for control */
    virtual void setTargetModel(const scene::node_ptr& graphicsModel) = 0;

    /** Gain control of target model. */
    virtual void acquire() = 0;

    /** Free control of target model. */
    virtual void unacquire() = 0;

    virtual ~Control() {}
};

typedef boost::intrusive_ptr<Control>		control_ptr;
typedef boost::intrusive_ptr<const Control>	const_control_ptr;

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_CENTRAL_H__
