#include "Control/Chain/PDControl.h"
#include <boost/numeric/ublas/io.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <slon/log/Logger.h>
#include <slon/Physics/PhysicsManager.h>

__DEFINE_LOGGER__("ctrl.chain.PDControl")

namespace ctrl {
namespace chain {

using namespace boost::numeric::ublas;

PDControl::PDControl(const loose_timer_ptr& timer)
:	Control(timer, false)
,   needRestart(false)
,	lastUpdate(0.0)
,	timeInterval(0.005)
{
}

PDControl::~PDControl()
{
    if ( acquired() ) {
        unacquire();
    }
}

void PDControl::loadConfig(const std::string& configFile)
{
    Control::loadConfig(configFile);

	boost::property_tree::ptree properties;
	boost::property_tree::read_ini(configFile, properties);
	/*
	dumpFuncsOnExit     = properties.get("DumpFuncsOnExit", false);
	loadFuncsOnStartup  = properties.get("LoadFuncsOnStartup", false);
	scalarFunctionDump  = properties.get("ScalarFunctionDump", "Data/Dump/scalar_function_dump.txt");
	vectorFunctionDump  = properties.get("VectorFunctionDump", "Data/Dump/vector_function_dump.txt");
	autosave            = properties.get("Autosave", false);
	autosaveTime        = properties.get("AutosaveTime", 60.0);
    episodic            = properties.get("Episodic", false);
    episodeLength       = properties.get("EpisodeLength", 0.0);
    rlType              = getRLType( properties.get("Learner", "DirectRL") );
    */
    timeInterval        = properties.get("TimeInterval", 0.03f);
    pdConfig            = properties.get("PDConfig", "");

    std::ifstream pdFile( pdConfig.c_str() );
    if ( pdFile.is_open() ) {
        pdFile >> Kp >> Kd;
    }
}

void PDControl::acquire_safe()
{
    using namespace physics;

	Control::acquire_safe();
	needRestart = true;

    environment->setControlType(ctrl::PhysicsEnvironment::CONTROL_FORCE);

    if ( Kp.size1() != environment->getActionSize() || Kp.size2() != environment->getActionSize() )
    {
        logger << slog::S_WARNING << "Kp size loaded from config differs from requested by model, reset" << LOG_FILE_AND_LINE;
        Kp = ublas::identity_matrix<float>( environment->getActionSize(), environment->getActionSize() );
    }

    if ( Kd.size1() != environment->getActionSize() || Kd.size2() != environment->getActionSize() ) 
    {
        logger << slog::S_WARNING << "Kd size loaded from config differs from requested by model, reset" << LOG_FILE_AND_LINE;
        Kd = ublas::identity_matrix<float>( environment->getActionSize(), environment->getActionSize() );
    }

    targetVelocity = ublas::zero_vector<physics::real>( environment->getActionSize() );
    targetPosition = ublas::zero_vector<physics::real>( environment->getActionSize() );

    // compute mass for each joint
    size_t index = 0;
    mass = ublas::zero_vector<physics::real>( environment->getActionSize() );
    for (PhysicsModel::constraint_iterator iter  = physicsModel->firstConstraint();
                                           iter != physicsModel->endConstraint(); 
                                           ++iter)
    {
        for (int i = 0; i<3; ++i)
        {
            if ( (*iter)->getMotor( Motor::TYPE(Motor::MOTOR_X_ROT + i) ) ) {
                mass[index++] = (*iter)->getRigidBodyB()->getMass();
            }
        }
    }
}

void PDControl::unacquire_safe()
{
	Control::unacquire_safe();
    /*
	while (!complete)
	{
		double dt = pre_sync();
		if (dt > 0.0) {
			timer->setTime(timer->getTime() + dt);
		}
	}
    */
}

double PDControl::pre_sync()
{
    if (needRestart)
    {
        acquire_safe();
        environment->reset();
        needRestart = false;
        return physics::currentPhysicsManager().getDynamicsWorld()->getStateDesc().fixedTimeStep + math::EPS_6f;
    }

    environment->makeState();

	// calculate pd term
    environment->targetForce = ublas::prod(Kd, targetVelocity - environment->velocity) + ublas::prod(Kp, targetPosition - environment->position);
    if (gravityCompensation) {
        environment->targetForce += getGravityCompensation();
    }
    environment->makeAction();
    
    return timeInterval;
}

void PDControl::drawDebugInfo()
{
    if (debugDraw) {
        Control::drawDebugInfo();
	}
}

} // namespace chain
} // namespace ctrl
