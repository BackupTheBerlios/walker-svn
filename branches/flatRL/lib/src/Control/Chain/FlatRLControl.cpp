#include "Control/Chain/FlatRLControl.h"
#include <boost/numeric/ublas/io.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <slon/log/Logger.h>
#include <slon/Physics/PhysicsManager.h>

__DEFINE_LOGGER__("ctrl.chain.FlatRLControl")

namespace ctrl {
namespace chain {

using namespace boost::numeric::ublas;

FlatRLControl::FlatRLControl(const loose_timer_ptr& timer)
:	Control(timer, false)
,   needRestart(false)
,	lastUpdate(0.0)
,	timeInterval(0.005)
{
}

FlatRLControl::~FlatRLControl()
{
    if ( acquired() ) {
        unacquire();
    }
}
void FlatRLControl::dumpFunctions()
{
	learn::io::serialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::serialize(scalarFunctionDump.c_str(), scalarFunction);
}

void FlatRLControl::loadFunctions()
{
	learn::io::deserialize(vectorFunctionDump.c_str(), vectorFunction);
	learn::io::deserialize(scalarFunctionDump.c_str(), scalarFunction);
}

void FlatRLControl::initFunctions(radial_basis_function_type&    scalarFunction,
                                 htangent_neural_network_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    boost::variate_generator< boost::mt19937&, 
                              boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(-1.0f, 1.0f));

#ifdef DEBUG
    const size_t numSamples = 2500;
#else
    const size_t numSamples = 2500;
#endif

    matrix<float> sample(numSamples, stateSize);
    std::generate(sample.begin1(), sample.end1(), generator);

    scalarFunction = FlatRLControl::radial_basis_function_type( stateSize, 
                                                               sample.size1() * sample.size2(), 
                                                               0.34f, 
                                                               -10000, 
                                                               10000, 
                                                               sample );

    learn::make_cascade_neural_network(vectorFunction, stateSize, actionSize, 2, true);
    learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>(-0.2f, 0.2f, 0.1f) );
}

void FlatRLControl::initFunctions(scalar_neural_network_type&    scalarFunction,
                                 htangent_neural_network_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    generic_neural_network_type network;
    learn::make_cascade_neural_network(network, stateSize, 1, 3, true);
    learn::distribute_weights( network, learn::uniform_weight_distributor<float>() );

    // fill up functions
    learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 3, make_generic_function_with_derivative( learn::linear<float>() ) );
    
    scalarFunction = scalar_neural_network_type(network);
    learn::make_cascade_neural_network(vectorFunction, stateSize, actionSize, 3, true);
    learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>() );
}

void FlatRLControl::initFunctions(scalar_neural_network_type&    scalarFunction,
                                 vector_neural_network_function_type&  vectorFunction,
                                 size_t                         stateSize,
                                 size_t                         actionSize)
{
    generic_neural_network_type network;
    learn::make_cascade_neural_network(network, stateSize, 1, 3, true);
    learn::distribute_weights( network, learn::uniform_weight_distributor<float>() );

    // fill up functions
    learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    learn::setup_function( network, 3, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
    
    scalarFunction = scalar_neural_network_type(network);
    vectorFunction = vector_neural_network_function_type(actionSize, scalarFunction);
}

void FlatRLControl::initFunctions(radial_basis_function_type&        scalarFunction,
                                 vector_radial_basis_function_type& vectorFunction,
                                 size_t                             stateSize,
                                 size_t                             actionSize)
{
    boost::variate_generator< boost::mt19937&, 
                              boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(-1.0f, 1.0f));

#ifdef DEBUG
    const size_t numSamples = 2500;
#else
    const size_t numSamples = 2500;
#endif

    matrix<float> sample(numSamples, stateSize);
    std::generate(sample.begin1(), sample.end1(), generator);
    /*
    std::vector< std::vector<float> > sample;
    //for (int i = 0; i<numSamples; ++i) {
    //    std::generate(sample[i].begin(), sample[i].end(), generator);
    //}
    for (float x1 = -1.4f; x1 < 1.4f; x1 += 0.2f)
    {
        for (float x2 = -1.4f; x2 < 1.4f; x2 += 0.2f)
        {
           // for (float x3 = -1.1f; x3 < 1.1f; x3 += 0.3f)
            {
                //for (float x4 = -1.1f; x4 < 1.1f; x4 += 0.3f)
                {
                    sample.push_back(std::vector<float>(stateSize));
                    sample.back()[0] = x1;
                    sample.back()[1] = x2;
                    //sample.back()[2] = x3;
                    //sample.back()[3] = x4;
                }
            }
        }
    }
    */

    scalarFunction = radial_basis_function_type(stateSize, sample.size1() * sample.size2(), 0.1f, -10000, 10000, sample);
    vectorFunction = vector_radial_basis_function_type(actionSize, learn::radial_basis_function<float>( stateSize, 
                                                                                                        sample.size1() * sample.size2(), 
                                                                                                        0.1f, 
                                                                                                        -1.0f, 
                                                                                                        1.0f, 
                                                                                                        sample ));
}

void FlatRLControl::initFunctions(linear_radial_basis_function_type&         scalarFunction,
                                 vector_linear_radial_basis_function_type&  vectorFunction,
                                 size_t                                     stateSize,
                                 size_t                                     actionSize)
{
    scalarFunction = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 

    linear_radial_basis_function_type lin_func = learn::make_linear_rbf(stateSize, -10000.0f, 1000.0f); 
    vectorFunction = vector_linear_radial_basis_function_type(actionSize, lin_func);
}

void FlatRLControl::loadConfig(const std::string& configFile)
{
    Control::loadConfig(configFile);

	boost::property_tree::ptree properties;
	boost::property_tree::read_ini(configFile, properties);
	
	dumpFuncsOnExit     = properties.get("DumpFuncsOnExit", false);
	loadFuncsOnStartup  = properties.get("LoadFuncsOnStartup", false);
	scalarFunctionDump  = properties.get("ScalarFunctionDump", "Data/Dump/scalar_function_dump.txt");
	vectorFunctionDump  = properties.get("VectorFunctionDump", "Data/Dump/vector_function_dump.txt");
	autosave            = properties.get("Autosave", false);
	autosaveTime        = properties.get("AutosaveTime", 60.0);
    episodic            = properties.get("Episodic", false);
    episodeLength       = properties.get("EpisodeLength", 0.0);
    controlType         = getControlType( properties.get("ControlType", "VELOCITY") );
    rlType              = getRLType( properties.get("Learner", "DirectRL") );
    rlConfig            = properties.get("LearnerConfig", "");
}

void FlatRLControl::loadConfig(const std::string& configFile)
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
    //pdConfig            = properties.get("PDConfig", "");

    dumpFuncsOnExit     = properties.get("DumpFuncsOnExit", false);
	loadFuncsOnStartup  = properties.get("LoadFuncsOnStartup", false);
	scalarFunctionDump  = properties.get("ScalarFunctionDump", "Data/Dump/scalar_function_dump.txt");
	vectorFunctionDump  = properties.get("VectorFunctionDump", "Data/Dump/vector_function_dump.txt");
	autosave            = properties.get("Autosave", false);
	autosaveTime        = properties.get("AutosaveTime", 60.0);
    episodic            = properties.get("Episodic", false);
    episodeLength       = properties.get("EpisodeLength", 0.0);
    controlType         = getControlType( properties.get("ControlType", "VELOCITY") );
    rlType              = getRLType( properties.get("Learner", "DirectRL") );
    rlConfig            = properties.get("LearnerConfig", "");

    std::ifstream pdFile( pdConfig.c_str() );
    if ( pdFile.is_open() ) {
        pdFile >> Kp >> Kd;
    }
}

void FlatRLControl::acquire_safe()
{
    using namespace physics;

	Control::acquire_safe();
	needRestart = true;

    environment->setControlType(controlType);
    environment->reset();

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

void FlatRLControl::unacquire_safe()
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

double FlatRLControl::pre_sync()
{
    environment->makeState();

	// calculate pd term
    environment->targetForce = ublas::prod(Kd, targetVelocity - environment->velocity) + ublas::prod(Kp, targetPosition - environment->position) +
    chainLearner->update(    ;
    
    if (gravityCompensation) {
        environment->targetForce += getGravityCompensation();
    }
    environment->makeAction();
    
    return timeInterval;
}

void FlatRLControl::drawDebugInfo()
{
    if (debugDraw) {
        Control::drawDebugInfo();
	}
}

} // namespace chain
} // namespace ctrl
