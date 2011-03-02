#include "Control/InverseDynamics/Control.h"
#include <slon/Graphics/Common.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawCommon.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawPhysics.h>
#include <slon/Realm/World.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#undef CreateFont

namespace {
    boost::mt19937 rng;
}

namespace ctrl {
namespace id {

Control::Control(const loose_timer_ptr& timer, bool multithreaded)
:	PhysicsControl(timer, multithreaded)
,   debugDraw(true)
,	debugDrawForceScale(3.0f)
{
}

void Control::initialize()
{
    // copy descs
    {
        rigidBodiesInitialDescs.resize( std::distance( physicsModel->firstRigidBody(), physicsModel->endRigidBody() ) );
        std::transform( physicsModel->firstRigidBody(), 
                        physicsModel->endRigidBody(), 
                        rigidBodiesInitialDescs.begin(),
                        boost::bind(&physics::RigidBody::getStateDesc, _1) );

        constraintsInitialDescs.resize( std::distance( physicsModel->firstConstraint(), physicsModel->endConstraint() ) );
        std::transform( physicsModel->firstConstraint(), 
                        physicsModel->endConstraint(), 
                        constraintsInitialDescs.begin(),
                        boost::bind(&physics::Constraint::getStateDesc, _1) );

        // create Environment
        {
            Environment::DESC                   desc;
            std::vector<physics::RigidBody*>    rigidBodies;
            std::vector<physics::Constraint*>   constraints;

            std::transform( physicsModel->firstRigidBody(),
                            physicsModel->endRigidBody(),
                            std::back_inserter(rigidBodies),
                            boost::mem_fn(&physics::rigid_body_ptr::get) );
            
            std::transform( physicsModel->firstConstraint(),
                            physicsModel->endConstraint(),
                            std::back_inserter(constraints),
                            boost::mem_fn(&physics::constraint_ptr::get) );

            desc.maxForce       = maxForce;
            desc.maxVelocity    = maxVelocity;
            desc.rigidBodies    = &rigidBodies[0];
            desc.numRigidBodies = rigidBodies.size();
            desc.constraints    = &constraints[0];
            desc.numConstraints = constraints.size();
            desc.freeJoints     = freeJoints;
        
            environment.reset(new Environment(desc));
        }
	}


    // prepare debug draw objects
    {
        debugMesh.reset(new graphics::DebugMesh);

        // create font
        debugFont.reset( graphics::currentDevice()->CreateFont() );
        {
            sgl::Image* image = graphics::currentDevice()->CreateImage();
            image->LoadFromFile("Data/Fonts/font.png");
            debugFont->SetTexture( image->CreateTexture2D() );
        }
    }

    toggleDebugDraw(true);
}

void Control::loadConfig(const std::string& fileName)
{
    assert(!acquired() && "unacquire first");

	boost::property_tree::ptree properties;
	boost::property_tree::read_ini(fileName, properties);

    freeJoints      = properties.get("FreeJoints", true);
    randomStartup   = properties.get("RandomStartup", false);
    maxForce        = properties.get("MaxForce", 20.0f);
    maxVelocity     = properties.get("MaxVelocity", 10.0f);
}

void Control::setTargetModel(const scene::node_ptr& targetModel_)
{
    assert(!acquired() && "unacquire first");
    targetModel = targetModel_;
    if (physicsModel) {
        initialize();
    }
}

void Control::setPhysicsModel(const physics::physics_model_ptr& physicsModel_)
{
    assert(!acquired() && "unacquire first");
    physicsModel = physicsModel_;
    if (targetModel) {
        initialize();
    }
}

void Control::toggleDebugDraw(bool toggle)
{
    debugDraw = toggle;
}

void Control::acquire_safe()
{
    using namespace physics;
/*
    rigid_body_desc_vector rigidBodiesDescs(rigidBodiesInitialDescs);
    constraint_desc_vector constraintsDescs(constraintsInitialDescs);

    // reset rigid bodies
    {
        rigid_body_desc_vector::iterator descIter = rigidBodiesDescs.begin();
        for (PhysicsModel::rigid_body_iterator iter  = physicsModel->firstRigidBody();
                                               iter != physicsModel->endRigidBody(); 
                                               ++iter, ++descIter)
        {
            (*iter)->reset(*descIter);
        }
    }

    // reset constraints
    {
        constraint_desc_vector::iterator descIter = constraintsDescs.begin();
        for (PhysicsModel::constraint_iterator iter  = physicsModel->firstConstraint();
                                               iter != physicsModel->endConstraint(); 
                                               ++iter, ++descIter)
        {
            (*iter)->reset(*descIter);
        }
    }

    if (randomStartup)
    {

        boost::variate_generator< boost::mt19937&, 
                                  boost::uniform_real<float> > generator(rng, boost::uniform_real<float>(0.0f, 1.0f));

        // std axes
        math::Vector3f axes[3] = 
        {
            math::Vector3f(1.0f, 0.0f, 0.0f),
            math::Vector3f(0.0f, 1.0f, 0.0f),
            math::Vector3f(0.0f, 0.0f, 1.0f),
        };

        // rotate rigid bodies around constraints
        for (size_t i = 0; i<constraintsDescs.size(); ++i)
        {
            physics::RigidBody* rbody0 = constraintsInitialDescs[i].rigidBodies[0];
            physics::RigidBody* rbody1 = constraintsInitialDescs[i].rigidBodies[1];

            // find desc by name
            physics::RigidBody::state_desc* desc0 = 0;
            physics::RigidBody::state_desc* desc1 = 0;
            for (size_t j = 0; j<rigidBodiesDescs.size(); ++j) 
            {
                if ( rigidBodiesDescs[j].name == rbody0->getName() ) {
                    desc0 = &rigidBodiesDescs[j];
                }
                else if ( rigidBodiesDescs[j].name == rbody1->getName() ) {
                    desc1 = &rigidBodiesDescs[j];
                }
            }
            assert(desc0 && desc1);

            // get transform of the constraint from bottom cone
            math::Matrix4f transform = desc0->initialTransform * constraintsDescs[i].frames[0]; 
            for (int j = 0; j<3; ++j)
            {
                float angle = constraintsDescs[i].angularLimits[0][j] + generator() * (constraintsDescs[i].angularLimits[1][j] - constraintsDescs[i].angularLimits[0][j]);
                transform  *= math::make_rotation(angle, axes[j]);
            }

            desc1->initialTransform = transform * math::invert(constraintsDescs[i].frames[1]);
        }

        // reset rigid bodies
        {
            rigid_body_desc_vector::iterator descIter = rigidBodiesDescs.begin();

            for (PhysicsModel::rigid_body_iterator iter  = physicsModel->firstRigidBody();
                                                   iter != physicsModel->endRigidBody(); 
                                                   ++iter, ++descIter)
            {
                (*iter)->reset(*descIter);
            }
        }
    }
*/
    if ( scene::Group* group = dynamic_cast<scene::Group*>(targetModel.get()) ) 
    {
        // acquire_safe also called from LearningControl, so need check to disallow child duplicates
        if (debugMesh->getParent() != group) {
            group->addChild(debugMesh.get());
        }
    }
}

void Control::unacquire_safe()
{
    if ( scene::Group* group = dynamic_cast<scene::Group*>(targetModel.get()) ) {
        group->removeChild(debugMesh.get());
    }
}

double Control::post_sync()
{
	if (debugDraw) {
		drawDebugInfo();
	}

	return -1.0;
}

void Control::drawDebugInfo()
{
	if (debugDraw) 
	{
		using namespace physics;
		using namespace graphics::debug;

		const math::Vector3f axes[] =
		{
			math::Vector3f( 0.0f, 1.0f, 0.0f),
			math::Vector3f(-1.0f, 0.0f, 0.0f)
		};

		debugMesh->clear();

		// draw rigid bodies
		for (PhysicsModel::rigid_body_iterator iter  = physicsModel->firstRigidBody();
											   iter != physicsModel->endRigidBody(); 
											   ++iter)
		{
			if ( RigidBody::AS_SLEEPING == (*iter)->getActivationState() 
				 || RigidBody::AS_DISABLE_SIMULATION == (*iter)->getActivationState() ) 
			{
				(*debugMesh) << color(0.5f, 0.5f, 0.5f, 1.0f);
			}
			else {
				(*debugMesh) << color(1.0f, 0.0f, 0.0f, 1.0f);
			}

			(*debugMesh) << transform( math::Matrix4f((*iter)->getTransform()) )
						 << static_cast<const physics::ConeShape&>( *(*iter)->getCollisionShape() );
		}

		// draw forces
		(*debugMesh) << color(1.0f, 0.0f, 0.0f, 1.0f);
		for (PhysicsModel::constraint_iterator iter  = physicsModel->firstConstraint();
											   iter != physicsModel->endConstraint(); 
											   ++iter)
		{
			const physics::Constraint& constraint = (**iter);

			math::Vector3f force(0.0f, 0.0f, 0.0f);
			for (int i = 0; i<2; ++i)
			{
                const physics::Motor* motor = constraint.getMotor( physics::Motor::TYPE(physics::Motor::MOTOR_X_ROT + i) );
				if (motor) {
					force += axes[i] * (float)motor->getForce();
				}
			}

			force *= debugDrawForceScale;
			force /= environment->getMaxVelocity();

			math::Vector3f origin0( math::get_translation( constraint.getRigidBodyA()->getTransform() * constraint.getStateDesc().frames[0] ) );
			math::Vector3f origin1( math::get_translation( constraint.getRigidBodyA()->getTransform() ) );
			math::Vector3f origin2( math::get_translation( constraint.getRigidBodyB()->getTransform() ) );
			math::Vector3f tip = math::xyz( math::Matrix4f(constraint.getRigidBodyA()->getTransform() * constraint.getStateDesc().frames[0]) * math::make_vec(force, 1.0f) );
			(*debugMesh) << transform( math::make_identity<float, 4>() )
						 << line(origin0, tip)
						 << line(origin1, tip) 
						 << line(origin2, tip)
						 << line(origin0, origin1)
						 << line(origin0, origin2);
		}
	}
}

} // namespace id
} // namespace ctrl
