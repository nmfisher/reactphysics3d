/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef TEST_RIGIDBODY_REACTIVATION_H
#define TEST_RIGIDBODY_REACTIVATION_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestRigidBodyReactivation
/**
 * Unit test to make sure a body that has been deactivated with setIsActive(false)
 * and re-activated with setIsActive(true) is correctly woken up and simulated
 * again (a body that is put to sleep while inactive must not remain sleeping
 * once it is active again).
 */
class TestRigidBodyReactivation : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRigidBodyReactivation(const std::string& name) : Test(name) {

        }

        /// Destructor
        virtual ~TestRigidBodyReactivation() {

        }

        /// Run the tests
        void run() {
            testReactivationOfDynamicBody();
            testReactivationOfKinematicBody();
        }

        void testReactivationOfDynamicBody() {

            PhysicsWorld* world = mPhysicsCommon.createPhysicsWorld();
            world->setIsGravityEnabled(false);

            // A dynamic body with a collider
            RigidBody* dynamicBody = world->createRigidBody(Transform::identity());
            BoxShape* boxShape = mPhysicsCommon.createBoxShape(Vector3(0.5, 0.5, 0.5));
            dynamicBody->addCollider(boxShape, Transform::identity());
            dynamicBody->setLinearVelocity(Vector3(2, 0, 0));

            // Let the body move a bit and settle the world state
            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            const decimal xBeforeDeactivation = dynamicBody->getTransform().getPosition().x;
            rp3d_test(dynamicBody->isActive());
            rp3d_test(!dynamicBody->isSleeping());

            // Deactivate the body (this also puts it to sleep) and step the world
            dynamicBody->setIsActive(false);
            rp3d_test(!dynamicBody->isActive());

            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            rp3d_test(approxEqual(dynamicBody->getTransform().getPosition().x, xBeforeDeactivation));

            // Reactivate the body : it must be awake to be simulated again
            dynamicBody->setIsActive(true);
            rp3d_test(dynamicBody->isActive());
            rp3d_test(!dynamicBody->isSleeping());

            // Deactivating a body zeroes its velocity, so give it a velocity again
            dynamicBody->setLinearVelocity(Vector3(2, 0, 0));

            // The body must move again
            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            rp3d_test(dynamicBody->getTransform().getPosition().x > xBeforeDeactivation + decimal(0.1));

            mPhysicsCommon.destroyPhysicsWorld(world);
        }

        void testReactivationOfKinematicBody() {

            PhysicsWorld* world = mPhysicsCommon.createPhysicsWorld();

            // A kinematic body with a collider
            RigidBody* kinematicBody = world->createRigidBody(Transform::identity());
            kinematicBody->setType(BodyType::KINEMATIC);
            BoxShape* boxShape = mPhysicsCommon.createBoxShape(Vector3(0.5, 0.5, 0.5));
            kinematicBody->addCollider(boxShape, Transform::identity());
            kinematicBody->setLinearVelocity(Vector3(3, 0, 0));

            // Let the body move a bit
            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            const decimal xBeforeDeactivation = kinematicBody->getTransform().getPosition().x;
            rp3d_test(kinematicBody->isActive());
            rp3d_test(!kinematicBody->isSleeping());

            // Deactivate the body (this also puts it to sleep) and step the world
            kinematicBody->setIsActive(false);
            rp3d_test(!kinematicBody->isActive());

            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            rp3d_test(approxEqual(kinematicBody->getTransform().getPosition().x, xBeforeDeactivation));

            // Reactivate the body : it must be awake to be simulated again
            kinematicBody->setIsActive(true);
            rp3d_test(kinematicBody->isActive());
            rp3d_test(!kinematicBody->isSleeping());

            // Deactivating a body zeroes its velocity, so give it a velocity again
            kinematicBody->setLinearVelocity(Vector3(3, 0, 0));

            // The body must move again
            for (int i=0; i < 10; i++) {
                world->update(1.0f / 60.0f);
            }
            rp3d_test(kinematicBody->getTransform().getPosition().x > xBeforeDeactivation + decimal(0.1));

            mPhysicsCommon.destroyPhysicsWorld(world);
        }
};

}

#endif
