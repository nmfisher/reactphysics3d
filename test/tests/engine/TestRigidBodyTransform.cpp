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

#include <reactphysics3d/reactphysics3d.h>
#include "Test.h"

using namespace reactphysics3d;

class TestRigidBodyTransform : public Test {
    public:
        TestRigidBodyTransform() : Test("RigidBodyTransform") {}

        void run() override {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            world->setIsGravityEnabled(false);
            const decimal timeStep = decimal(1.0 / 60.0);
            const Vector3 velocity(2, 3, 4);
            const Vector3 angularVelocity(0, 0, 1);
            const BodyType types[] = {BodyType::DYNAMIC, BodyType::KINEMATIC};
            const Vector3 centers[] = {Vector3::zero(), Vector3(2, 1, -3)};
            const Transform transforms[] = {
                Transform(Vector3(10, 0, 0), Quaternion::identity()),
                Transform(Vector3::zero(), Quaternion::fromEulerAngles(0.3, 0.7, 0.2)),
                Transform(Vector3(-4, 7, 2), Quaternion::fromEulerAngles(0.6, -0.2, 0.9))
            };

            for (BodyType type : types) {
                for (const Vector3& center : centers) {
                    for (const Transform& transform : transforms) {
                        RigidBody* body = world->createRigidBody(Transform::identity());
                        body->setType(type);
                        body->setLocalCenterOfMass(center);
                        body->setLinearVelocity(velocity);
                        body->setAngularVelocity(angularVelocity);

                        body->setTransform(transform);
                        rp3d_test(Vector3::approxEqual(body->getLinearVelocity(), velocity));
                        rp3d_test(Vector3::approxEqual(body->getAngularVelocity(), angularVelocity));
                        rp3d_test(Vector3::approxEqual(body->getTransform().getPosition(), transform.getPosition()));

                        world->update(timeStep);
                        rp3d_test(Vector3::approxEqual(body->getLinearVelocity(), velocity));
                        // Allow rounding from the center-of-mass/world-transform round trip.
                        rp3d_test(Vector3::approxEqual(body->getTransform() * center,
                                                      transform * center + velocity * timeStep, decimal(1e-5)));
                        world->destroyRigidBody(body);
                    }
                }
            }
            common.destroyPhysicsWorld(world);
        }
};

int main() {
    TestRigidBodyTransform test;
    test.run();
    return test.report() == 0 ? 0 : 1;
}
