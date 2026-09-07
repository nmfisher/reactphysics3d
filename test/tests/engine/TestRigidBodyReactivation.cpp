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

class TestRigidBodyReactivation : public Test {
    public:
        TestRigidBodyReactivation() : Test("RigidBodyReactivation") {}

        void run() override {
            PhysicsCommon common;
            SphereShape* shape = common.createSphereShape(1);
            const decimal timeStep = decimal(1.0 / 60.0);
            const BodyType types[] = {BodyType::DYNAMIC, BodyType::KINEMATIC, BodyType::STATIC};

            for (BodyType type : types) {
                for (bool addWhileInactive : {false, true}) {
                    PhysicsWorld* world = common.createPhysicsWorld();
                    RigidBody* body = world->createRigidBody(Transform::identity());
                    body->setType(type);
                    if (!addWhileInactive) body->addCollider(shape, Transform::identity());
                    body->setIsActive(false);
                    if (addWhileInactive) body->addCollider(shape, Transform::identity());

                    // Repeated activation must resume simulation without an explicit wake-up.
                    for (int cycle = 0; cycle < 2; cycle++) {
                        const Vector3 position = body->getTransform().getPosition();
                        world->update(timeStep);
                        rp3d_test(!body->isActive());
                        rp3d_test(Vector3::approxEqual(body->getTransform().getPosition(), position));

                        body->setIsActive(true);
                        body->setIsActive(true);
                        rp3d_test(body->isActive());
                        rp3d_test(!body->isSleeping());
                        if (type == BodyType::KINEMATIC) body->setLinearVelocity(Vector3(1, 0, 0));
                        world->update(timeStep);

                        if (type == BodyType::DYNAMIC) {
                            rp3d_test(body->getLinearVelocity().y < 0);
                            rp3d_test(body->getTransform().getPosition().y < position.y);
                        }
                        else if (type == BodyType::KINEMATIC) {
                            rp3d_test(body->getTransform().getPosition().x > position.x);
                        }
                        else {
                            rp3d_test(Vector3::approxEqual(body->getTransform().getPosition(), position));
                        }
                        body->setIsActive(false);
                    }
                    common.destroyPhysicsWorld(world);
                }
            }
            common.destroySphereShape(shape);
        }
};

int main() {
    TestRigidBodyReactivation test;
    test.run();
    return test.report() == 0 ? 0 : 1;
}
