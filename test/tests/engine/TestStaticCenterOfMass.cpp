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

class TestStaticCenterOfMass : public Test {
    public:
        TestStaticCenterOfMass() : Test("StaticCenterOfMass") {}

        void run() override {
            PhysicsCommon common;
            SphereShape* shape = common.createSphereShape(1);
            const Vector3 center(2, 1, -3);
            const Transform pose(Vector3(5, 2, 3), Quaternion::fromEulerAngles(0.3, 0.7, 0.2));

            // Cover every way to change a static body's world center of mass.
            for (int mode = 0; mode < 5; mode++) {
                PhysicsWorld* world = common.createPhysicsWorld();
                world->setIsGravityEnabled(false);
                RigidBody* anchor = world->createRigidBody(pose);
                if (mode < 2) anchor->setLocalCenterOfMass(center);
                anchor->setType(BodyType::STATIC);

                if (mode == 1) {
                    anchor->setTransform(Transform(Vector3(-7, 3, 4), Quaternion::fromEulerAngles(0.1, 0.4, 0.8)));
                }
                else if (mode == 2) {
                    anchor->setLocalCenterOfMass(center);
                }
                else if (mode >= 3) {
                    anchor->addCollider(shape, Transform(center, Quaternion::identity()));
                    if (mode == 3) anchor->updateLocalCenterOfMassFromColliders();
                    else anchor->updateMassPropertiesFromColliders();
                }
                rp3d_test(Vector3::approxEqual(anchor->getLocalCenterOfMass(), center));

                const Transform expectedPose = anchor->getTransform();
                RigidBody* body = world->createRigidBody(expectedPose);
                FixedJointInfo info(anchor, body, expectedPose.getPosition());
                world->createJoint(info);

                for (int step = 0; step < 3; step++) {
                    world->update(decimal(1.0 / 60.0));
                    rp3d_test(Vector3::approxEqual(body->getTransform().getPosition(), expectedPose.getPosition(), decimal(1e-5)));
                    rp3d_test(Vector3::approxEqual(body->getLinearVelocity(), Vector3::zero()));
                    rp3d_test(Vector3::approxEqual(anchor->getTransform().getPosition(), expectedPose.getPosition(), decimal(1e-5)));
                }
                common.destroyPhysicsWorld(world);
            }
            common.destroySphereShape(shape);
        }
};

int main() {
    TestStaticCenterOfMass test;
    test.run();
    return test.report() == 0 ? 0 : 1;
}
