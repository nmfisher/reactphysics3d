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

class FilterEventListener : public EventListener {
    public:
        uint32 exits = 0;
        uint32 contacts = 0;

        void onContact(const CollisionCallback::CallbackData& data) override {
            for (uint32 i = 0; i < data.getNbContactPairs(); i++) {
                if (data.getContactPair(i).getEventType() == CollisionCallback::ContactPair::EventType::ContactExit) exits++;
                else contacts++;
            }
        }

        void onTrigger(const OverlapCallback::CallbackData& data) override {
            for (uint32 i = 0; i < data.getNbOverlappingPairs(); i++) {
                if (data.getOverlappingPair(i).getEventType() == OverlapCallback::OverlapPair::EventType::OverlapExit) exits++;
                else contacts++;
            }
        }

        void reset() { exits = 0; contacts = 0; }
};

class TestCollisionRefiltering : public Test {
    public:
        TestCollisionRefiltering() : Test("CollisionRefiltering") {}

        void run() override {
            PhysicsCommon common;
            SphereShape* sphere = common.createSphereShape(1);
            float heights[9] = {};
            std::vector<Message> messages;
            HeightField* heightField = common.createHeightField(3, 3, heights,
                HeightField::HeightDataType::HEIGHT_FLOAT_TYPE, messages);
            rp3d_test(heightField != nullptr);
            if (heightField == nullptr) return;
            HeightFieldShape* terrain = common.createHeightFieldShape(heightField);
            CollisionShape* shapes[] = {sphere, terrain};
            const decimal timeStep = decimal(1.0 / 60.0);

            for (CollisionShape* shape : shapes) {
                for (bool changeCategory : {false, true}) {
                    for (bool sleeping : {false, true}) {
                        for (bool trigger : {false, true}) {
                            FilterEventListener listener;
                            PhysicsWorld* world = common.createPhysicsWorld();
                            world->setIsGravityEnabled(false);
                            world->setEventListener(&listener);
                            RigidBody* fixed = world->createRigidBody(Transform::identity());
                            fixed->setType(BodyType::STATIC);
                            Collider* fixedCollider = fixed->addCollider(shape, Transform::identity());
                            fixedCollider->setCollideWithMaskBits(1);
                            RigidBody* body = world->createRigidBody(Transform(Vector3(0, 0.5, 0), Quaternion::identity()));
                            Collider* collider = body->addCollider(sphere, Transform::identity());
                            collider->setIsTrigger(trigger);
                            world->update(timeStep);
                            rp3d_test(listener.contacts == 1);
                            rp3d_test(world->testOverlap(fixed, body));

                            // A filter edit that still permits this pair must preserve contact.
                            listener.reset();
                            collider->setCollisionCategoryBits(3);
                            collider->setCollideWithMaskBits(1);
                            world->update(timeStep);
                            rp3d_test(listener.exits == 0);
                            rp3d_test(listener.contacts == 1);
                            if (sleeping) body->setIsSleeping(true);

                            listener.reset();
                            if (changeCategory) collider->setCollisionCategoryBits(2);
                            else collider->setCollideWithMaskBits(0);
                            const Vector3 position = body->getTransform().getPosition();
                            world->update(timeStep);
                            rp3d_test(listener.exits == 1);
                            rp3d_test(listener.contacts == 0);
                            rp3d_test(!world->testOverlap(fixed, body));
                            rp3d_test(Vector3::approxEqual(body->getTransform().getPosition(), position));

                            // An invalid pair should emit exactly one exit event.
                            listener.reset();
                            world->update(timeStep);
                            rp3d_test(listener.exits == 0);
                            rp3d_test(listener.contacts == 0);

                            // Restoring the filter must rediscover the pair without moving it.
                            collider->setCollisionCategoryBits(1);
                            collider->setCollideWithMaskBits(1);
                            body->setIsSleeping(false);
                            rp3d_test(world->testOverlap(fixed, body));
                            listener.reset();
                            world->update(timeStep);
                            rp3d_test(listener.contacts == 1);
                            rp3d_test(listener.exits == 0);
                            common.destroyPhysicsWorld(world);
                        }
                    }
                }
            }
            common.destroyHeightFieldShape(terrain);
            common.destroyHeightField(heightField);
            common.destroySphereShape(sphere);
        }
};

int main() {
    TestCollisionRefiltering test;
    test.run();
    return test.report() == 0 ? 0 : 1;
}
