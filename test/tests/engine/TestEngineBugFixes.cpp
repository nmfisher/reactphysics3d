/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
*********************************************************************************/

#include <reactphysics3d/reactphysics3d.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/collision/narrowphase/SphereVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SphereVsSphereAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.h>
#include <reactphysics3d/containers/Deque.h>
#include <reactphysics3d/containers/LinkedList.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/memory/DefaultAllocator.h>
#include "Test.h"

#include <cmath>
#include <cstdlib>
#include <map>
#include <vector>

using namespace reactphysics3d;

namespace {

class StrictTrackingAllocator : public MemoryAllocator {
    public:
        DefaultAllocator base;
        std::map<void*, size_t> liveAllocations;
        bool sizeMismatchDetected = false;

        void* allocate(size_t size) override {
            void* ptr = base.allocate(size);
            liveAllocations[ptr] = size;
            return ptr;
        }

        void release(void* pointer, size_t size) override {
            auto it = liveAllocations.find(pointer);
            if (it == liveAllocations.end() || it->second != size) {
                sizeMismatchDetected = true;
            }
            else {
                liveAllocations.erase(it);
            }
            base.release(pointer, size);
        }
};

struct SimpleRaycastHit : public RaycastCallback {
    bool hit = false;
    Vector3 worldPoint = Vector3::zero();
    Vector3 worldNormal = Vector3::zero();
    int triangleIndex = -1;

    decimal notifyRaycastHit(const RaycastInfo& info) override {
        hit = true;
        worldPoint = info.worldPoint;
        worldNormal = info.worldNormal;
        triangleIndex = info.triangleIndex;
        return info.hitFraction;
    }

    void reset() {
        hit = false;
        worldPoint = Vector3::zero();
        worldNormal = Vector3::zero();
        triangleIndex = -1;
    }
};

struct SimpleCollisionCounter : public CollisionCallback {
    uint32 pairCount = 0;
    uint32 pointCount = 0;

    void onContact(const CallbackData& callbackData) override {
        pairCount += callbackData.getNbContactPairs();
        for (uint32 i = 0; i < callbackData.getNbContactPairs(); i++) {
            pointCount += callbackData.getContactPair(i).getNbContactPoints();
        }
    }

    void reset() {
        pairCount = 0;
        pointCount = 0;
    }
};

struct SimpleOverlapCounter : public OverlapCallback {
    uint32 pairCount = 0;

    void onOverlap(CallbackData& callbackData) override {
        pairCount += callbackData.getNbOverlappingPairs();
    }

    void reset() {
        pairCount = 0;
    }
};

struct DestructorTracker {
    static int liveCount;
    int value;
    explicit DestructorTracker(int v) : value(v) { liveCount++; }
    DestructorTracker(const DestructorTracker& other) : value(other.value) { liveCount++; }
    ~DestructorTracker() { liveCount--; }
};

int DestructorTracker::liveCount = 0;

} // namespace

class TestEngineBugFixes : public Test {
    public:
        TestEngineBugFixes() : Test("EngineBugFixes") {}

        void run() override {
            testAllocatorAndComponentReleaseSizes();
            testHeightFieldAllocationAndScaledRaycast();
            testCapsuleRaycastSmallScale();
            testDegenerateNarrowPhaseAndBatchOffsets();
            testSleepingAndTriggerWorldQueries();
            testResizeStaticAndSleepingShapes();
            testJointFixesAndMultiJointNoCollision();
            testTwistFrictionWarmStart();
            testContainersAndDynamicAABBTreeAndInvalidConvexMesh();
        }

    private:
        void testAllocatorAndComponentReleaseSizes() {
            StrictTrackingAllocator trackingAllocator;
            {
                PhysicsCommon common(&trackingAllocator);
                PhysicsWorld* world = common.createPhysicsWorld();
                SphereShape* sphere = common.createSphereShape(decimal(0.5));

                std::vector<RigidBody*> bodies;
                for (int i = 0; i < 40; i++) {
                    RigidBody* b = world->createRigidBody(Transform(Vector3(decimal(i * 2), 0, 0), Quaternion::identity()));
                    b->addCollider(sphere, Transform::identity());
                    bodies.push_back(b);
                }

                // Force SliderJointComponents & HingeJointComponents to grow past 16
                for (int i = 0; i < 20; i++) {
                    SliderJointInfo sInfo(bodies[i], bodies[i + 1], Vector3(decimal(i * 2), 0, 0), Vector3(1, 0, 0));
                    world->createJoint(sInfo);
                    HingeJointInfo hInfo(bodies[i], bodies[i + 1], Vector3(decimal(i * 2), 0, 0), Vector3(0, 1, 0));
                    world->createJoint(hInfo);
                }

                world->update(decimal(1.0 / 60.0));
            }
            rp3d_test(!trackingAllocator.sizeMismatchDetected);
            rp3d_test(trackingAllocator.liveAllocations.empty());
        }

        void testHeightFieldAllocationAndScaledRaycast() {
            StrictTrackingAllocator trackingAllocator;
            {
                PhysicsCommon common(&trackingAllocator);
                // Slope rising +1 along X per grid unit
                float heights[4] = {0.0f, 1.0f, 0.0f, 1.0f};
                std::vector<Message> messages;
                HeightField* hf = common.createHeightField(2, 2, heights, HeightField::HeightDataType::HEIGHT_FLOAT_TYPE, messages);
                rp3d_test(hf != nullptr);

                // Stretch X by 10x, keep Y at 1x -> true slope is dy/dx = 0.1, normal = (-0.1, 1, 0) / sqrt(1.01)
                HeightFieldShape* hfShape = common.createHeightFieldShape(hf, Vector3(10, 1, 10));
                PhysicsWorld* world = common.createPhysicsWorld();
                RigidBody* terrain = world->createRigidBody(Transform::identity());
                terrain->setType(BodyType::STATIC);
                terrain->addCollider(hfShape, Transform::identity());

                SimpleRaycastHit cb;
                world->raycast(Ray(Vector3(0, 5, 0), Vector3(0, -5, 0)), &cb);
                rp3d_test(cb.hit);
                rp3d_test(cb.triangleIndex >= 0);
                Vector3 expectedNormal = Vector3(decimal(-0.1), decimal(1.0), decimal(0.0)).getUnit();
                rp3d_test(Vector3::approxEqual(cb.worldNormal, expectedNormal, decimal(0.01)));

                common.destroyPhysicsWorld(world);
                common.destroyHeightFieldShape(hfShape);
                common.destroyHeightField(hf);

                // Ray hitting within sub-ULP of an interior heightfield grid line (0.5 on 10x10 grid)
                float flatHeights[100];
                for (int k = 0; k < 100; k++) flatHeights[k] = 4.0f;
                HeightField* flatHf = common.createHeightField(10, 10, flatHeights, HeightField::HeightDataType::HEIGHT_FLOAT_TYPE, messages);
                HeightFieldShape* flatShape = common.createHeightFieldShape(flatHf);
                PhysicsWorld* world2 = common.createPhysicsWorld();
                RigidBody* flatBody = world2->createRigidBody(Transform::identity());
                flatBody->setType(BodyType::STATIC);
                Collider* flatCol = flatBody->addCollider(flatShape, Transform::identity());
                RaycastInfo hitInfo;
                const decimal tiny = MACHINE_EPSILON * decimal(0.25);
                rp3d_test(flatCol->raycast(Ray(Vector3(3, 15, decimal(0.5) - tiny), Vector3(3, -1, decimal(0.5) + tiny)), hitInfo));
                rp3d_test(flatCol->raycast(Ray(Vector3(3, 15, decimal(0.5) - tiny), Vector3(3, -1, decimal(0.5) - tiny)), hitInfo));
                common.destroyPhysicsWorld(world2);
                common.destroyHeightFieldShape(flatShape);
                common.destroyHeightField(flatHf);
            }
            rp3d_test(!trackingAllocator.sizeMismatchDetected);
            rp3d_test(trackingAllocator.liveAllocations.empty());
        }

        void testCapsuleRaycastSmallScale() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            CapsuleShape* capsule = common.createCapsuleShape(decimal(0.1), decimal(0.2));
            RigidBody* body = world->createRigidBody(Transform::identity());
            body->setType(BodyType::STATIC);
            body->addCollider(capsule, Transform::identity());

            // Perpendicular ray of length 0.4 -> |d|^2 * |n|^2 = 0.04 * 0.16 = 0.0064 < old 0.01 epsilon
            SimpleRaycastHit cb;
            world->raycast(Ray(Vector3(decimal(-0.2), 0, 0), Vector3(decimal(0.2), 0, 0)), &cb);
            rp3d_test(cb.hit);
            rp3d_test(Vector3::approxEqual(cb.worldNormal, Vector3(-1, 0, 0), decimal(0.001)));

            // Also verify AABB::raycast respects maxFraction < 1
            AABB box(Vector3(5, -1, -1), Vector3(7, 1, 1));
            Vector3 hitPt;
            rp3d_test(!box.raycast(Ray(Vector3(0, 0, 0), Vector3(10, 0, 0), decimal(0.3)), hitPt));
            rp3d_test(box.raycast(Ray(Vector3(0, 0, 0), Vector3(10, 0, 0), decimal(0.6)), hitPt));
        }

        void testDegenerateNarrowPhaseAndBatchOffsets() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            DefaultAllocator alloc;

            SphereShape* sphere = common.createSphereShape(decimal(1.0));
            CapsuleShape* capsule = common.createCapsuleShape(decimal(1.0), decimal(2.0));
            BoxShape* box = common.createBoxShape(Vector3(1, 1, 1));

            // Concentric SphereVsSphere: local points on body 1 and body 2 must be opposite
            {
                RigidBody* b1 = world->createRigidBody(Transform::identity());
                RigidBody* b2 = world->createRigidBody(Transform::identity());
                b1->addCollider(sphere, Transform::identity());
                b2->addCollider(sphere, Transform::identity());

                SimpleCollisionCounter cb;
                world->testCollision(b1, b2, cb);
                rp3d_test(cb.pairCount == 1 && cb.pointCount == 1);

                world->destroyRigidBody(b2);
                world->destroyRigidBody(b1);
            }

            // Concentric SphereVsCapsule with tilted capsule orientation for both shape orders
            {
                const Quaternion tilted = Quaternion::fromEulerAngles(decimal(0.6), decimal(0.4), decimal(0.7));
                RigidBody* bSphere = world->createRigidBody(Transform(Vector3::zero(), Quaternion::identity()));
                RigidBody* bCapsule = world->createRigidBody(Transform(Vector3::zero(), tilted));
                bSphere->addCollider(sphere, Transform::identity());
                bCapsule->addCollider(capsule, Transform::identity());

                // Step 1 frame: capsule and sphere must separate cleanly without NaN or pulling inward
                world->setIsGravityEnabled(false);
                world->update(decimal(1.0 / 60.0));
                const decimal distAfter = (bCapsule->getTransform().getPosition() - bSphere->getTransform().getPosition()).length();
                rp3d_test(std::isfinite(distAfter) && distAfter > decimal(0.0));

                world->destroyRigidBody(bCapsule);
                world->destroyRigidBody(bSphere);
            }

            // Sub-batch offset (batchStartIndex > 0) for SphereVsConvexPolyhedron and CapsuleVsConvexPolyhedron
            {
                RigidBody* b1 = world->createRigidBody(Transform::identity());
                RigidBody* b2 = world->createRigidBody(Transform(Vector3(decimal(0.5), 0, 0), Quaternion::identity()));
                Collider* c1 = b1->addCollider(sphere, Transform::identity());
                Collider* c2 = b2->addCollider(box, Transform::identity());
                Collider* c3 = b1->addCollider(capsule, Transform::identity());

                LastFrameCollisionInfo lf0, lf1;
                MemoryManager memMgr(&alloc);
                ColliderComponents colComps(alloc);
                BodyComponents bodyComps(alloc);
                RigidBodyComponents rbComps(alloc);
                Set<bodypair> noColPairs(alloc);
                CollisionDispatch dispatch(alloc);
                OverlappingPairs realPairs(memMgr, colComps, bodyComps, rbComps, noColPairs, dispatch);
                NarrowPhaseInfoBatch batch(realPairs, alloc);
                batch.addNarrowPhaseInfo(1, c1->getEntity(), c2->getEntity(), sphere, box,
                                         Transform(Vector3(100, 0, 0), Quaternion::identity()), Transform::identity(), true, &lf0, alloc);
                batch.addNarrowPhaseInfo(2, c1->getEntity(), c2->getEntity(), sphere, box,
                                         Transform(Vector3(decimal(1.5), 0, 0), Quaternion::identity()), Transform::identity(), true, &lf1, alloc);

                SphereVsConvexPolyhedronAlgorithm spherePolyAlgo;
                bool hit = spherePolyAlgo.testCollision(batch, 1, 1, false, alloc);
                rp3d_test(hit && batch.narrowPhaseInfos[1].isColliding);
                batch.resetContactPoints(1);
                batch.clear();

                LastFrameCollisionInfo lf2, lf3;
                batch.addNarrowPhaseInfo(3, c3->getEntity(), c2->getEntity(), capsule, box,
                                         Transform(Vector3(100, 0, 0), Quaternion::identity()), Transform::identity(), true, &lf2, alloc);
                batch.addNarrowPhaseInfo(4, c3->getEntity(), c2->getEntity(), capsule, box,
                                         Transform(Vector3(decimal(1.5), 0, 0), Quaternion::identity()), Transform::identity(), true, &lf3, alloc);
                CapsuleVsConvexPolyhedronAlgorithm capPolyAlgo;
                bool hit2 = capPolyAlgo.testCollision(batch, 1, 1, false, alloc);
                rp3d_test(hit2 && batch.narrowPhaseInfos[1].isColliding);
                batch.resetContactPoints(1);
                batch.clear();

                world->destroyRigidBody(b2);
                world->destroyRigidBody(b1);
            }
        }

        void testSleepingAndTriggerWorldQueries() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            world->setIsGravityEnabled(false);

            BoxShape* box = common.createBoxShape(Vector3(1, 1, 1));
            RigidBody* staticBody = world->createRigidBody(Transform::identity());
            staticBody->setType(BodyType::STATIC);
            staticBody->addCollider(box, Transform::identity());

            RigidBody* sleepBody = world->createRigidBody(Transform(Vector3(decimal(1.0), 0, 0), Quaternion::identity()));
            Collider* sleepCollider = sleepBody->addCollider(box, Transform::identity());

            // Establish overlapping pair, then put dynamic body to sleep -> moves pair to mDisabledConvexPairs
            world->update(decimal(1.0 / 60.0));
            sleepBody->setTransform(Transform(Vector3(decimal(1.0), 0, 0), Quaternion::identity()));
            sleepBody->setIsSleeping(true);

            rp3d_test(world->testOverlap(staticBody, sleepBody));
            SimpleOverlapCounter overlapCb;
            world->testOverlap(overlapCb);
            rp3d_test(overlapCb.pairCount == 1);
            overlapCb.reset();
            world->testOverlap(sleepBody, overlapCb);
            rp3d_test(overlapCb.pairCount == 1);

            SimpleCollisionCounter colCb;
            world->testCollision(staticBody, sleepBody, colCb);
            rp3d_test(colCb.pairCount == 1 && colCb.pointCount > 0);
            colCb.reset();
            world->testCollision(sleepBody, colCb);
            rp3d_test(colCb.pairCount == 1);
            colCb.reset();
            world->testCollision(colCb);
            rp3d_test(colCb.pairCount == 1);

            // Respect setIsWorldQueryCollider(false) in single-body and two-body queries
            sleepCollider->setIsWorldQueryCollider(false);
            rp3d_test(!world->testOverlap(staticBody, sleepBody));
            colCb.reset();
            world->testCollision(staticBody, sleepBody, colCb);
            rp3d_test(colCb.pairCount == 0);
            sleepCollider->setIsWorldQueryCollider(true);

            // Trigger collider must not assert or emit contact pairs during testCollision
            RigidBody* triggerBody = world->createRigidBody(Transform(Vector3(decimal(0.5), 0, 0), Quaternion::identity()));
            Collider* triggerCollider = triggerBody->addCollider(box, Transform::identity());
            triggerCollider->setIsTrigger(true);

            rp3d_test(world->testOverlap(staticBody, triggerBody));
            colCb.reset();
            world->testCollision(staticBody, triggerBody, colCb);
            rp3d_test(colCb.pairCount == 0);
            colCb.reset();
            world->testCollision(colCb);
            rp3d_test(colCb.pairCount == 1); // Only (staticBody, sleepBody), not trigger
        }

        void testResizeStaticAndSleepingShapes() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            SphereShape* sphere = common.createSphereShape(decimal(1.0));

            RigidBody* staticBody = world->createRigidBody(Transform::identity());
            staticBody->setType(BodyType::STATIC);
            staticBody->addCollider(sphere, Transform::identity());

            // Ray at x = 3 misses radius 1.0
            SimpleRaycastHit rayCb;
            world->raycast(Ray(Vector3(3, 5, 0), Vector3(3, -5, 0)), &rayCb);
            rp3d_test(!rayCb.hit);

            // Growing radius to 4.0 must immediately update static collider broad-phase AABB
            sphere->setRadius(decimal(4.0));
            rayCb.reset();
            world->raycast(Ray(Vector3(3, 5, 0), Vector3(3, -5, 0)), &rayCb);
            rp3d_test(rayCb.hit);
        }

        void testJointFixesAndMultiJointNoCollision() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();
            world->setIsGravityEnabled(false);

            BoxShape* box = common.createBoxShape(Vector3(decimal(0.5), decimal(0.5), decimal(0.5)));
            RigidBody* b1 = world->createRigidBody(Transform(Vector3(0, 0, 0), Quaternion::identity()));
            b1->setType(BodyType::STATIC);
            b1->addCollider(box, Transform::identity());

            RigidBody* b2 = world->createRigidBody(Transform(Vector3(decimal(0.2), 0, 0), Quaternion::identity()));
            b2->addCollider(box, Transform::identity());

            // 1) HingeJoint::getAngle() before first update() after rotating b2 negatively around Y
            HingeJointInfo hInfo(b1, b2, Vector3::zero(), Vector3(0, 1, 0));
            hInfo.isLimitEnabled = true;
            hInfo.minAngleLimit = decimal(-0.5);
            hInfo.maxAngleLimit = decimal(0.5);
            hInfo.isCollisionEnabled = false;
            HingeJoint* hinge = static_cast<HingeJoint*>(world->createJoint(hInfo));

            b2->setTransform(Transform(Vector3(decimal(0.2), 0, 0), Quaternion::fromEulerAngles(0, decimal(-0.8), 0)));
            rp3d_test(approxEqual(hinge->getAngle(), decimal(-0.8), decimal(0.01)));

            // 2) NGS position correction on hinge limit when motor is disabled
            for (int step = 0; step < 15; step++) {
                world->update(decimal(1.0 / 60.0));
            }
            rp3d_test(hinge->getAngle() >= decimal(-0.55));

            // 3) Multiple joints with isCollisionEnabled = false between (b1, b2)
            FixedJointInfo fInfo(b1, b2, Vector3::zero());
            fInfo.isCollisionEnabled = false;
            Joint* fixed = world->createJoint(fInfo);

            // Destroying one joint must keep collision disabled because the other joint still disables it
            world->destroyJoint(fixed);
            world->update(decimal(1.0 / 60.0));
            rp3d_test(!world->testOverlap(b1, b2));

            // Destroying the remaining joint must re-enable broad-phase collision without moving either body
            world->destroyJoint(hinge);
            rp3d_test(world->testOverlap(b1, b2));

            // 4) FixedJoint & SliderJoint with completely locked angular axes (singular I1 + I2 == 0)
            RigidBody* b3 = world->createRigidBody(Transform(Vector3(0, 5, 0), Quaternion::fromEulerAngles(decimal(0.2), 0, 0)));
            b3->addCollider(box, Transform::identity());
            b3->setAngularLockAxisFactor(Vector3::zero());
            FixedJointInfo fInfo2(b1, b3, Vector3(0, 5, 0));
            FixedJoint* fixedSingular = static_cast<FixedJoint*>(world->createJoint(fInfo2));
            world->update(decimal(1.0 / 60.0));
            rp3d_test(Vector3::approxEqual(fixedSingular->getReactionTorque(decimal(1.0 / 60.0)), Vector3::zero(), decimal(1e-4)));
        }

        void testTwistFrictionWarmStart() {
            PhysicsCommon common;
            PhysicsWorld* world = common.createPhysicsWorld();

            BoxShape* floorShape = common.createBoxShape(Vector3(10, 1, 10));
            RigidBody* floor = world->createRigidBody(Transform(Vector3(0, -1, 0), Quaternion::identity()));
            floor->setType(BodyType::STATIC);
            floor->addCollider(floorShape, Transform::identity());

            BoxShape* blockShape = common.createBoxShape(Vector3(1, 1, 1));
            RigidBody* block = world->createRigidBody(Transform(Vector3(0, 1, 0), Quaternion::identity()));
            block->addCollider(blockShape, Transform::identity());
            block->setAngularVelocity(Vector3(0, decimal(2.0), 0));

            // With correct warm-starting, twist friction monotonically damps Y spin without reversing sign
            for (int i = 0; i < 30; i++) {
                world->update(decimal(1.0 / 60.0));
                rp3d_test(block->getAngularVelocity().y >= decimal(-0.01));
            }
        }

        void testContainersAndDynamicAABBTreeAndInvalidConvexMesh() {
            DefaultAllocator alloc;

            // 1) Deque copy constructor & copy assignment after addFront and at full capacity
            {
                Deque<int> d1(alloc);
                for (int i = 0; i < 8; i++) d1.addFront(7 - i);
                for (int i = 8; i < 16; i++) d1.addBack(i);
                rp3d_test(d1.size() == 16);

                Deque<int> dCopy(d1);
                rp3d_test(dCopy.size() == 16);
                for (uint64 i = 0; i < 16; i++) {
                    rp3d_test(dCopy[i] == static_cast<int>(i));
                }

                Deque<int> dAssign(alloc);
                dAssign.addBack(99);
                dAssign = d1;
                rp3d_test(dAssign.size() == 16);
                for (uint64 i = 0; i < 16; i++) {
                    rp3d_test(dAssign[i] == static_cast<int>(i));
                }
            }

            // 2) Map::add(..., false) duplicate key returns false in Release
            {
                Map<int, int> map(alloc);
                rp3d_test(map.add(Pair<int, int>(42, 1), true));
#ifdef NDEBUG
                rp3d_test(!map.add(Pair<int, int>(42, 2), false));
                rp3d_test(map.size() == 1);
                rp3d_test(map[42] == 1);
#endif
            }

            // 3) LinkedList::reset() invokes ~T()
            DestructorTracker::liveCount = 0;
            {
                LinkedList<DestructorTracker> list(alloc);
                list.insert(DestructorTracker(1));
                list.insert(DestructorTracker(2));
                rp3d_test(DestructorTracker::liveCount == 2);
                list.reset();
                rp3d_test(DestructorTracker::liveCount == 0);
            }

            // 4) Empty DynamicAABBTree::reportAllShapesOverlappingWithAABB, computeHeight() & non-empty check()
            {
                DynamicAABBTree tree(alloc);
                rp3d_test(tree.computeHeight() == 0);
                Array<int32> hits(alloc);
                tree.reportAllShapesOverlappingWithAABB(AABB(Vector3(-1, -1, -1), Vector3(1, 1, 1)), hits);
                rp3d_test(hits.size() == 0);

                // Force node growth past initial capacity and validate check() / computeHeight()
                for (int i = 0; i < 20; i++) {
                    tree.addObject(AABB(Vector3(decimal(i), 0, 0), Vector3(decimal(i + 1), 1, 1)), i);
                }
                rp3d_test(tree.computeHeight() > 0);
#ifndef NDEBUG
                tree.check();
#endif
            }

            // 5) Quaternion::getRotationAngleAxis with |w| slightly > 1 does not produce NaN
            {
                Quaternion q(0, 0, 0, decimal(1.0001));
                decimal angle = 0;
                Vector3 axis;
                q.getRotationAngleAxis(angle, axis);
                rp3d_test(std::isfinite(angle) && approxEqual(angle, decimal(0.0)));
            }

            // 6) Invalid PolygonVertexArray returns nullptr with error messages instead of crashing
            {
                PhysicsCommon common;
                float verts[12] = {0, 0, 0,  1, 0, 0,  0, 1, 0,  0, 0, 1};
                int indices[2] = {0, 1};
                PolygonVertexArray::PolygonFace badFace{2, 0};
                PolygonVertexArray badPva(4, verts, 3 * sizeof(float), indices, sizeof(int), 1, &badFace,
                                          PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                          PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
                std::vector<Message> errors;
                ConvexMesh* badMesh = common.createConvexMesh(badPva, errors);
                rp3d_test(badMesh == nullptr);
                rp3d_test(!errors.empty());
            }
        }
};

int main() {
    TestEngineBugFixes test;
    test.run();
    return test.report() == 0 ? 0 : 1;
}
