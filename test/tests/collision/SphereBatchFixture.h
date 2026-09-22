#ifndef RP3D_TEST_SPHERE_BATCH_FIXTURE_H
#define RP3D_TEST_SPHERE_BATCH_FIXTURE_H

#include <reactphysics3d/reactphysics3d.h>
#include <reactphysics3d/collision/narrowphase/CollisionDispatch.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include <reactphysics3d/engine/OverlappingPairs.h>
#include <initializer_list>

namespace reactphysics3d {

// Direct narrow-phase fixture: controls packet offsets, tails and reporting
// independently of broad-phase ordering. Also used by the optional benchmark.
struct SphereBatchFixture {
    PhysicsCommon common;
    MemoryManager memory{nullptr};
    ColliderComponents colliders{memory.getHeapAllocator()};
    BodyComponents bodies{memory.getHeapAllocator()};
    RigidBodyComponents rigidBodies{memory.getHeapAllocator()};
    Set<bodypair> exclusions{memory.getHeapAllocator()};
    CollisionDispatch dispatch{memory.getHeapAllocator()};
    OverlappingPairs pairs{memory, colliders, bodies, rigidBodies, exclusions, dispatch};
    NarrowPhaseInfoBatch batch{pairs, memory.getHeapAllocator()};
    NarrowPhaseInfoBatch reference{pairs, memory.getHeapAllocator()};
    SphereVsSphereAlgorithm algorithm;

    void add(decimal radius1, decimal radius2, const Transform& transform1,
             const Transform& transform2, bool reportContacts) {
        SphereShape* sphere1 = common.createSphereShape(radius1);
        SphereShape* sphere2 = common.createSphereShape(radius2);
        const uint32 i = batch.getNbObjects();
        for (NarrowPhaseInfoBatch* target : {&batch, &reference}) {
            target->addNarrowPhaseInfo(i, Entity(2 * i, 0), Entity(2 * i + 1, 0), sphere1, sphere2,
                                      transform1, transform2, reportContacts, nullptr, memory.getHeapAllocator());
        }
    }

    static void reset(NarrowPhaseInfoBatch& target) {
        for (uint32 i = 0; i < target.getNbObjects(); i++) {
            target.narrowPhaseInfos[i].nbContactPoints = 0;
            target.narrowPhaseInfos[i].isColliding = false;
        }
    }

    ~SphereBatchFixture() {
        reset(batch);
        reset(reference);
    }
};

}
#endif
