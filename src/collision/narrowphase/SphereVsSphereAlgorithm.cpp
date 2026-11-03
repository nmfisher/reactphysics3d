/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

// Libraries
#include <reactphysics3d/collision/narrowphase/SphereVsSphereAlgorithm.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h>
#include "SpherePairSIMD.h"

using namespace reactphysics3d;

namespace {

// Contact emission is shared by packets and the scalar tail. Keep the existing
// local-space construction and coincident-center convention, including triggers.
void reportSphereContact(NarrowPhaseInfoBatch& batch, uint32 index, const Vector3& betweenCenters,
                         decimal distanceSquare, decimal penetrationDepth, decimal radius1, decimal radius2) {
    auto& info = batch.narrowPhaseInfos[index];
    if (info.reportContacts) {
        const Transform transform1Inverse = info.shape1ToWorldTransform.getInverse();
        const Transform transform2Inverse = info.shape2ToWorldTransform.getInverse();
        Vector3 point1, point2, normal;
        if (distanceSquare > MACHINE_EPSILON) {
            const Vector3 center2Local = transform1Inverse * info.shape2ToWorldTransform.getPosition();
            const Vector3 center1Local = transform2Inverse * info.shape1ToWorldTransform.getPosition();
            point1 = radius1 * center2Local.getUnit();
            point2 = radius2 * center1Local.getUnit();
            normal = betweenCenters.getUnit();
        }
        else {
            normal.setAllValues(0, 1, 0);
            point1 = radius1 * (transform1Inverse.getOrientation() * normal);
            point2 = radius2 * (transform2Inverse.getOrientation() * normal);
        }
        batch.addContactPoint(index, normal, penetrationDepth, point1, point2);
    }
    info.isColliding = true;
}

bool testScalarRange(NarrowPhaseInfoBatch& batch, uint32 index, uint32 end) {
    bool isCollisionFound = false;
    for (; index < end; index++) {
        const auto& info = batch.narrowPhaseInfos[index];
        assert(info.nbContactPoints == 0 && !info.isColliding);
        const Vector3 betweenCenters = info.shape2ToWorldTransform.getPosition() - info.shape1ToWorldTransform.getPosition();
        const decimal distanceSquare = betweenCenters.lengthSquare();
        const decimal radius1 = static_cast<const SphereShape*>(info.collisionShape1)->getRadius();
        const decimal radius2 = static_cast<const SphereShape*>(info.collisionShape2)->getRadius();
        const decimal radiusSum = radius1 + radius2;
        if (distanceSquare < radiusSum * radiusSum) {
            const decimal depth = radiusSum - std::sqrt(distanceSquare);
            if (depth > 0) {
                reportSphereContact(batch, index, betweenCenters, distanceSquare, depth, radius1, radius2);
                isCollisionFound = true;
            }
        }
    }
    return isCollisionFound;
}


}

#ifdef RP3D_SPHERE_PAIR_SIMD
namespace {

// Consume leading triggers without entering the contact-packet routine. Keeping
// this loop small avoids contact scratch storage/register pressure for trigger
// batches, while stopping at the first reporting pair preserves mixed batches.
bool testTriggerPrefix(NarrowPhaseInfoBatch& batch, uint32& index, uint32 end) {
    bool isCollisionFound = false;
    for (; index < end && !batch.narrowPhaseInfos[index].reportContacts; index++) {
        auto& info = batch.narrowPhaseInfos[index];
        assert(info.nbContactPoints == 0 && !info.isColliding);
        const Vector3 betweenCenters = info.shape2ToWorldTransform.getPosition() - info.shape1ToWorldTransform.getPosition();
        const decimal distanceSquare = betweenCenters.lengthSquare();
        const decimal radiusSum = static_cast<const SphereShape*>(info.collisionShape1)->getRadius() +
                                  static_cast<const SphereShape*>(info.collisionShape2)->getRadius();
        if (distanceSquare < radiusSum * radiusSum && radiusSum - std::sqrt(distanceSquare) > 0) {
            info.isColliding = true;
            isCollisionFound = true;
        }
    }
    return isCollisionFound;
}

// Pack only ordinary reporting contacts. Rejected pairs and triggers never pay
// for vector gathers, and contacts can share a packet across intervening misses.
struct SphereContactPacket {
    uint32 count = 0;
    uint32 indices[4];
    Vector3 betweenCenters[4];
    float distanceSquare[4], radius1[4], radius2[4];

    bool flush(NarrowPhaseInfoBatch& batch) {
        bool isCollisionFound = false;
        if (count == 4) {
            using Float4 = detail::SpherePairFloat4;
            using Vector4 = detail::SpherePairVector4;
            const Float4 r1 = Float4::set(radius1[0], radius1[1], radius1[2], radius1[3]);
            const Float4 r2 = Float4::set(radius2[0], radius2[1], radius2[2], radius2[3]);
            const Float4 distances = Float4::set(distanceSquare[0], distanceSquare[1], distanceSquare[2], distanceSquare[3]);
            float depth[4];
            (r1 + r2 - distances.sqrt()).store(depth);
            Vector3 centers1Local[4], centers2Local[4], local1[4], local2[4], normals[4];
            for (uint32 lane = 0; lane < 4; lane++) {
                const auto& info = batch.narrowPhaseInfos[indices[lane]];
                // Retain scalar transforms: vector contraction can amplify
                // cancellation for bodies far from the origin.
                const Transform inverse1 = info.shape1ToWorldTransform.getInverse();
                const Transform inverse2 = info.shape2ToWorldTransform.getInverse();
                centers2Local[lane] = inverse1 * info.shape2ToWorldTransform.getPosition();
                centers1Local[lane] = inverse2 * info.shape1ToWorldTransform.getPosition();
            }
            (Vector4::gather(centers2Local).unit() * r1).store(local1);
            (Vector4::gather(centers1Local).unit() * r2).store(local2);
            Vector4::gather(betweenCenters).unit().store(normals);
            for (uint32 lane = 0; lane < 4; lane++) {
                // Squared-distance overlap can still round to zero depth.
                if (depth[lane] > 0) {
                    batch.addContactPoint(indices[lane], normals[lane], depth[lane], local1[lane], local2[lane]);
                    batch.narrowPhaseInfos[indices[lane]].isColliding = true;
                    isCollisionFound = true;
                }
            }
        }
        else {
            for (uint32 lane = 0; lane < count; lane++) {
                const decimal depth = radius1[lane] + radius2[lane] - std::sqrt(distanceSquare[lane]);
                if (depth > 0) {
                    reportSphereContact(batch, indices[lane], betweenCenters[lane], distanceSquare[lane],
                                        depth, radius1[lane], radius2[lane]);
                    isCollisionFound = true;
                }
            }
        }
        count = 0;
        return isCollisionFound;
    }
};

bool testSpherePackets(NarrowPhaseInfoBatch& batch, uint32 index, uint32 end) {
    bool isCollisionFound = false;
    SphereContactPacket packet;
    for (; index < end; index++) {
        const auto& info = batch.narrowPhaseInfos[index];
        assert(info.nbContactPoints == 0 && !info.isColliding);
        const Vector3 betweenCenters = info.shape2ToWorldTransform.getPosition() - info.shape1ToWorldTransform.getPosition();
        // Keep the exact scalar classification expression, including tangency
        // and the coincident-center threshold, without vector boundary repair.
        const decimal distanceSquare = betweenCenters.lengthSquare();
        const decimal radius1 = static_cast<const SphereShape*>(info.collisionShape1)->getRadius();
        const decimal radius2 = static_cast<const SphereShape*>(info.collisionShape2)->getRadius();
        const decimal radiusSum = radius1 + radius2;
        if (distanceSquare < radiusSum * radiusSum) {
            if (info.reportContacts && distanceSquare > MACHINE_EPSILON) {
                const uint32 lane = packet.count++;
                packet.indices[lane] = index;
                packet.betweenCenters[lane] = betweenCenters;
                packet.distanceSquare[lane] = distanceSquare;
                packet.radius1[lane] = radius1;
                packet.radius2[lane] = radius2;
                if (packet.count == 4) isCollisionFound |= packet.flush(batch);
            }
            else {
                const decimal depth = radiusSum - std::sqrt(distanceSquare);
                if (depth > 0) {
                    // Flush before an exceptional contact to retain pair order.
                    // Triggers emit no contacts, so they need not break packets.
                    if (info.reportContacts) isCollisionFound |= packet.flush(batch);
                    reportSphereContact(batch, index, betweenCenters, distanceSquare, depth, radius1, radius2);
                    isCollisionFound = true;
                }
            }
        }
    }
    return packet.flush(batch) || isCollisionFound;
}
}
#endif

bool SphereVsSphereAlgorithm::testCollision(NarrowPhaseInfoBatch& batch, uint32 batchStartIndex,
                                           uint32 batchNbItems, MemoryAllocator& /*memoryAllocator*/) {
    assert(batchStartIndex <= batch.getNbObjects());
    assert(batchNbItems <= batch.getNbObjects() - batchStartIndex);
    const uint32 end = batchStartIndex + batchNbItems;
#ifdef RP3D_SPHERE_PAIR_SIMD
    if (batchNbItems >= 4) {
        uint32 index = batchStartIndex;
        const bool triggersFound = testTriggerPrefix(batch, index, end);
        if (index == end) return triggersFound;
        const bool contactsFound = end - index >= 4 ? testSpherePackets(batch, index, end) : testScalarRange(batch, index, end);
        return contactsFound || triggersFound;
    }
#endif
    return testScalarRange(batch, batchStartIndex, end);
}
