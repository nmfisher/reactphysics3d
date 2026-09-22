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
bool testSpherePackets(NarrowPhaseInfoBatch& batch, uint32 index, uint32 end) {
    bool isCollisionFound = false;
    using Float4 = detail::SpherePairFloat4;
    for (; end - index >= 4; index += 4) {
        // Trigger/mixed-reporting packets have too little contact arithmetic to
        // amortize packing; retain the scalar path for these common workloads.
        if (!batch.narrowPhaseInfos[index].reportContacts || !batch.narrowPhaseInfos[index + 1].reportContacts ||
            !batch.narrowPhaseInfos[index + 2].reportContacts || !batch.narrowPhaseInfos[index + 3].reportContacts) {
            isCollisionFound |= testScalarRange(batch, index, index + 4);
            continue;
        }

        Vector3 betweenCenters[4];
        float radius1[4], radius2[4], radiusSum[4];
        for (uint32 lane = 0; lane < 4; lane++) {
            const auto& info = batch.narrowPhaseInfos[index + lane];
            assert(info.nbContactPoints == 0 && !info.isColliding);
            betweenCenters[lane] = info.shape2ToWorldTransform.getPosition() - info.shape1ToWorldTransform.getPosition();
            radius1[lane] = static_cast<const SphereShape*>(info.collisionShape1)->getRadius();
            radius2[lane] = static_cast<const SphereShape*>(info.collisionShape2)->getRadius();
            radiusSum[lane] = radius1[lane] + radius2[lane];
        }

        // Lanes are independent pairs, not the coordinates of one vector. The
        // explicit gathers never read a fourth float from a three-float Vector3.
        const Float4 x = Float4::set(betweenCenters[0].x, betweenCenters[1].x, betweenCenters[2].x, betweenCenters[3].x);
        const Float4 y = Float4::set(betweenCenters[0].y, betweenCenters[1].y, betweenCenters[2].y, betweenCenters[3].y);
        const Float4 z = Float4::set(betweenCenters[0].z, betweenCenters[1].z, betweenCenters[2].z, betweenCenters[3].z);
        const Float4 distances = (x * x + y * y) + z * z;
        float distanceSquare[4];
        distances.store(distanceSquare);

        const Float4 radii = Float4::set(radiusSum[0], radiusSum[1], radiusSum[2], radiusSum[3]);
        const Float4 radiusSquares = radii * radii;
        const Float4 tolerance = Float4::set(8 * MACHINE_EPSILON, 8 * MACHINE_EPSILON, 8 * MACHINE_EPSILON, 8 * MACHINE_EPSILON);
        const Float4 epsilon = Float4::set(MACHINE_EPSILON, MACHINE_EPSILON, MACHINE_EPSILON, MACHINE_EPSILON);
        const unsigned int boundary = (distances - radiusSquares).abs().lessMask(radiusSquares * tolerance) |
                                      (distances - epsilon).abs().lessMask(epsilon * tolerance);
        unsigned int candidates = distances.lessMask(radiusSquares);
        // Resolve decision boundaries with the original scalar expression:
        // FMA contraction may otherwise change classification by a rounding bit.
        for (uint32 lane = 0; lane < 4; lane++) {
            if ((boundary & (1u << lane)) != 0) {
                distanceSquare[lane] = betweenCenters[lane].lengthSquare();
                candidates &= ~(1u << lane);
                if (distanceSquare[lane] < radiusSum[lane] * radiusSum[lane]) candidates |= 1u << lane;
            }
        }
        if (candidates == 0) continue;

        const Float4 correctedDistances = Float4::set(distanceSquare[0], distanceSquare[1], distanceSquare[2], distanceSquare[3]);
        float depth[4];
        (radii - correctedDistances.sqrt()).store(depth);

        // Use wide contact arithmetic for four ordinary reporting contacts.
        // Exceptional/trigger/sparse packets retain the established scalar path.
        unsigned int contacts = 0;
        for (uint32 lane = 0; lane < 4; lane++) {
            if ((candidates & (1u << lane)) != 0 && depth[lane] > 0 && distanceSquare[lane] > MACHINE_EPSILON &&
                batch.narrowPhaseInfos[index + lane].reportContacts) contacts |= 1u << lane;
        }
        if (contacts == 15) {
            using Vector4 = detail::SpherePairVector4;
            Vector3 centers1Local[4], centers2Local[4], local1[4], local2[4], normals[4];
            for (uint32 lane = 0; lane < 4; lane++) {
                const auto& info = batch.narrowPhaseInfos[index + lane];
                // Retain scalar transform arithmetic: changing its contraction
                // can amplify cancellation for bodies far from the origin.
                const Transform inverse1 = info.shape1ToWorldTransform.getInverse();
                const Transform inverse2 = info.shape2ToWorldTransform.getInverse();
                centers2Local[lane] = inverse1 * info.shape2ToWorldTransform.getPosition();
                centers1Local[lane] = inverse2 * info.shape1ToWorldTransform.getPosition();
            }
            (Vector4::gather(centers2Local).unit() * Float4::set(radius1[0], radius1[1], radius1[2], radius1[3])).store(local1);
            (Vector4::gather(centers1Local).unit() * Float4::set(radius2[0], radius2[1], radius2[2], radius2[3])).store(local2);
            Vector4::gather(betweenCenters).unit().store(normals);
            for (uint32 lane = 0; lane < 4; lane++) {
                batch.addContactPoint(index + lane, normals[lane], depth[lane], local1[lane], local2[lane]);
                batch.narrowPhaseInfos[index + lane].isColliding = true;
            }
            isCollisionFound = true;
        }
        else {
            for (uint32 lane = 0; lane < 4; lane++) {
                // Rounding may produce zero depth despite the squared-distance test.
                if ((candidates & (1u << lane)) != 0 && depth[lane] > 0) {
                    reportSphereContact(batch, index + lane, betweenCenters[lane], distanceSquare[lane],
                                        depth[lane], radius1[lane], radius2[lane]);
                    isCollisionFound = true;
                }
            }
        }
    }
    return testScalarRange(batch, index, end) || isCollisionFound;
}
}
#endif

bool SphereVsSphereAlgorithm::testCollision(NarrowPhaseInfoBatch& batch, uint32 batchStartIndex,
                                           uint32 batchNbItems, MemoryAllocator& /*memoryAllocator*/) {
    assert(batchStartIndex <= batch.getNbObjects());
    assert(batchNbItems <= batch.getNbObjects() - batchStartIndex);
    const uint32 end = batchStartIndex + batchNbItems;
#ifdef RP3D_SPHERE_PAIR_SIMD
    if (batchNbItems >= 4) return testSpherePackets(batch, batchStartIndex, end);
#endif
    return testScalarRange(batch, batchStartIndex, end);
}
