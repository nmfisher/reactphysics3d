#ifndef RP3D_TEST_SPHERE_VS_SPHERE_BATCH_H
#define RP3D_TEST_SPHERE_VS_SPHERE_BATCH_H

#include "SphereBatchFixture.h"
#include "Test.h"
#include <cmath>
#include <limits>

namespace reactphysics3d {

class TestSphereVsSphereBatch : public Test {
    bool close(decimal a, decimal b) {
        return std::abs(a - b) <= decimal(32) * MACHINE_EPSILON * std::max(decimal(1), std::max(std::abs(a), std::abs(b)));
    }

    void compareVector(const Vector3& a, const Vector3& b) {
        rp3d_test(close(a.x, b.x));
        rp3d_test(close(a.y, b.y));
        rp3d_test(close(a.z, b.z));
    }

    void compareRange(SphereBatchFixture& fixture, uint32 start, uint32 count) {
        SphereBatchFixture::reset(fixture.batch);
        SphereBatchFixture::reset(fixture.reference);
        // Sentinels detect writes outside the requested range, even on rejected pairs.
        for (uint32 i = 0; i < fixture.batch.getNbObjects(); i++) {
            if (i < start || i >= start + count) {
                fixture.batch.narrowPhaseInfos[i].isColliding = true;
                fixture.reference.narrowPhaseInfos[i].isColliding = true;
            }
        }
        const bool result = fixture.algorithm.testCollision(fixture.batch, start, count, fixture.memory.getHeapAllocator());
        bool expected = false;
        // Calls with one pair always exercise the scalar implementation.
        for (uint32 i = start; i < start + count; i++) {
            expected |= fixture.algorithm.testCollision(fixture.reference, i, 1, fixture.memory.getHeapAllocator());
        }
        rp3d_test(result == expected);
        for (uint32 i = 0; i < fixture.batch.getNbObjects(); i++) {
            const auto& actual = fixture.batch.narrowPhaseInfos[i];
            const auto& scalar = fixture.reference.narrowPhaseInfos[i];
            rp3d_test(actual.overlappingPairId == i);
            rp3d_test(actual.isColliding == scalar.isColliding);
            rp3d_test(actual.nbContactPoints == scalar.nbContactPoints);
            if (actual.nbContactPoints != scalar.nbContactPoints) continue;
            for (uint32 j = 0; j < actual.nbContactPoints; j++) {
                const auto& a = actual.contactPoints[j];
                const auto& b = scalar.contactPoints[j];
                rp3d_test(close(a.penetrationDepth, b.penetrationDepth));
                compareVector(a.normal, b.normal);
                compareVector(a.localPoint1, b.localPoint1);
                compareVector(a.localPoint2, b.localPoint2);
            }
        }
    }

    void testAnalyticCases() {
        SphereBatchFixture fixture;
        const Quaternion identity = Quaternion::identity();
        // Overlap, separation, exact touching, coincident centers, and a trigger.
        for (decimal x : {decimal(2), decimal(4), decimal(3), decimal(0), decimal(1)}) {
            fixture.add(1, 2, Transform(Vector3(0, 0, 0), identity),
                        Transform(Vector3(x, 0, 0), identity), x != decimal(1));
        }
        rp3d_test(fixture.algorithm.testCollision(fixture.batch, 0, 5, fixture.memory.getHeapAllocator()));
        const auto& hit = fixture.batch.narrowPhaseInfos[0];
        rp3d_test(hit.nbContactPoints == 1);
        rp3d_test(hit.contactPoints[0].penetrationDepth == decimal(1));
        compareVector(hit.contactPoints[0].normal, Vector3(1, 0, 0));
        compareVector(hit.contactPoints[0].localPoint1, Vector3(1, 0, 0));
        compareVector(hit.contactPoints[0].localPoint2, Vector3(-2, 0, 0));
        rp3d_test(!fixture.batch.narrowPhaseInfos[1].isColliding);
        rp3d_test(!fixture.batch.narrowPhaseInfos[2].isColliding);
        compareVector(fixture.batch.narrowPhaseInfos[3].contactPoints[0].normal, Vector3(0, 1, 0));
        rp3d_test(fixture.batch.narrowPhaseInfos[4].isColliding);
        rp3d_test(fixture.batch.narrowPhaseInfos[4].nbContactPoints == 0);
    }

    void testPacketsAndBoundaries() {
        SphereBatchFixture fixture;
        uint32 seed = 1234567;
        auto random = [&seed]() {
            seed = seed * 1664525u + 1013904223u;
            return decimal(seed >> 8) / decimal(16777216);
        };
        const Quaternion q1 = Quaternion::fromEulerAngles(decimal(0.3), decimal(-0.7), decimal(0.2));
        const Quaternion q2 = Quaternion::fromEulerAngles(decimal(-0.4), decimal(0.1), decimal(0.8));
        for (uint32 i = 0; i < 257; i++) {
            const decimal r1 = decimal(0.01) + random() * 3;
            const decimal r2 = decimal(0.01) + random() * 3;
            const Vector3 center(random() * 10, random() * 10, random() * 10);
            Vector3 delta(random() - decimal(0.5), random() - decimal(0.5), random() - decimal(0.5));
            delta = delta.getUnit() * ((r1 + r2) * random() * 2);
            if (i % 7 == 0) delta = Vector3::zero();
            fixture.add(r1, r2, Transform(center, q1), Transform(center + delta, q2), i % 3 != 0);
        }
        // Adjacent representable distances around tangency and the degeneracy threshold.
        for (decimal boundary : {decimal(2), std::sqrt(MACHINE_EPSILON)}) {
            for (decimal distance : {std::nextafter(boundary, decimal(0)), boundary,
                                     std::nextafter(boundary, std::numeric_limits<decimal>::infinity())}) {
                fixture.add(1, 1, Transform(Vector3::zero(), q1), Transform(Vector3(distance, 0, 0), q2), true);
            }
        }
        // Oblique near-tangent pairs exercise rounding differences in the
        // three-term sum, including targets that contract scalar operations.
        for (decimal scale : {decimal(0.001), decimal(1), decimal(1000)}) {
            for (uint32 i = 0; i < 32; i++) {
                const Vector3 direction = Vector3(random(), random(), random()).getUnit();
                for (decimal distance : {std::nextafter(scale * 2, decimal(0)), scale * 2,
                                         std::nextafter(scale * 2, std::numeric_limits<decimal>::infinity())}) {
                    fixture.add(scale, scale, Transform(Vector3::zero(), q1),
                                Transform(direction * distance, q2), true);
                }
            }
        }
        for (uint32 start = 0; start < 4; start++) {
            for (uint32 count = 0; count <= 17; count++) compareRange(fixture, start, count);
            compareRange(fixture, start, fixture.batch.getNbObjects() - start);
        }
        compareRange(fixture, fixture.batch.getNbObjects(), 0);
    }

    void testLaneMasks() {
        for (uint32 mask = 0; mask < 16; mask++) {
            SphereBatchFixture fixture;
            for (uint32 lane = 0; lane < 4; lane++) {
                const decimal x = (mask & (1u << lane)) != 0 ? decimal(1) : decimal(3);
                fixture.add(1, 1, Transform::identity(), Transform(Vector3(x, 0, 0), Quaternion::identity()), true);
            }
            const bool result = fixture.algorithm.testCollision(fixture.batch, 0, 4, fixture.memory.getHeapAllocator());
            rp3d_test(result == (mask != 0));
            for (uint32 lane = 0; lane < 4; lane++) {
                const auto& info = fixture.batch.narrowPhaseInfos[lane];
                const bool hit = (mask & (1u << lane)) != 0;
                rp3d_test(info.isColliding == hit);
                rp3d_test(info.nbContactPoints == (hit ? 1 : 0));
            }
        }
    }

    void testReportingPackets() {
        SphereBatchFixture fixture;
        const Quaternion q1 = Quaternion::fromEulerAngles(decimal(0.7), decimal(0.3), decimal(-0.8));
        const Quaternion q2 = Quaternion::fromEulerAngles(decimal(-0.2), decimal(1.1), decimal(0.6));
        for (uint32 i = 0; i < 33; i++) {
            const Vector3 position(decimal(i) / 3, decimal(-2), decimal(4));
            fixture.add(decimal(0.7), decimal(1.1), Transform(position, q1),
                        Transform(position + Vector3(decimal(0.5), decimal(0.7), decimal(-0.3)), q2), true);
        }
        for (uint32 start = 0; start < 4; start++) compareRange(fixture, start, 33 - start);

        SphereBatchFixture distant;
        for (decimal offset : {decimal(1000), decimal(100000)}) {
            for (uint32 i = 0; i < 4; i++) {
                const Vector3 position(offset, -offset, offset);
                distant.add(1, 1, Transform(position, q1),
                            Transform(position + Vector3(decimal(0.5), decimal(0.75), decimal(-0.25)), q2), true);
            }
        }
        compareRange(distant, 0, 8);
    }

    void testInterleavedContacts() {
        // Nonadjacent contacts, trigger prefixes, coincident centers, and tails.
        // Exercise every reporting mask across multiple packets and subranges.
        for (uint32 reportingMask = 0; reportingMask < 16; reportingMask++) {
            SphereBatchFixture fixture;
            for (uint32 i = 0; i < 37; i++) {
                const decimal x = i % 7 == 0 ? decimal(0) : i % 3 == 0 ? decimal(3) : decimal(1);
                const bool report = i >= 5 && (reportingMask & (1u << (i % 4))) != 0;
                fixture.add(1, 1, Transform::identity(), Transform(Vector3(x, 0, 0), Quaternion::identity()), report);
            }
            for (uint32 start = 0; start < 8; start++) {
                for (uint32 tail = 0; tail < 4; tail++) compareRange(fixture, start, 37 - start - tail);
            }
        }
    }

    public:
        explicit TestSphereVsSphereBatch(const std::string& name) : Test(name) {}
        void run() override {
            testAnalyticCases();
            testPacketsAndBoundaries();
            testLaneMasks();
            testReportingPackets();
            testInterleavedContacts();
        }
};

}
#endif
