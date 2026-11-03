#include "tests/collision/SphereBatchFixture.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

using namespace reactphysics3d;

int main() {
    // Includes gathers, contact output and resetting results between calls.
    // Build once with RP3D_SIMD_ENABLED=ON and once with OFF to compare the
    // same workload without introducing per-pair scalar function-call overhead.
    volatile unsigned int sink = 0;
    std::puts("pairs,scenario,median_ns_per_pair");
    for (uint32 count : {3u, 4u, 7u, 64u, 1024u}) {
        for (int scenario = 0; scenario < 4; scenario++) {
            SphereBatchFixture fixture;
            const bool reportContacts = scenario != 3;
            for (uint32 i = 0; i < count; i++) {
                const bool overlap = scenario >= 2 || (scenario == 1 && i % 2 == 0);
                fixture.add(1, 1, Transform::identity(),
                            Transform(overlap ? Vector3(1, decimal(0.2), decimal(0.1)) : Vector3(decimal(1.5), decimal(1.5), 0),
                                      Quaternion::identity()), reportContacts);
            }
            const uint32 iterations = std::max(100u, 200000u / count);
            std::vector<double> samples;
            for (int sample = 0; sample < 8; sample++) {
                const auto start = std::chrono::steady_clock::now();
                for (uint32 iteration = 0; iteration < iterations; iteration++) {
                    SphereBatchFixture::reset(fixture.batch);
                    sink += fixture.algorithm.testCollision(fixture.batch, 0, count, fixture.memory.getHeapAllocator());
                }
                const auto stop = std::chrono::steady_clock::now();
                if (sample != 0) {
                    samples.push_back(std::chrono::duration<double, std::nano>(stop - start).count() / (double(iterations) * count));
                }
            }
            std::sort(samples.begin(), samples.end());
            const char* names[] = {"separated", "mixed", "contacts", "triggers"};
            std::printf("%u,%s,%.3f\n", count, names[scenario], samples[samples.size() / 2]);
        }
    }
    return sink == 0 ? 1 : 0;
}
