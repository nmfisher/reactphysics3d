# Full-world sphere benchmark

`sphere_world_benchmark` measures complete `PhysicsWorld::update()` calls, including broad phase, narrow phase, island/contact preparation, solving, integration, and event callbacks. World construction, warmup, CSV formatting, and final-state inspection are outside the measured interval. It is an explicit build target, excluded from normal builds and tests.

## Workloads and output

Defaults: 512 dynamic spheres (`--side 8` cubed), radius 0.5, fixed 1/60-second timestep, six velocity iterations, three position iterations, zero restitution, 120 warmup steps, 360 measured steps, three fresh-world repetitions. All scenes are deterministic and use no random input or per-step resets.

| Scene | Setup | Purpose |
| --- | --- | --- |
| `dense` | Spheres fall into a box with a floor and four walls; sleeping disabled | Persistent contacts, island preparation, solver, and sphere/box boundary contacts |
| `sparse` | Isolated diagonal pairs with overlapping AABBs and separated spheres; common velocity, no gravity | Broad-phase candidates that narrow phase rejects |
| `triggers` | Overlapping sphere grid, all colliders are triggers; common velocity, no gravity | Persistent trigger overlaps without contact solving |
| `sleeping` | Widely separated spheres; 31 of every 32 initially asleep, remaining bodies move | Mostly sleeping world overhead |

CSV output includes scenario, repetition, sphere count, awake spheres at the end, mean current contact/trigger pairs per measured step, median and nearest-rank p95 step times in microseconds, and a final-state checksum. Contact/trigger counts exclude exit events; dense counts include contacts with the five static boundaries. The checksum is a diagnostic, not a collision-correctness assertion. The benchmark fails if final sphere positions or linear velocities are non-finite. Command-line bounds reject zero measured steps and excessive scene sizes.

Sleeping bodies are set up once, before warmup. At the default duration, the 16 moving spheres remain separated from the sleeping ones. Changing duration or scene size may change the workload. Warmup advances the physics, so it also selects the physical interval being measured.

## Reproduce the comparison

With `RP3D_COMPILE_TESTS=ON`, configure two otherwise identical Release builds:

```sh
cmake -S . -B build-world-scalar -DCMAKE_BUILD_TYPE=Release \
  -DRP3D_COMPILE_TESTS=ON -DRP3D_SIMD_ENABLED=OFF
cmake -S . -B build-world-simd -DCMAKE_BUILD_TYPE=Release \
  -DRP3D_COMPILE_TESTS=ON -DRP3D_SIMD_ENABLED=ON
cmake --build build-world-scalar --target sphere_world_benchmark --parallel
cmake --build build-world-simd --target sphere_world_benchmark --parallel
./build-world-scalar/test/sphere_world_benchmark > scalar.csv
./build-world-simd/test/sphere_world_benchmark > simd.csv
```

Run timed processes sequentially on an otherwise idle machine, repeat in reverse order, and compare per-scene distributions and contact counts. Do not run builds or the two benchmarks concurrently. On multi-configuration generators, build with `--config Release` and use the executable under `test/Release/`.

For Wasm, use `emcmake cmake` to configure each build, set `-DRP3D_WASM_SIMD_ENABLED=ON` in **both**, and execute the generated `.js` programs with the same Node version. That isolates explicit batching from compiler autovectorization. Separately compare a build with the Wasm feature disabled if assessing the compiler flag itself. A Node measurement is not a browser measurement.

Select a smaller or longer run with, for example:

```sh
./build-world-simd/test/sphere_world_benchmark --scene dense --side 6 \
  --warmup 180 --steps 600 --repeats 5
```

`--scene` accepts `all`, `dense`, `sparse`, `triggers`, or `sleeping`. `--help` lists the defaults.

## Phase attribution

Use a **separate** build with `-DRP3D_PROFILING_ENABLED=ON` and the same other settings. The benchmark resets the existing profiler after warmup and writes inclusive phase totals, normalized per measured step, to stderr:

```sh
./build-world-profile/test/sphere_world_benchmark --repeats 1 \
  > profiled-world.csv 2> phases.csv
```

Each phase row includes its full tree path and call count. Parent timings include their children: do not add parent and child rows together. The profiler adds timing calls in some frequently executed routines; use this build to locate costs, not to claim production speedups. World creation may create an empty `rp3d_profiling_sphere_world_benchmark.txt` through the library's default profiler destination; the benchmark removes that destination and emits its own phase CSV.

## Interpretation

The sphere kernel is only one part of a world step. In a local profiled dense-pile run, contact solving and initialization accounted for more than half the total step, whereas narrow phase also included contact processing and manifold reduction beyond the sphere kernel itself.

Small floating-point differences can change a pile's later contact network, so scalar and SIMD checksums and contact counts may diverge even when direct contact regression tests pass. Compare workloads and stability as well as timings; these scenes do not establish cross-platform determinism or a general full-engine speedup. Keep explicit SIMD opt-in until measurements on the consuming application's scenes justify it.

The [kernel configuration and numerical tests](SphereSphereSIMD.md) document the backend and correctness coverage. CI builds and smoke-runs this benchmark on native platforms and Node without setting performance thresholds on shared runners.

## Local results (2026-09-22)

Measured on the development ARM64 macOS host with Apple Clang 21.0.0, Emscripten 5.0.1, and Node 26.7.0, Release float builds, profiling off, default benchmark settings. Both Wasm builds used `-msimd128`; only explicit sphere batching differed. Two process runs per variant, each with three fresh-world repetitions, reversed scalar/SIMD order between runs. Figures are medians of the six per-world median/p95 estimates, **not** a pooled p95. [Raw world results](benchmarks/sphere-world-2026-09-22.csv) preserve counts and checksums.

| Platform | Scene | Scalar median / p95 (µs) | SIMD median / p95 (µs) |
| --- | --- | ---: | ---: |
| Native ARM64 | Dense pile | 1061.83 / 1235.17 | 1057.66 / 1228.46 |
| Native ARM64 | Sparse near misses | 34.63 / 101.81 | 34.71 / 100.75 |
| Native ARM64 | Triggers | 254.29 / 484.92 | 276.00 / 479.67 |
| Native ARM64 | Mostly sleeping | 1.67 / 3.65 | 1.71 / 4.38 |
| Wasm / Node | Dense pile | 1188.72 / 1375.04 | 1175.30 / 1355.40 |
| Wasm / Node | Sparse near misses | 47.13 / 118.52 | 47.39 / 120.50 |
| Wasm / Node | Triggers | 351.01 / 662.15 | 347.49 / 611.92 |
| Wasm / Node | Mostly sleeping | 2.48 / 6.04 | 2.42 / 5.96 |

These runs show no substantial full-world gain. The native trigger median is about 8.5% slower despite a faster isolated trigger kernel; that discrepancy is unresolved. In a separate profiling comparison it disappeared (mean total step approximately 324 µs scalar vs 323 µs SIMD, narrow phase 70.4 vs 69.9 µs). The instrumented build therefore does not explain the uninstrumented regression. Do not dismiss it or enable SIMD by default based on the kernel numbers.

Native dense-pile mean contact counts were 1066.94 scalar versus 1056.68 SIMD, with different final checksums. Wasm contact counts were 1038.54 for both variants with matching printed checksums. Each variant reproduced its own checksum across repetitions; a matching checksum alone does not prove equal body states. Sparse scenes reported zero contacts/triggers, trigger scenes 1344 active overlaps, and sleeping scenes ended with 16 of 512 spheres awake in all runs.

The separate native SIMD profile measured approximately 1122 µs per dense step, of which 631 µs (56%) was solving and initialization, and 209 µs (19%) was narrow phase including contact processing. [Raw inclusive phase results](benchmarks/sphere-phases-2026-09-22.csv) include the dense run and the separate trigger comparison. Further full-world optimization should start by investigating the solver/preparation costs and the uninstrumented trigger discrepancy. These measurements do not justify another collision kernel solely on expected world-step gains.
