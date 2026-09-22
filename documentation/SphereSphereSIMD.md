# Sphere–sphere SIMD batches

The optional sphere narrow-phase path batches contact arithmetic across four single-precision pairs. It supports Wasm SIMD128, ARM64 NEON, and x86 SSE2 without changing public math types, contact ordering, or collider storage.

`RP3D_SIMD_ENABLED` defaults to **OFF**: kernel improvements do not necessarily improve complete simulation steps, and small batches can still pay extra dispatch costs. Measure a representative application before enabling it.

## Build

Native:

```sh
cmake -S . -B build-simd -DCMAKE_BUILD_TYPE=Release \
  -DRP3D_COMPILE_TESTS=ON -DRP3D_SIMD_ENABLED=ON
cmake --build build-simd --parallel
ctest --test-dir build-simd --output-on-failure
```

WebAssembly, with Emscripten tools on `PATH`:

```sh
emcmake cmake -S . -B build-wasm-simd -DCMAKE_BUILD_TYPE=Release \
  -DRP3D_COMPILE_TESTS=ON -DRP3D_SIMD_ENABLED=ON -DRP3D_WASM_SIMD_ENABLED=ON
cmake --build build-wasm-simd --parallel
ctest --test-dir build-wasm-simd --output-on-failure
```

The Wasm option requires Emscripten and propagates `-msimd128` to final consumers through the CMake target. A SIMD Wasm binary needs a runtime supporting SIMD128; provide a separately compiled baseline binary when older runtimes must be supported. Threads are not required. Test and benchmark executables allow Wasm heap growth; that setting is not imposed on library consumers.

For a scalar comparison, configure another build with `RP3D_SIMD_ENABLED=OFF`. Keep `RP3D_WASM_SIMD_ENABLED` unchanged to isolate explicit sphere batching from general compiler autovectorization. Disabling explicit kernels does not disable the compiler's own vectorization.

Manual builds select a backend from compiler target macros unless `RP3D_DISABLE_SIMD` is defined. Compile with `-msimd128` to select the Wasm backend. Double precision and unsupported targets always use the scalar implementation; no AVX2 requirement or runtime x86 dispatch is introduced.

## Behavior and limits

- Overlap classification uses the original scalar squared-distance expression, including tangency and the coincident-center threshold. Rejected pairs do not incur SIMD packing or boundary-repair costs.
- Four ordinary overlapping reporting pairs share a packet for penetration, the three vector normalizations, and radius scaling. They need not be adjacent: misses and triggers can occur between them.
- Leading triggers use a small scalar loop. Triggers inside a reporting batch are handled directly without breaking a pending contact packet.
- Batches smaller than four, incomplete contact packets, and coincident contacts use scalar contact construction. Pending contacts are emitted before a coincident contact to preserve pair order.
- Local transforms retain scalar arithmetic. Vectorizing these expressions changed rounding under cancellation at large world coordinates in testing, so they remain on the existing path.
- The strict overlap and positive-penetration checks, coincident-center normal, pair IDs, and contact reporting order are retained. No approximate reciprocal or square root is used.
- Numerical equivalence is tested within floating-point tolerance; bitwise-identical simulation trajectories across architectures/builds are not promised.

## Benchmark

The benchmark is excluded from normal builds. With tests configured, build and run it explicitly:

```sh
cmake --build build-simd --target sphere_sphere_benchmark --parallel
./build-simd/test/sphere_sphere_benchmark
```

For Wasm, run `node build-wasm-simd/test/sphere_sphere_benchmark.js`. Compare two complete builds, rather than calling the scalar algorithm once per pair, which would unfairly add function-call overhead to the scalar reference.

The executable reports the median of seven measured samples after one warmup sample. It includes input gathers, contact emission, and resetting batch results between calls. Shape construction, broad phase, and world solving are outside the timed region. Cases cover 3/4/7/64/1024 pairs and separated, mixed, all-contact, and trigger workloads.

### Original development measurements

Measured on the development ARM64 macOS host, Release builds, Apple Clang 21.0.0 and Emscripten 5.0.1. Wasm measurements used Node 26.7.0 on the same host. Both Wasm variants enabled `-msimd128`; only explicit sphere batching differed. Values below are one local comparison, not a cross-machine performance guarantee.

| 1024-pair workload | Native scalar → batches (ns/pair) | Wasm scalar → batches (ns/pair) |
| --- | ---: | ---: |
| All contacts | 17.26 → 14.43 (**1.20×**) | 39.89 → 21.52 (**1.85×**) |
| Half overlapping | 9.71 → 11.11 | 23.00 → 19.28 |
| Separated | 3.11 → 3.76 | 5.83 → 6.62 |
| Triggers | 3.58 → 4.78 | 6.85 → 8.15 |

These original measurements describe the adjacent-pair implementation in `ade22a01`. The subsequent implementation packs only overlapping reporting pairs and uses a dedicated trigger prefix. See the follow-up measurements below and the [full-world benchmark](SphereWorldBenchmark.md). These timings measure narrow-phase batches, not full simulation speedups; browser engines and native x86 hardware need their own timing runs.

### Contact-packing follow-up (2026-09-22)

Same toolchains and host, with fresh scalar comparisons. Each figure below is the median of five process results, alternating scalar/SIMD execution order. Each process reports seven measured samples after warmup. Both Wasm builds retain `-msimd128`. [Raw kernel results](benchmarks/sphere-kernel-2026-09-22.csv) include every batch size and process result.

| 1024-pair workload | Native scalar → batches (ns/pair) | Wasm scalar → batches (ns/pair) |
| --- | ---: | ---: |
| All contacts | 17.89 → 15.60 (**1.15×**) | 38.48 → 19.53 (**1.97×**) |
| Half overlapping | 9.66 → 8.99 (**1.07×**) | 22.94 → 12.38 (**1.85×**) |
| Separated | 3.15 → 3.30 | 5.62 → 5.58 |
| Triggers | 3.58 → 3.36 | 6.60 → 4.47 |

Packing actual contacts removes the earlier large sparse/trigger kernel penalties and improves half-overlapping batches. Native all-contact throughput is slightly lower than the original adjacent-pair path; this trades peak throughput for broader workload coverage. Native separated pairs still show a small penalty, and sub-four-pair dispatch can remain slower. The Wasm all-contact process results ranged from 19.32 to 28.64 ns/pair, so the median is not a performance guarantee. Full-world timings are mostly unchanged and one native trigger workload remains slower; see the [world results and limitations](SphereWorldBenchmark.md#local-results-2026-09-22).

## Validation

Regression tests run through the existing `tests` executable. They cover analytic contacts, every four-lane collision mask, nonzero range starts, empty ranges, small batches and tails, triggers, varied radii/rotations, coincident centers, adjacent floating-point values around decision boundaries, oblique near-tangent pairs, and large-coordinate local contacts. Interleaved reporting masks exercise nonadjacent contacts, trigger prefixes, exceptional contacts, and incomplete packets. Packet output is compared with one-pair scalar calls, with independent analytic checks for expected geometry and collision flags.

Local validation passed native ARM64 Debug/Release, explicit scalar, double precision, ASan/UBSan, x86-64 SSE2 execution through Rosetta, and Wasm SIMD/scalar execution through Node. Windows/MSVC, native Linux, browsers, and other ARM devices were not executed locally. The CI workflow now configures Linux/GCC, ARM64 macOS, and Windows/MSVC with explicit kernels enabled/disabled and float/double builds. A Wasm job executes the suite under Node with explicit kernels enabled/disabled and a separate non-SIMD module. These new hosted jobs have not been run locally; the workflow passes `actionlint`.
