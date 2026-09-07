# Verification record

Final local source is fingerprinted in the held-out environment manifest and linked by the publication commit. All tests use actual compiled C++ executables. The separate [SIL record](sil-validation.md) covers actual local ROS 2 / Gazebo execution. No physical robot, hardware-in-the-loop device, Kubernetes/cloud deployment, GPU workload or production traffic was run.

| Verification | Result and evidence |
|---|---|
| C++20 Release build, warnings-as-errors | GCC 15.2.0, CMake 4.2.3; build succeeded |
| Initial discrete Release CTest | 3 test programs passed: 22 core cases, 4 CLI integration cases, 1 independent-oracle corruption case; [JUnit](evidence/release-tests.xml), [core output](evidence/core-tests.txt) |
| Initial discrete ASan + UBSan + LeakSanitizer | 3 programs passed outside ptrace sandbox; [JUnit](evidence/sanitizer-tests-final.xml). Original [sandbox failure](evidence/sanitizer-tests.xml) is retained separately |
| Injected early-service defect | Isolated buggy copy fails with future reservation conflict; fixed seed-91 run completes 21 tasks, no checked collision; [before/after](evidence/service-regression.json) |
| Paired allocator matrix | See [evaluation](evaluation.md): all successful runs must satisfy independent spatial, state/accounting, same-input and zero-lambda checks |
| Pause and temporary closure | Both applied, task accounting preserved; [fault analysis](failure-analysis.md) and raw [trace](evidence/recovery/trace.csv) |
| Published artifact revalidation | `python3 scripts/check_evidence.py` checks source/map and every decompressed artifact hash, then reruns the independent Python oracle; [record](evidence/archive-verification.json) |
| GitHub Actions | Ubuntu 24.04 workflow runs tests with and without sanitizers and a small paired matrix. Live execution status: [Actions](https://github.com/JDinSeattle/warehouse-fleet-lab/actions). It is separate from the local performance matrix |

The current native suite adds ten adversarial lease/watchdog/startup-retry cases: **37 cases across four CTest programs**, passing in [Release](evidence/sil/release-tests.log) and [ASan/UBSan/LeakSanitizer](evidence/sil/sanitizer-tests.log). Eight additional integration cases exercise the compiled allocator over actual ROS DDS services. Physics validation is documented separately from CI and the discrete 360-run experiment.

## Commands

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DFLEET_SANITIZE=ON
cmake --build build-sanitize -j4
ctest --test-dir build-sanitize --output-on-failure
python3 scripts/reproduce_service_regression.py
python3 benchmarks/run.py --out runs/fresh-held-out
python3 scripts/check_evidence.py
```

Local host: Ubuntu 26.04.1, Intel Core i9-13900K, 32 logical CPUs, 129,209,084 KiB RAM. The C++ core has no third-party runtime dependency. CLI/oracle verification used Python 3.14 (CTest discovered 3.14.7; command-line experiment environment records its own interpreter). Build configuration is Release, standard CMake GCC Release flags `-O3 -DNDEBUG`; sanitizers use Debug. The shared-host timings are descriptive, and reproducible procedure does not imply identical wall-clock results.
