# Market and technology review — 2026-09-06 (America/Los_Angeles)

## Context and scope

The starting workspace was empty: no source, README, tests or Git history; `.git` was an empty protected placeholder. The user-provided Chinese specification is a greenfield design, not evidence of implemented robotics. Its phase-one acceptance permits a complete discrete simulator before ROS 2/Gazebo integration.

Current profile and banks were read locally, using history.jsonl and the local thread-history SQLite database as path clues. Authoritative bank location is `Desktop/Find work/jd-resume-tailor/data/`. At inspection it contained 10 experiences: clinical backend work, AsyncFlow, VelaServe, Quorum Market, resume tooling, GEMM lab, FleetRelay, MaterialFlow, AdPulse and LakeBridge. These already cover databases, durable queues, control planes, cloud manifests and ML data/serving qualification. This project adds C++ graph algorithms, shared-space scheduling, bounded planning and simulation validation; it does not duplicate FleetRelay's device-delivery service.

The current profile lists NEU CS MS, expected May 2027, and a software-engineering bachelor's degree. The source specification's mathematics bachelor's description conflicts with it; no education field is changed here. No applicable AGENTS.md or readable `Project Experience Bank Generator.md` was found in the project, Desktop, Documents, Downloads or the relevant accessible history. No separate memories directory was available. Existing bank schemas and the user's explicit evidence instructions govern the update.

## Official job sample

These are directional samples, not a labor-market survey. Retrieval date: **2026-09-06 PDT / 2026-09-07 UTC**. Pages were searched for recent roles, prioritizing early career and the preceding 3–6 months. Original posting dates could not be reliably established; the official Ashby posting API returned HTTP 403. Search crawl dates are **not** posting dates. Do not claim this sample consists entirely of roles posted in the last six months.

| Official source | Level/location and fit | Observed requirements | Date/evidence limitation |
|---|---|---|---|
| [Eventual — Software Engineer, New Grad](https://jobs.ashbyhq.com/eventualcomputing/becb5675-3480-4d2b-b126-2acad40fd088) | San Francisco; within roughly a year of graduation, suitable stage to monitor for May 2027 | Systems fundamentals; Python/Rust/C++/Go; workload profiling and benchmarking; projects and open source | Full official posting text accessible; original date unavailable |
| [Foxglove — Software Engineer, New Grad](https://jobs.ashbyhq.com/foxglove/def61478-8b86-43e5-b27b-be7b76900449) | San Francisco; title says new grad | Robotics infrastructure, C++/Rust/TypeScript, constrained execution, observability and replay | Full posting accessible; salary bands incongruously say senior/staff, so neither compensation nor exact leveling is inferred; date unavailable |
| [Ellipsis Labs — Campus, Intern & New Grad](https://jobs.ashbyhq.com/ellipsislabs/caa49297-d5a3-4d03-b8c9-bdc7b77fb0bb) | New York; campus/intern track matches student stage | Scoped design-through-monitoring ownership, algorithms, tested code, reliability, performance; Docker/CI/cloud are preferred exposure | Official indexed text accessible; original date unavailable |
| [Amazon — Fleet Management 10499160](https://www.amazon.jobs/en/jobs/10499160/software-development-engineer-fleet-management) | North Reading, MA; 3+ years non-internship development and 2+ years architecture, **stretch only** | Task allocation, fleet-resource state, fault tolerance, simulation/emulator and live-robot validation | Official page accessible; original date unavailable; not an entry-level eligibility claim |

Additional 2027 Whatnot and Ellipsis listings were located, but direct pages returned JavaScript-only bodies; they are not counted as full-text requirement evidence. Nuro's official career page linked early-career work, but the specific JD could not be independently retrieved; third-party descriptions were excluded.

Repeated within this small sample: programming/CS fundamentals (3 early-career samples), maintainable systems work (all three), and performance or observability (all three). Eventual's Rust/data engine emphasis and Ellipsis's Solana/DeFi interest are company-specific; neither justifies adding those stacks here. ROS and robotics telemetry are Foxglove-specific within the early-career sample, corroborated as domain relevance by Amazon's experienced fleet role. No popularity or universal-hiring-demand claim follows from GitHub activity.

## Priorities and acceptance

1. C++ scheduling core: deterministic nearest/batch/congestion allocation; two-leg tasks; time-expanded A* and endpoint/edge reservations. Accept via boundary regressions and an independent execution oracle.
2. Reliability update: atomic reservation replacement, bounded search/queue/candidate work, typed failure outcomes, pause/closure lifecycle and no-progress signals. Accept via injected faults, task conservation and rejected corrupt traces.
3. Evaluation infrastructure: immutable run directories, completion marker, bounded subprocess concurrency and timeout, source/binary fingerprints, identical offered workloads and zero-lambda equivalence. Accept via 90 paired inputs per policy and all raw results, including negative outcomes.
4. Delivery: reproducible CMake/CTest, hosted CI and trace-derived demo; bank bullets cite only verified artifacts. No production, cloud, GPU/ML execution or physical-robot claim.

## Technology decision

See [upstream.md](upstream.md) for exact official repository versions, compatibility and migration evidence. This is a new dependency-light C++20 implementation, not a framework upgrade. The kernel uses only the C++ standard library; experiments use Python's standard library. CMake's minimum is 3.20, preserving Ubuntu 24.04 toolchain compatibility without requiring the newest release. Local CMake 4.2.3/GCC 15.2.0 are recorded in each run, and hosted CI supplies a second environment.

The next ROS stage should use Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic and the Jazzy Nav2 line. The local host is Ubuntu 26.04, and ROS 2/Gazebo are not installed. Installing mismatched binaries or claiming waypoint conversion is safe multi-robot execution would not complete that stage. Its acceptance requires a compatible isolated runtime, namespaces/TF/sim clock, action cancellation feedback, conservative resource locks, and actual 1→3 robot runs.
