## Path Planning Algorithm Scalability Assessment

Role: Software Architect (Automotive / MBSE)
Scope: Current MATLAB A* + Motion Primitives implementation (files: `Pathplanning_Astar.m`, `Add_MP_toworkspace.m`, `g_cost.m`, `h_cost.m`, obstacle & helper scripts) – focus on scalability, extensibility, safety, performance, and integration readiness.

---
### 1. Executive Summary
The current solution is a research-grade prototype suitable for demonstrating motion–primitive–based A* docking in a constrained yard. It will not scale directly to: larger depots, higher discretization resolution, dynamic obstacles, multi-vehicle scenarios, tighter real‑time constraints, or variant vehicle platforms. Core limitations stem from: (1) monolithic script architecture, (2) ad-hoc data structures, (3) heuristic admissibility violations, (4) static, hard‑coded motion primitive library, (5) lack of configuration, (6) absence of verification & traceability hooks, and (7) inefficient search frontier management.

---
### 2. Intended Scaling Directions (Target Use Cases)
| Dimension | Expected Evolution | Current Readiness |
|-----------|-------------------|-------------------|
Map size & resolution | Larger yards, finer grid (≤0.25 m) | Poor (arrays & O(n) scans grow quickly) |
Vehicle variants | Different lengths, multi‑trailer, forklifts | Hard (library tightly bound to one kinematic set) |
Dynamic / moving obstacles | Predictive avoidance, replan-on-the-fly | Not supported (static polygon test only) |
Multi-agent coordination | Fleet path negotiation | Not supported |
Real-time guarantees | Bounded planning latency (e.g. <100 ms incremental updates) | Not achievable with current design |
Regulatory / safety compliance | ISO 26262 work products, traceability | No foundations |
Simulation fidelity | SIL/HIL co-simulation, timing-in-loop | Weak (script side‑effects, globals) |
Anytime / replanning | Partial plan delivery, interruptions | Not supported |

---
### 3. Key Complexity Drivers & Risks
1. State Space Explosion: Discretization of θ (orientation) and γ (articulation) combined with translational grid expansion creates multiplicative growth; no pruning or dominance checks.
2. Motion Primitive Library Growth: Pre-rotated duplication (rotating base set for each θ) scales linearly with angle samples; memory & load time increase rapidly.
3. Non-Admissible / Inconsistent Heuristics: Heuristic scaling factor (2.5) + heavy penalties (×1000) can break A* optimality and cause search thrash.
4. Reverse / Forward Duplication: Separate logic blocks for direction introduce branching and code duplication; maintenance overhead.
5. O(n) Frontier Operations: Open list is a simple MATLAB array; every expansion performs full cost scans – becomes a bottleneck.
6. Global State & Side Effects: Globals (`f_state`, etc.) and script execution order fragile → integration / test complexity.
7. Hard-Coded Scenario Geometry: Dock coordinates & polygon arrays inline; changing scenario requires code edits instead of data/config ingestion.
8. Mixed Units & Conversion Noise: Frequent rad↔deg conversions with rounding risks inconsistent hashing of states.
9. Lack of Abstraction Layers: Environment model, search algorithm, motion primitive generation, and visualization are entangled.
10. Limited Collision Checking Model: Only vertex polygon inclusion; no swept volume / vehicle footprint rotation per segment at fine resolution; risk of false negatives.
11. No Dynamic Obstacle Model: Cannot integrate predicted trajectories or time dimension; path length ‘time’ surrogate insufficient for spatiotemporal planning.
12. Memory Footprint Risk: Arrays (`t_x`, `t_y`, etc.) grow unbounded; no reclamation or upper bound checks.
13. Heuristic Zone Logic Complexity: Ad hoc rectangular zone logic & virtual obstacle insertion increases branch conditions → harder to generalize.
14. Rigid Primitive Metadata: Library lacks curvature, max steering rate, energy/time cost, risk scores → constrains multi-objective optimization.
15. Unclear Failure Modes: No structured exit reasons (timeout, no path, infeasible start); loop halts only when final zone check passes.
16. Replanning Latency: Entire search restarts; no incremental / heuristic repair (e.g., D* Lite, Anytime Repairing A*).
17. Observability: No logging hooks, telemetry, or performance counters for profiling in SIL/HIL loops.
18. Testability: Pure scripts without function signatures hamper unit tests & parameterized regression suites.
19. Determinism: Dependence on MATLAB global state & dynamic workspace lowers reproducibility.
20. Safety / MBSE Gap: No linkage to formal requirements, hazards, or SysML/UML artifacts; traceability missing.

---
### 4. Detailed Issue Catalogue
#### 4.1 Algorithmic / Search Strategy
- A* frontier managed via unsorted arrays → O(E) min extraction (should be binary heap / Fibonacci heap / pairing heap or priority queue abstraction).
- No tie-breaking (e.g., favor greater g or lower h) leads to expanded node order variance.
- Heuristic penalties (×1000) introduce non-monotonicity; may cause node re-expansions or suboptimal path acceptance.
- Forward and reverse expansion loops duplicate 80% of logic; increases bug surface.
- No pruning of states dominated by (x,y,θ,γ) with lower g-cost; closed list uniqueness check only on rounded (x,y) ignoring orientation & articulation.

#### 4.2 Motion Primitive Library
- Precomputed rotation inflates library size; runtime rotation with caching could reduce memory.
- Hard-coded path length & time arrays; no parameter variation (wheelbase, trailer length changes require full regeneration).
- No versioning / checksum for primitive sets → reproducibility risk when regenerating.
- Straight primitives artificially inserted with multiple lengths but heuristics unaware of variable temporal cost vs spatial cost.

#### 4.3 Cost Functions
- `g_cost` omits direction penalties (forward vs reverse) — unrealistic for yard operations where reversing may be slower / riskier.
- `h_cost` mixes contextual conditions; environment-specific logic (zone detection) is interwoven with generic distance heuristic.
- Admissibility not validated; coefficient 2.5 likely >1 causing overestimation.

#### 4.4 Data Structures & Memory
- Parallel primitive attributes kept in separate arrays (AoS vs SoA confusion); could migrate to struct arrays or tables for clarity.
- No hashing / indexing map (e.g., dictionary keyed by discretized (x,y,θ,γ)).
- Potential for floating rounding inconsistencies due to `ppround_1` after arithmetic.

#### 4.5 Collision & Environment Modeling
- Uses `InPolygon` per candidate; no caching or spatial index (R-tree / grid). Complexity grows with number of obstacles * candidate points.
- Vehicle shape not explicitly applied for each interpolated segment (only endpoints sometimes considered) risking corner clipping.
- Virtual obstacle logic embedded in planner (tight coupling).

#### 4.6 Code Quality / Maintainability
- Monolithic script `Pathplanning_Astar.m` mixes initialization, search loop, environment logic, and output.
- Extensive use of globals; hidden dependencies impede reuse (e.g., `global f_state`).
- Magic numbers (e.g., 1000 penalty, rectangle dims, heuristics coefficient 2.5) scattered.
- Commented legacy code left in place → noise.

#### 4.7 Real-Time & Performance
- No early exit heuristics (e.g., goal bounding) beyond rectangular region checks.
- No iterative deepening or anytime variant; cannot provide progressively improving paths.
- No timing guard / watchdog for real-time cycle deadlines.

#### 4.8 Integration & Toolchain
- Not encapsulated as MATLAB function or Simulink block with defined interfaces (inputs: map, start, goal; outputs: path, status, metrics).
- HIL/SIL deployment requires code generation readiness; current script not code-gen compliant (I/O, figures, dynamic workspace usage).
- No Python API layer for orchestration or batch scenario evaluation.

#### 4.9 Verification, Validation & MBSE Gaps
- Absence of requirement IDs, test vectors, and acceptance criteria mapping.
- No scenario library (parameterized test cases) with expected outcomes.
- Lack of KPIs logging (expansions, branching factor, open list max size) → cannot trend improvements.

#### 4.10 Safety & Reliability
- No safety constraints enforcement (e.g., maximum articulation angle rate, stability criteria).
- No fault handling (input out of bounds, unreachable goal, inconsistent primitive set).
- Penalty-based logic could mask infeasible geometry rather than raising explicit failure.

#### 4.11 Extensibility / Multi-Objective Potential
- Single scalar cost (distance) + heuristic; cannot easily extend to weighted objectives (time, energy, risk, emissions) without refactor.
- Reverse vs forward direction cost neutrality forbids optimization for operational efficiency.

#### 4.12 Observability & Logging
- No structured logging (JSON, MAT struct) of each expansion.
- Debug visualization requires enabling figure calls (slows execution, not headless-friendly).

---
### 5. Impact Priority Matrix
| Issue Cluster | Severity (Path to Product) | Effort (Refactor) | Notes |
|---------------|----------------------------|-------------------|-------|
Heuristic validity & penalties | High | Medium | Must ensure admissibility for optimality / predictability |
Data structure (open/closed) | High | Medium | Priority queue + hashed closed list |
Global state & script form | High | Medium | Modularization prerequisite for code-gen |
Motion primitive architecture | High | High | Needs regeneration pipeline & parameterization |
Collision model fidelity | High | High | Swept volume & time dimension for dynamic obstacles |
Configuration & scenario abstraction | Medium | Medium | External YAML/JSON/MAT config ingestion |
Logging & metrics | Medium | Low | Add instrumentation early |
Testing & MBSE traceability | High | Medium | Introduce requirements mapping harness |
Reverse/forward duplication | Medium | Low | Single expansion function with direction flag |

---
### 6. Recommended Refactor Roadmap (Phased)
Phase 1 (Stabilize Core)
1. Encapsulate planner as function: `[path, stats] = planPath(env, start, goal, options)`.
2. Introduce configuration struct & remove magic numbers.
3. Replace open list with priority queue (binary heap) and implement state hashing (x,y,θ,γ).
4. Separate heuristic strategies; validate admissibility (coefficient ≤1 if Euclidean). Parameterize penalties.
5. Consolidate forward/reverse expansion into reusable function.

Phase 2 (Performance & Extensibility)
6. Implement dominance pruning (discard states with higher g for same discrete tuple).
7. Lazy rotation of primitives at expansion time with memoization; add primitive metadata (curvature, time, feasible steering bounds).
8. Abstract collision checker; implement swept footprint and spatial index (grid binning or bounding volume hierarchy).
9. Add logging & profiling: expansions/sec, peak open size, memory, heuristic error.

Phase 3 (Advanced Capabilities)
10. Integrate dynamic obstacle prediction (time-indexed occupancy); extend state to include time dimension or reservation table.
11. Add cost plugins (energy, reverse penalty, risk) with weight vector; multi-objective (e.g., lexicographic or scalarization).
12. Introduce Anytime Repairing A* or Hybrid A*/State Lattice with smoothing post-processing.
13. Provide Simulink wrapper block & code-generation compliance (no figures, preallocated arrays, fixed-size options where possible).

Phase 4 (MBSE & Safety Alignment)
14. Link requirements (Req ID) to code modules & tests; create SysML parametric diagram mapping.
15. Hazard analysis: identify unsafe states (jackknife, collision margin violation) → integrate guards.
16. Verification suite: scenario regression harness (MATLAB + optional Python orchestrator for batch runs & statistical metrics).

---
### 7. Immediate Quick Wins
- Remove inline `figure` calls behind a verbosity flag.
- Centralize constants (dimensions, penalties, heuristic factors) into one config file.
- Add unit tests for `g_cost`, `h_cost` ensuring monotonicity & admissibility under chosen settings.
- Provide deterministic random seed (if stochastic features added later).
- Wrap `InPolygon` calls with memoization for repeated identical queries.

---
### 8. Proposed Target Architecture (High-Level)
Layers:
1. Input Layer: Scenario loader (map polygons, vehicle params, start/goal).
2. Primitive Provider: On-demand generator + cache keyed by (θ_i, γ_i, primitive_id, direction).
3. Planner Core: Priority queue A*/Hybrid A* engine with plugin heuristic and cost evaluators.
4. Collision & Feasibility Service: Footprint projection, dynamic occupancy query, kinematics constraints.
5. Post-Processor: Path smoothing / polynomial fitting / reverse maneuver minimization.
6. Interface Layer: MATLAB function API + Simulink block + (future) Python wrapper.
7. Telemetry & Logging: Structured emission of stats, events, diagnostics.
8. Verification Harness: Scenario test runner & MBSE trace links.

---
### 9. Risk if Unaddressed
- Performance cliffs on larger yards leading to planner timeouts.
- Non-deterministic path variations degrade validation confidence.
- Inability to certify or integrate into real-time stack (control & actuation delays not modeled).
- High cost of change when introducing new vehicle geometries.
- Hidden heuristic inadmissibility producing suboptimal or unsafe reversed maneuvers.

---
### 10. Summary
The current prototype succeeds as a proof-of-concept but sits at an architectural inflection point. Investing early in modularization, data structure efficiency, and admissible heuristics will unlock scalability and set the foundation for MBSE-aligned verification, safety analysis, and real-time deployment. The phased roadmap above should be initiated with Phase 1 tasks in the next iteration cycle.

---
Prepared by: Architect Mode
Date: 2025-09-21
