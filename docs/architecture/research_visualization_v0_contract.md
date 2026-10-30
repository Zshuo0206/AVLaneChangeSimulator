# AV Lane Change Simulator
## Research Visualization V0 Contract

Status: FROZEN / PASS, as confirmed by the current project baseline. This is a thin
trajectory-analysis and research-figure layer, not a new theory phase.

## Purpose and responsibility

The layer collects the frozen single-state analyses along a simulation and
displays physical vehicle motion, current execution capability, finite
boundaries, signed reserves and final mechanism memberships. Existing
[P0.2.5](p0.2.5_simulation_engine_contract_v0.1.md),
[P0.2.6](p0.2.6_theory_state_builder_contract_v0.1.md),
[P0.2.7](p0.2.7_current_capability_contract_v0.1.md) and
[P0.2.8](p0.2.8_reserve_diagnostics_contract_v0.1.md) behavior is unchanged.

```text
runSimulation -> analyzeTrajectory -> plotResearchSummary
```

Analysis creates no figure. Plotting never invokes buildTheoryState,
evaluateCapability, evaluateCapabilityDiagnostics or runSimulation.

## Trajectory analysis API and inputs

```matlab
analysis = avlc.analysis.analyzeTrajectory(scenario, result);
```

Scenario must be the matching experiment definition and result must be its
SimulationResult. Scenario validation is reused. Local indexing validation
requires a finite strictly increasing double time column starting at zero,
four equally long column state histories, metadata.numSteps = N and N EGO
controls. Misaligned or malformed indexing raises
`AVLC:InvalidTrajectoryResult`; no dynamics or fixed-grid construction is
duplicated. Individual states are validated by the existing builder.

For every k=1,...,N+1 the adapter calls, in order:

```matlab
z = avlc.analysis.buildTheoryState(scenario, result.states.EGO(k), ...
    result.states.FO(k), result.states.RO(k), result.states.RT(k));
cap = avlc.analysis.evaluateCapability(scenario, z);
d = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
```

The adapter contains no theoretical formulas. Existing validation errors
propagate unchanged. In particular, all samples must satisfy the frozen
capability domain y>=0,w>=0; an unsupported sample raises the existing
`AVLC:CapabilityDomainViolation`. It is not skipped, extrapolated or given
invented capability values. Negative gaps and reserves retain the existing
branch semantics.

## Analysis output and time-index contract

The output is an ordinary struct with these groups in construction order:

| Field | Collected data |
| --- | --- |
| time | result.time, unchanged |
| z | All eight buildTheoryState fields |
| abort | All cap.abort fields, then all d.abort fields |
| completion | All cap.completion fields, then all d.completion fields |
| joint | All cap.joint fields |
| reserves | All d.reserves fields: rO,lT,rT |

Every leaf history is an (N+1)-by-1 column. Numeric fields remain doubles,
flags remain logicals and currentBranch remains a string column. Time index
k identifies exactly the physical and theoretical sample at result.time(k).
Sample 1 is the supplied simulation initial state; sample N+1 is the final
state. There is no terminal control and no dropped initial/final sample.
Diagnostic fields are merged into the corresponding groups without storing
duplicate copies or introducing empty future compatibility fields.

Values are copied pointwise from the existing calls, including every active
flag in a tie. NaN stays NaN and Inf stays Inf; no filling, interpolation,
nearest/previous replacement, smoothing, clipping or sentinel conversion
occurs. Inputs are not modified.

## Research figure API and panels

```matlab
fig = avlc.visualization.plotResearchSummary(scenario, result, analysis);
% Noninteractive rendering for tests or export:
fig = avlc.visualization.plotResearchSummary(scenario, result, analysis, Visible="off");
```

The inputs must describe the same trajectory. The plot checks time/history
alignment before creating a figure; mismatch raises
`AVLC:InvalidResearchSummaryInput`. It displays one ordinary static MATLAB
figure, with no callbacks, slider or application state. It writes no files.

1. **Left: physical road and vehicles.** Horizontal screen axis is lateral y
   and vertical screen axis is longitudinal x, both in m, with equal metric
   scaling. Original lane center is 0 and target center is Road.laneWidth.
   Colored paths come directly from result.states; circles mark initial
   positions and rectangles show final positions. Vehicle width spans
   lateral y and length spans longitudinal x around the geometric center.
   No position is reconstructed from theory gaps. Four roles only: EGO/FO/RO/RT.
2. **Upper right: sampled current capability.** Three separate stair rows
   show abort.feasible, completion.feasible and joint.dualCapable. For visual
   separation only, row values are offset encodings of the unchanged 0/1
   samples; labeled ticks identify each row's 0 and 1. The stair transitions
   occur at the recorded sample times without filtering.
3. **Middle right: lateral state and boundaries.** Raw y, YO, YTin and YTout
   are plotted against time. NaN values break curves; missing intervals are
   never bridged or replaced.
4. **Lower right: signed reserves.** Raw rO,lT,rT are separate curves with a
   zero reference line. Negative values remain visible. They are not combined
   into an overall safety margin.

The three right panels have linked time limits and one shared time scale.
The final sample's abort active set, completion-upper set and current time
branch are annotated. All simultaneous memberships appear (e.g. R+G or I+II).
Undefined finite active sets are labeled undefined, not assigned a mechanism.
Plotting does not modify any analysis history.

## Reproducible example and verification

```matlab
run("examples/run_research_visualization_v0.m")
```

The script locates src from its own path and restores the previous MATLAB
path. It leaves researchScenario, researchResult, researchAnalysis and
researchFigure in the workspace. It uses ordinary frozen creators, constant
Ego control (ax=0,ay=0.5), dt=0.05 s and duration 4 s: 80 intervals and 81
state samples. This trial exposes changing boundaries, sampled capability
transitions, undefined finite-window intervals and signed reserves without
introducing a controller or a new model.

Focused tests check all-history alignment, exact pointwise equality,
initial/final inclusion, hand-calculated reserves, NaN/Inf histories,
mechanism flags and indexing/domain failures. Plot tests create invisible
figures, inspect data/physical geometry and close them; there is no pixel
regression suite. Manual inspection of the example checks lanes, positions,
time alignment, broken NaN curves, boolean transitions and reserve signs.

## Non-goals and future P0.2.9

dualCapable still means current coexistence of execution capabilities.
Gamma still means frozen forward-boundary geometry. A displayed boolean
transition describes this sampled trial, not a new prediction of dynamic
loss order or positive-time shared preservation.

No GF1/GF2/GF3, Type-L/Type-X, localPreservable, shared-control compatibility,
reserve derivatives, control-set intersections, global horizon, tau_dual,
controller, animation or GUI framework is implemented. Future P0.2.9 can
extend the simple histories after its own contract is defined; no placeholder
compatibility fields are added here.
