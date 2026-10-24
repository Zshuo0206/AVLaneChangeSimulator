# AV Lane Change Simulator
## Phase 0 Software Requirements v0.1

### Purpose

This project is a two-dimensional MATLAB research simulator for autonomous-vehicle lane-change capability research.

Phase 0 exists to provide a stable, modular and reproducible numerical simulation platform before the final lane-change capability theory is integrated.

Phase 0 does NOT implement the final theoretical definitions of:

```text
K_T
K_O_abort
commitment boundary
capability margin
```

### Core research scene

Phase 0 uses two straight parallel lanes:

```text
Original Lane
Target Lane
```

and four core vehicles:

```text
EGO — ego vehicle
FO  — front vehicle in original lane
RO  — rear vehicle in original lane
RT  — rear vehicle in target lane
```

EGO is the only vehicle that requires active lateral lane-change motion in Phase 0.

The architecture may later support additional vehicles such as FT, but they are outside the mandatory Phase 0 scope.

### Coordinate system

Use:

```text
+x = road forward direction
+y = from original lane toward target lane
```

Vehicle position `(x,y)` refers to the vehicle geometric center.

This convention is frozen for Phase 0.

### Units

All internal quantities use SI units:

```text
position      m
vehicle size  m
velocity      m/s
acceleration  m/s^2
time          s
angle         rad
```

Display layers may later convert units, but internal calculations must remain SI.

### Unified vehicle state interface

The Phase 0 software state interface is:

```text
x
y
vx
vy
ax
ay
```

equivalent to:

```text
X_i = [x_i, y_i, vx_i, vy_i, ax_i, ay_i]^T
```

Not every dynamics model must actively use every field, but the software interface retains all six fields.

### Vehicle geometry

Every vehicle has at least:

```text
length
width
```

Vehicles are represented geometrically as rectangles for 2-D visualization.

Phase 0 does not model:

```text
pitch
roll
suspension
tire deformation
3-D vehicle geometry
```

### Surrounding-vehicle motion

Phase 0 must eventually support:

```text
constant velocity
constant acceleration
piecewise acceleration
```

for surrounding vehicles.

### Ego dynamics

The final ego vehicle dynamics are NOT frozen.

The architecture must allow the ego dynamics model to be replaced without rewriting the simulation engine.

Potential later models may include:

```text
simple kinematics
point-mass model
bounded longitudinal/lateral model
bicycle model
research-specific final dynamics
```

These are possibilities, not Phase 0 commitments.

### Ego placeholder lane-change trajectory

A smooth predefined lateral trajectory may be used only as a Phase 0 software-testing placeholder.

A quintic polynomial trajectory is permitted.

This placeholder must not be described as the final lane-change control law.

### Simulation time

Phase 0 uses a configurable fixed simulation time step:

```text
dt
```

and configurable simulation end time:

```text
endTime
```

Simulation time and animation playback time must remain separate.

Changing animation speed must never modify numerical simulation results.

### Dynamic gaps

Phase 0 must eventually compute at least:

```text
d_FO
d_RO
d_RT
```

Because `(x,y)` is the vehicle geometric center, longitudinal bumper-to-bumper gap calculations use center position and vehicle lengths.

For example:

```text
d_FO = x_FO - x_EGO - (L_FO + L_EGO)/2
```

Equivalent consistent definitions shall be used for rear gaps.

The software must not mix center distance, bumper gap and axle-reference distance.

### Relative velocities

Phase 0 must eventually compute relative longitudinal velocities associated with FO, RO and RT.

The exact sign convention must be explicitly frozen before implementation.

### Scenario

A Scenario describes the world before and during a simulation and must eventually include:

```text
road
vehicles
initial conditions
events
```

Scenario logic must not directly perform numerical integration.

### Simulation engine

The simulation engine must eventually:

```text
validate input
initialize state
build simulation time
advance vehicle states
call dynamics modules
compute analysis quantities
record results
process simple events
return SimulationResult
```

The simulation engine must not:

```text
draw GUI
draw figures
hard-code scenario values
implement final capability theory
perform file UI operations
```

### SimulationResult

Simulation results must eventually include at least:

```text
metadata
time
vehicle states
gaps
relative velocities
events
capability
```

The `capability` field is reserved during early Phase 0.

### Capability Evaluator

An independent Capability Evaluator interface is reserved.

Phase 0 must not guess or implement formulas for:

```text
K_T
K_O_abort
commitment boundary
capability margin
```

### Visualization

Visualization must consume SimulationResult.

Visualization must not independently advance vehicle dynamics.

Animation is a replay of numerical results, not a second simulation.

### Interactive and batch execution

The core simulation must be runnable without GUI.

The architecture must eventually support both:

```text
interactive simulation
batch simulation
```

so later research can perform parameter sweeps and Monte Carlo experiments.

### Data export and reproducibility

Simulation results must later preserve enough metadata to recover:

```text
scenario
model
parameters
simulation configuration
software version
```

Identical deterministic inputs must produce identical deterministic outputs.

### Automated tests

Core numerical functions must be testable without GUI.

Phase 0 tests will eventually include:

```text
constant velocity analytic test
constant acceleration analytic test
gap calculation test
stationary vehicle test
time-vector test
data recording test
```

### Explicit Phase 0 exclusions

Phase 0 excludes:

```text
final K_T implementation
final K_O_abort implementation
final commitment-boundary implementation
HJ PDE solver
MPC
full trajectory planner
SUMO
CARLA
large traffic-flow generation
IDM
MOBIL
traffic lights
intersections
urban road networks
camera simulation
LiDAR simulation
radar simulation
localization uncertainty
perception uncertainty
3-D vehicle dynamics
collision physics engine
high-fidelity tire models
```

### Phase 0 completion philosophy

Phase 0 is complete when the scientific simulation infrastructure is correct, modular, testable and reproducible.

GUI appearance is not the completion criterion.

The software is:

```text
a two-dimensional numerical simulation foundation
for autonomous lane-change capability theory research
```

not a general-purpose microscopic traffic simulator.
