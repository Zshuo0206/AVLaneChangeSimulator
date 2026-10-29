# Changelog

## 0.1.0

### P0.2.5

- Implemented the minimal fixed-step four-vehicle Simulation Engine
- Added constant Ego-control simulation
- Added synchronized role-to-dynamics orchestration
- Added deterministic trajectory result storage
- Frozen the N-control / N+1-state result convention
- Added fixed-grid validation and simulation-engine regression tests

### P0.2.4

- Added Control data contract
- Added exact constant-velocity propagation primitive
- Added exact Ego double-integrator propagation primitive
- Enforced frozen Ego acceleration bounds
- Preserved acceleration snapshot/control semantic separation
- Added dynamics regression tests and versioned contract documentation

### P0.2.3

- Added validated TheoryModelParameters data contract
- Added validated four-vehicle Scenario model
- Added Scenario-level vehicle ID and role consistency checks
- Integrated the frozen theory-to-software parameter contract into Scenario structure
- Reserved the event container without implementing event semantics
- Added regression and boundary tests for Scenario and theory parameters

### P0.2.2b

- Frozen the theory-to-software mapping contract
- Defined center-coordinate safety-distance mapping
- Defined Road-derived target lateral reference W
- Clarified acceleration as control rather than analytical state
- Frozen the future reduced Theory State interface

### P0.2.2

- Added validated Phase 0 fundamental data models
- Added VehicleState, Vehicle, Road and SimulationConfig contracts
- Added MATLAB unit tests for fundamental data models
- Added versioned P0.2.2 data-model contract documentation

### P0.2.1a

- Added versioned Phase 0 software requirements
- Added versioned P0.2 software architecture baseline
- Established repository documentation as the development source of truth

### P0.2.1
- Initialized MATLAB project skeleton
- Added avlc package namespace
- Added runSimulation placeholder
- Added CapabilityEvaluator placeholder
- Added project structure verification
- Added Git repository metadata and MIT License
- Preserved empty project directories with .gitkeep files
