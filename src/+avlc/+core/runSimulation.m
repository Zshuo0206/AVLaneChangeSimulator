function result = runSimulation(scenario, simulationConfig, egoControl)
%RUNSIMULATION Run a deterministic fixed-step four-vehicle simulation.
%   RESULT = avlc.core.runSimulation(SCENARIO, SIMULATIONCONFIG, EGOCONTROL)
%   reuses the existing input validators and P0.2.4 dynamics primitives.
%   One constant Ego control acts on N intervals; time and each column
%   VehicleState history have N+1 samples, with supplied initial states
%   preserved at t=0. RESULT contains time, states, controls, metadata only.
%   dt and endTime are seconds; state/control units remain SI.
%   Requires endTime/dt close to integer N>=1, with ratio tolerance
%   max(1e-12,32*eps(max(1,abs(ratio)))). Invalid grids raise
%   AVLC:InvalidSimulationGrid. Unsupported capability evaluation or sparse
%   storage raises AVLC:UnsupportedSimulationFeature. Validator/dynamics
%   errors propagate unchanged. Inputs are never modified. No safety,
%   capability, theory-state, event, GUI or controller framework is included.

avlc.scenario.validateScenario(scenario);
avlc.model.validateSimulationConfig(simulationConfig);
avlc.model.validateControl(egoControl);
if simulationConfig.enableCapabilityEvaluation || ~simulationConfig.storeEveryStep
    error('AVLC:UnsupportedSimulationFeature', ...
        'P0.2.5 requires capability evaluation disabled and storeEveryStep=true.');
end
dt = double(simulationConfig.dt);
endTime = double(simulationConfig.endTime);
ratio = endTime / dt;
numSteps = round(ratio);
gridTolerance = max(1e-12, 32 * eps(max(1, abs(ratio))));
if ~isfinite(ratio) || numSteps < 1 || numSteps >= flintmax || ...
        abs(ratio - numSteps) > gridTolerance
    error('AVLC:InvalidSimulationGrid', ...
        'endTime/dt must be a representable integer N>=1 within grid tolerance.');
end
time = (0:numSteps).' * dt;
if ~all(isfinite(time)) || abs(time(end) - endTime) > gridTolerance * dt
    error('AVLC:InvalidSimulationGrid', 'Generated final time does not agree with endTime within grid tolerance.');
end
result = struct();
result.time = time;
result.states = struct();
roles = {'EGO', 'FO', 'RO', 'RT'};
for index = 1:numel(roles)
    role = roles{index};
    result.states.(role) = repmat(scenario.vehicles.(role).initialState, numSteps + 1, 1);
end
result.controls = struct();
result.controls.EGO = repmat(egoControl, numSteps, 1);
result.metadata = struct('scenarioName', scenario.name, ...
    'dt', simulationConfig.dt, 'endTime', simulationConfig.endTime, ...
    'numSteps', numSteps, 'controlMode', "CONSTANT_EGO_CONTROL");
for k = 1:numSteps
    for index = 1:numel(roles)
        role = roles{index};
        currentState = result.states.(role)(k);
        if index == 1
            nextState = avlc.dynamics.stepEgoDoubleIntegrator( ...
                currentState, egoControl, scenario.theoryParameters, dt);
        else
            nextState = avlc.dynamics.stepConstantVelocity(currentState, dt);
        end
        % Update only dynamics-owned fields; initial compatible metadata is
        % carried unchanged, allowing a homogeneous struct history schema.
        fields = fieldnames(nextState);
        for fieldIndex = 1:numel(fields)
            result.states.(role)(k + 1).(fields{fieldIndex}) = nextState.(fields{fieldIndex});
        end
    end
end
end
