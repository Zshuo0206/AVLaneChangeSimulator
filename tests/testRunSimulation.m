function tests = testRunSimulation
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testExactResultStructureAndOrientations(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(fieldnames(r), {'time'; 'states'; 'controls'; 'metadata'});
testCase.verifyEqual(fieldnames(r.states), {'EGO'; 'FO'; 'RO'; 'RT'});
testCase.verifyEqual(fieldnames(r.controls), {'EGO'});
testCase.verifySize(r.time, [11, 1]);
for role = {'EGO', 'FO', 'RO', 'RT'}
    testCase.verifyTrue(isstruct(r.states.(role{1})));
    testCase.verifySize(r.states.(role{1}), [11, 1]);
end
testCase.verifyTrue(isstruct(r.controls.EGO));
testCase.verifySize(r.controls.EGO, [10, 1]);
end

function testInitialStatesAndSnapshotsPreserved(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(r.time(1), 0, 'AbsTol', 1e-10);
for role = {'EGO', 'FO', 'RO', 'RT'}
    testCase.verifyEqual(r.states.(role{1})(1), s.vehicles.(role{1}).initialState);
end
end

function testRawInitialNumericTypesAndMetadataPreserved(testCase)
[s, config, control] = fixture();
for role = {'EGO', 'FO', 'RO', 'RT'}
    s.vehicles.(role{1}).initialState.x = int16(2);
    s.vehicles.(role{1}).initialState.notes = "initial metadata";
end
control.notes = "control metadata";
r = avlc.core.runSimulation(s, config, control);
for role = {'EGO', 'FO', 'RO', 'RT'}
    history = r.states.(role{1});
    testCase.verifyEqual(history(1), s.vehicles.(role{1}).initialState);
    testCase.verifyClass(history(1).x, 'int16');
    testCase.verifyClass(history(2).x, 'double');
    testCase.verifyEqual(history(end).notes, "initial metadata");
end
testCase.verifyEqual(r.controls.EGO, repmat(control, 10, 1));
end

function testDecimalGridsAndFixedIntervals(testCase)
[s, ~, control] = fixture();
values = [0.1, 1, 10; 0.1, 0.3, 3; 0.07, 0.49, 7; 0.03, 0.3, 10];
for index = 1:size(values, 1)
    config = avlc.model.createSimulationConfig(values(index, 1), values(index, 2));
    r = avlc.core.runSimulation(s, config, control);
    testCase.verifySize(r.time, [values(index, 3)+1, 1]);
    testCase.verifyEqual(r.time, (0:values(index, 3)).' * config.dt, 'AbsTol', 1e-12);
    testCase.verifyEqual(diff(r.time), repmat(config.dt, values(index, 3), 1), 'AbsTol', 1e-12);
    testCase.verifyEqual(r.time(end), config.endTime, 'AbsTol', 1e-12);
end
end

function testSingleInterval(testCase)
[s, ~, control] = fixture();
r = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(1, 1), control);
testCase.verifySize(r.time, [2, 1]);
testCase.verifySize(r.controls.EGO, [1, 1]);
testCase.verifyEqual(r.metadata.numSteps, 1, 'AbsTol', 1e-10);
end

function testNearIntegralGridDoesNotForceFinalSample(testCase)
[s, ~, control] = fixture();
config = avlc.model.createSimulationConfig(0.1, 1 + 5e-14);
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(r.time, (0:10).' * 0.1);
testCase.verifyGreaterThan(config.endTime - r.time(end), 0);
testCase.verifyLessThan(abs(config.endTime-r.time(end)), 1e-13);
end

function testNonIntegralAndZeroStepGridsRejected(testCase)
[s, ~, control] = fixture();
for values = {[0.3, 1], [2, 1], [1, 0.1], [0.1, 1+1e-6]}
    config = avlc.model.createSimulationConfig(values{1}(1), values{1}(2));
    avlc.model.validateSimulationConfig(config); % Valid config, invalid engine grid.
    testCase.verifyError(@() avlc.core.runSimulation(s, config, control), 'AVLC:InvalidSimulationGrid');
end
end

function testUnrepresentableGridRejectedBeforeAllocation(testCase)
[s, ~, control] = fixture();
for values = {[realmin, realmax], [1, flintmax]}
    config = avlc.model.createSimulationConfig(values{1}(1), values{1}(2));
    testCase.verifyError(@() avlc.core.runSimulation(s, config, control), 'AVLC:InvalidSimulationGrid');
end
end

function testIntegerConfigFields(testCase)
[s, ~, control] = fixture();
config = avlc.model.createSimulationConfig(1, 2);
config.dt = uint8(1);
config.endTime = uint8(2);
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(r.time, [0;1;2], 'AbsTol', 1e-10);
testCase.verifyEqual(r.metadata.dt, config.dt);
testCase.verifyEqual(r.metadata.endTime, config.endTime);
end

function testEgoGlobalAnalyticalTrajectory(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(stateValues(r.states.EGO(end)), [11.2, 1.65, 8.5, 1.3, 1, 0.5], 'AbsTol', 1e-10);
t = r.time;
testCase.verifyEqual([r.states.EGO.x].', 3.2 + 7.5*t + 0.5*t.^2, 'AbsTol', 1e-10);
testCase.verifyEqual([r.states.EGO.y].', 0.6 + 0.8*t + 0.25*t.^2, 'AbsTol', 1e-10);
end

function testSurroundingGlobalAnalyticalTrajectories(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
expected = [23,-0.1,3,-0.2,0,0; -12,-0.2,-2,0.3,0,0; -19,4.1,1,-0.4,0,0];
roles = {'FO', 'RO', 'RT'};
for index = 1:3
    history = r.states.(roles{index});
    initial = s.vehicles.(roles{index}).initialState;
    testCase.verifyEqual(stateValues(history(end)), expected(index,:), 'AbsTol', 1e-10);
    testCase.verifyEqual([history.x].', initial.x + initial.vx*r.time, 'AbsTol', 1e-10);
    testCase.verifyEqual([history.y].', initial.y + initial.vy*r.time, 'AbsTol', 1e-10);
    testCase.verifyEqual([history(2:end).ax], zeros(1,10), 'AbsTol', 1e-10);
    testCase.verifyEqual([history(2:end).ay], zeros(1,10), 'AbsTol', 1e-10);
end
end

function testEveryStateAndControlValidAndSynchronized(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
for k = 1:11
    for role = {'EGO', 'FO', 'RO', 'RT'}
        avlc.model.validateVehicleState(r.states.(role{1})(k));
        if k <= 10
            if strcmp(role{1}, 'EGO')
                expected = avlc.dynamics.stepEgoDoubleIntegrator(r.states.EGO(k), control, s.theoryParameters, config.dt);
            else
                expected = avlc.dynamics.stepConstantVelocity(r.states.(role{1})(k), config.dt);
            end
            testCase.verifyEqual(r.states.(role{1})(k+1), expected);
        end
    end
end
for k = 1:10
    avlc.model.validateControl(r.controls.EGO(k));
    testCase.verifyEqual(r.controls.EGO(k), control);
end
end

function testContradictoryInitialSnapshotsDoNotDriveTrajectory(testCase)
[s, config, control] = fixture();
first = avlc.core.runSimulation(s, config, control);
s.vehicles.EGO.initialState.ax = -999;
s.vehicles.EGO.initialState.ay = 777;
second = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(first.states.EGO(2:end), second.states.EGO(2:end));
testCase.verifyEqual(stateValues(second.states.EGO(end)), [11.2,1.65,8.5,1.3,1,0.5], 'AbsTol', 1e-10);
testCase.verifyEqual(second.states.EGO(1).ax, -999, 'AbsTol', 1e-10);
end

function testNegativeVelocitiesAndUnsafeScenarioSimulate(testCase)
[s, config, ~] = fixture();
for role = {'EGO', 'FO', 'RO', 'RT'}
    s.vehicles.(role{1}).initialState = avlc.model.createVehicleState(-1,1,-2,-3,100,-100);
end
% Coincident vehicle centers with positive lengths/clearances are deliberately unsafe.
avlc.scenario.validateScenario(s);
r = avlc.core.runSimulation(s, config, avlc.model.createControl(0,0));
for role = {'EGO', 'FO', 'RO', 'RT'}
    testCase.verifyEqual(stateValues(r.states.(role{1})(end)), [-3,-2,-2,-3,0,0], 'AbsTol', 1e-10);
end
end

function testLateralZeroCrossingContinues(testCase)
[s, config, ~] = fixture();
s.vehicles.EGO.initialState = avlc.model.createVehicleState(0,0,0,0.25,0,0);
r = avlc.core.runSimulation(s, config, avlc.model.createControl(0,-2));
testCase.verifyGreaterThan(r.states.EGO(1).vy, 0);
testCase.verifyLessThan(r.states.EGO(3).vy, 0);
testCase.verifyEqual(stateValues(r.states.EGO(end)), [0,-0.75,0,-1.75,0,-2], 'AbsTol', 1e-10);
end

function testBoundErrorsPropagateFromDynamics(testCase)
[s, config, ~] = fixture();
for values = {[-5,0], [4,0], [0,-3], [0,3]}
    control = avlc.model.createControl(values{1}(1), values{1}(2));
    testCase.verifyError(@() avlc.core.runSimulation(s, config, control), 'AVLC:ControlOutOfBounds');
end
end

function testDeterministicResults(testCase)
[s, config, control] = fixture();
first = avlc.core.runSimulation(s, config, control);
second = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(first, second);
end

function testInputImmutabilityAndUnassignedDynamics(testCase)
[s, config, control] = fixture();
before = {s, config, control};
avlc.core.runSimulation(s, config, control);
testCase.verifyEqual({s, config, control}, before);
for role = {'EGO', 'FO', 'RO', 'RT'}
    testCase.verifyEqual(s.vehicles.(role{1}).dynamics.type, "UNASSIGNED");
end
end

function testExactMetadataContract(testCase)
[s, config, control] = fixture();
r = avlc.core.runSimulation(s, config, control);
testCase.verifyEqual(fieldnames(r.metadata), {'scenarioName'; 'dt'; 'endTime'; 'numSteps'; 'controlMode'});
testCase.verifyEqual(r.metadata.scenarioName, s.name);
testCase.verifyEqual(r.metadata.dt, config.dt);
testCase.verifyEqual(r.metadata.endTime, config.endTime);
testCase.verifyEqual(r.metadata.numSteps, 10, 'AbsTol', 1e-10);
testCase.verifyEqual(r.metadata.controlMode, "CONSTANT_EGO_CONTROL");
end

function testUnsupportedFeaturesRejected(testCase)
[s, config, control] = fixture();
for values = {[true,true], [false,false], [false,true]}
    config.storeEveryStep = values{1}(1);
    config.enableCapabilityEvaluation = values{1}(2);
    avlc.model.validateSimulationConfig(config);
    testCase.verifyError(@() avlc.core.runSimulation(s, config, control), 'AVLC:UnsupportedSimulationFeature');
end
end

function testInputValidatorErrorsPropagate(testCase)
[s, config, control] = fixture();
testCase.verifyError(@() avlc.core.runSimulation(rmfield(s,'road'),config,control), 'AVLC:InvalidScenario');
bad = config;
bad.dt = 0;
testCase.verifyError(@() avlc.core.runSimulation(s,bad,control), 'AVLC:InvalidSimulationConfig');
testCase.verifyError(@() avlc.core.runSimulation(s,config,rmfield(control,'ay')), 'AVLC:InvalidControl');
end

function testUnsupportedControllerKindsRejected(testCase)
[s, config, control] = fixture();
for value = {[control;control], @() control, {control}}
    testCase.verifyError(@() avlc.core.runSimulation(s,config,value{1}), 'AVLC:InvalidControl');
end
end

function [s, config, control] = fixture()
road = avlc.model.createRoad(3.6, [-100,100]);
p = avlc.model.createTheoryModelParameters(2,4,3,0,5,5,5);
roles = {'EGO','FO','RO','RT'};
values = [3.2,0.6,7.5,0.8,100,-100; 20,0.1,3,-0.2,80,-90; ...
    -10,-0.5,-2,0.3,60,70; -20,4.5,1,-0.4,-30,40];
vehicles = cell(1,4);
for index = 1:4
    inputs = num2cell(values(index,:));
    state = avlc.model.createVehicleState(inputs{:});
    vehicles{index} = avlc.model.createVehicle("engine-"+index,roles{index},4.5,1.8,state);
end
s = avlc.scenario.createScenario("engine fixture","",road,p,vehicles{:});
config = avlc.model.createSimulationConfig(0.1,1);
control = avlc.model.createControl(1,0.5);
end

function values = stateValues(state)
values = [state.x,state.y,state.vx,state.vy,state.ax,state.ay];
end
