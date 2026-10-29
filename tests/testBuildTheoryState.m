function tests = testBuildTheoryState
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testExactEightFieldsAndHandCalculatedNominalCase(testCase)
[s, states] = fixture();
z = avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual(fieldnames(z), {'y'; 'w'; 'gR'; 'cR'; 'gF'; 'cF'; 'gT'; 'cT'});
% Lengths 4/6/5/7 and clearances 2/3/4 give distances 7/7.5/9.5.
% Center separations 30/20/30 give front/rear/target gaps 23/12.5/20.5.
testCase.verifyEqual(cell2mat(struct2cell(z)).', [1.25, -0.75, 12.5, 3, 23, 2, 20.5, 4]);
end

function testEachVehicleLengthContribution(testCase)
[s, states] = fixture();
roles = {'EGO', 'FO', 'RO', 'RT'};
% Increasing one length by 2 decreases just its participating gaps by 1.
expected = [11.5, 22, 19.5; 12.5, 22, 20.5; ...
    11.5, 23, 20.5; 12.5, 23, 19.5];
for index = 1:4
    changed = s;
    changed.vehicles.(roles{index}).geometry.length = ...
        changed.vehicles.(roles{index}).geometry.length + 2;
    z = avlc.analysis.buildTheoryState(changed, states{:});
    testCase.verifyEqual([z.gR, z.gF, z.gT], expected(index, :));
end
end

function testClearancesAppliedToCorrespondingGapExactlyOnce(testCase)
[s, states] = fixture();
names = {'frontSafetyClearance', 'rearOriginalSafetyClearance', 'rearTargetSafetyClearance'};
expected = [12.5, 21, 20.5; 10.5, 23, 20.5; 12.5, 23, 18.5];
for index = 1:3
    changed = s;
    changed.theoryParameters.(names{index}) = changed.theoryParameters.(names{index}) + 2;
    z = avlc.analysis.buildTheoryState(changed, states{:});
    testCase.verifyEqual([z.gR, z.gF, z.gT], expected(index, :));
end
end

function testEachClosedGapBoundary(testCase)
[s, states] = fixture();
% Ego center is 10; independently calculated boundary centers FO/RO/RT.
centers = [17, 2.5, 0.5];
expected = [12.5, 0, 20.5; 0, 23, 20.5; 12.5, 23, 0];
for index = 1:3
    changed = states;
    changed{index+1}.x = centers(index);
    z = avlc.analysis.buildTheoryState(s, changed{:});
    testCase.verifyEqual([z.gR, z.gF, z.gT], expected(index, :));
end
end

function testEachNegativeGapRemainsRepresentable(testCase)
[s, states] = fixture();
centers = [16, 3.5, 1.5];
expected = [12.5, -1, 20.5; -1, 23, 20.5; 12.5, 23, -1];
for index = 1:3
    changed = states;
    changed{index+1}.x = centers(index);
    z = avlc.analysis.buildTheoryState(s, changed{:});
    testCase.verifyEqual([z.gR, z.gF, z.gT], expected(index, :));
end
end

function testTinyNegativeGapsAreNotCorrected(testCase)
[s, states] = fixture();
% Exact binary displacements, small enough to expose a 1e-12 correction.
delta = 2^-40;
states{1}.x = 0;
states{2}.x = 7 - delta;
states{3}.x = -7.5 + delta;
states{4}.x = -9.5 + delta;
z = avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual([z.gR, z.gF, z.gT], [-delta, -delta, -delta]);
end

function testFrontClosingSpeedSigns(testCase)
[s, states] = fixture();
velocities = [8, 10, 12];
expected = [2, 0, -2];
for index = 1:3
    states{2}.vx = velocities(index);
    z = avlc.analysis.buildTheoryState(s, states{:});
    testCase.verifyEqual(z.cF, expected(index));
end
end

function testOriginalRearClosingSpeedSigns(testCase)
[s, states] = fixture();
velocities = [12, 10, 8];
expected = [2, 0, -2];
for index = 1:3
    states{3}.vx = velocities(index);
    z = avlc.analysis.buildTheoryState(s, states{:});
    testCase.verifyEqual(z.cR, expected(index));
end
end

function testTargetRearClosingSpeedSigns(testCase)
[s, states] = fixture();
velocities = [12, 10, 8];
expected = [2, 0, -2];
for index = 1:3
    states{4}.vx = velocities(index);
    z = avlc.analysis.buildTheoryState(s, states{:});
    testCase.verifyEqual(z.cT, expected(index));
end
end

function testEgoLateralPositionAndSignedVelocity(testCase)
[s, states] = fixture();
values = [-20, -7; 0, 0; 25, 9];
for index = 1:3
    states{1}.y = values(index, 1);
    states{1}.vy = values(index, 2);
    states{1}.ay = 12345;
    z = avlc.analysis.buildTheoryState(s, states{:});
    testCase.verifyEqual([z.y, z.w], values(index, :));
end
end

function testAccelerationSnapshotsIgnoredForEveryRole(testCase)
[s, states] = fixture();
expected = avlc.analysis.buildTheoryState(s, states{:});
for index = 1:4
    changed = states;
    changed{index}.ax = -1e100;
    changed{index}.ay = 1e100;
    testCase.verifyEqual(avlc.analysis.buildTheoryState(s, changed{:}), expected);
end
end

function testSurroundingLateralStatesIgnoredForEveryRole(testCase)
[s, states] = fixture();
expected = avlc.analysis.buildTheoryState(s, states{:});
for index = 2:4
    changed = states;
    changed{index}.y = -1e100;
    changed{index}.vy = 1e100;
    testCase.verifyEqual(avlc.analysis.buildTheoryState(s, changed{:}), expected);
end
end

function testCurrentStatesOverrideScenarioInitialStates(testCase)
[s, states] = fixture();
for role = {'EGO', 'FO', 'RO', 'RT'}
    s.vehicles.(role{1}).initialState = ...
        avlc.model.createVehicleState(1000, 1000, -1000, 1000, 1000, -1000);
end
z = avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual(cell2mat(struct2cell(z)).', [1.25, -0.75, 12.5, 3, 23, 2, 20.5, 4]);
end

function testUnusualOrderingNegativeVelocityAndOffLaneStates(testCase)
[s, states] = fixture();
states{1} = avlc.model.createVehicleState(0, -50, -10, -2, 0, 0);
states{2} = avlc.model.createVehicleState(-30, 80, -12, -100, 0, 0);
states{3} = avlc.model.createVehicleState(20, -80, -8, 100, 0, 0);
states{4} = avlc.model.createVehicleState(30, 200, -13, -200, 0, 0);
z = avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual(cell2mat(struct2cell(z)).', [-50, -2, -27.5, 2, -37, 2, -39.5, -3]);
end

function testArgumentOrderDefinesRolesWithoutCoordinateInference(testCase)
[s, states] = fixture();
z = avlc.analysis.buildTheoryState(s, states{1}, states{3}, states{2}, states{4});
testCase.verifyEqual(cell2mat(struct2cell(z)).', [1.25, -0.75, -37.5, -2, -27, -3, 20.5, 4]);
end

function testInvalidScenarioUsesExistingValidator(testCase)
[s, states] = fixture();
testCase.verifyError(@() avlc.analysis.buildTheoryState(rmfield(s, 'road'), states{:}), ...
    'AVLC:InvalidScenario');
s.vehicles.FO.geometry.length = -1;
testCase.verifyError(@() avlc.analysis.buildTheoryState(s, states{:}), 'AVLC:InvalidScenario');
end

function testInvalidEgoStateUsesExistingValidator(testCase)
[s, states] = fixture();
states{1}.x = NaN;
testCase.verifyError(@() avlc.analysis.buildTheoryState(s, states{:}), 'AVLC:InvalidVehicleState');
end

function testInvalidFoStateUsesExistingValidator(testCase)
[s, states] = fixture();
states{2} = rmfield(states{2}, 'vx');
testCase.verifyError(@() avlc.analysis.buildTheoryState(s, states{:}), 'AVLC:InvalidVehicleState');
end

function testInvalidRoStateUsesExistingValidator(testCase)
[s, states] = fixture();
states{3}.x = [1, 2];
testCase.verifyError(@() avlc.analysis.buildTheoryState(s, states{:}), 'AVLC:InvalidVehicleState');
end

function testInvalidRtStateUsesExistingValidator(testCase)
[s, states] = fixture();
states{4}.vx = 1i;
testCase.verifyError(@() avlc.analysis.buildTheoryState(s, states{:}), 'AVLC:InvalidVehicleState');
end

function testIgnoredFieldsStillRequireStructuralValidity(testCase)
[s, states] = fixture();
for index = 1:4
    for name = {'ax', 'ay'}
        changed = states;
        changed{index}.(name{1}) = Inf;
        testCase.verifyError(@() avlc.analysis.buildTheoryState(s, changed{:}), ...
            'AVLC:InvalidVehicleState');
    end
end
for index = 2:4
    for name = {'y', 'vy'}
        changed = states;
        changed{index}.(name{1}) = NaN;
        testCase.verifyError(@() avlc.analysis.buildTheoryState(s, changed{:}), ...
            'AVLC:InvalidVehicleState');
    end
end
end

function testRawNumericInputsUseDoubleArithmetic(testCase)
[s, states] = fixture();
s.vehicles.EGO.geometry.length = uint8(4);
s.vehicles.FO.geometry.length = uint8(6);
s.vehicles.RO.geometry.length = uint8(5);
s.vehicles.RT.geometry.length = uint8(7);
s.theoryParameters.frontSafetyClearance = uint8(2);
s.theoryParameters.rearOriginalSafetyClearance = uint8(3);
s.theoryParameters.rearTargetSafetyClearance = uint8(4);
for index = 1:4
    states{index}.x = uint8(0);
    states{index}.vx = uint8(0);
end
states{1}.vx = uint8(10);
states{1}.y = int16(-2);
states{1}.vy = single(-0.5);
z = avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual(cell2mat(struct2cell(z)).', [-2, -0.5, -7.5, -10, -7, 10, -9.5, -10]);
for name = fieldnames(z).'
    testCase.verifyClass(z.(name{1}), 'double');
end
end

function testInputsRemainUnchangedAndNoDerivedFieldsPersist(testCase)
[s, states] = fixture();
s.notes = "compatible Scenario metadata";
states{1}.notes = "compatible state metadata";
beforeScenario = s;
beforeStates = states;
avlc.analysis.buildTheoryState(s, states{:});
testCase.verifyEqual(s, beforeScenario);
testCase.verifyEqual(states, beforeStates);
testCase.verifyFalse(isfield(s, 'W'));
testCase.verifyFalse(any(isfield(s.theoryParameters, {'W', 'dF', 'dR', 'dT'})));
end

function testNoninitialSimulationSampleIntegration(testCase)
[s, ~] = fixture();
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.1, 0.2), ...
    avlc.model.createControl(1, 0.5));
z = avlc.analysis.buildTheoryState(s, result.states.EGO(3), ...
    result.states.FO(3), result.states.RO(3), result.states.RT(3));
% At t=0.2 Ego is at (0.02,0.01), velocity (0.2,0.1); others stay at 0.
testCase.verifyEqual(cell2mat(struct2cell(z)).', [0.01, 0.1, -7.48, -0.2, -7.02, 0.2, -9.48, -0.2], ...
    'AbsTol', 1e-14);
end

function [scenario, states] = fixture()
road = avlc.model.createRoad(3.6, [-100, 100]);
params = avlc.model.createTheoryModelParameters(2, 4, 3, 0, 2, 3, 4);
roles = {'EGO', 'FO', 'RO', 'RT'};
lengths = [4, 6, 5, 7];
initial = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
vehicles = cell(1, 4);
for index = 1:4
    vehicles{index} = avlc.model.createVehicle(string(index), roles{index}, ...
        lengths(index), 2, initial);
end
scenario = avlc.scenario.createScenario("theory state fixture", "", road, params, vehicles{:});
states = {avlc.model.createVehicleState(10, 1.25, 10, -0.75, 0, 0), ...
    avlc.model.createVehicleState(40, 0, 8, 0, 0, 0), ...
    avlc.model.createVehicleState(-10, 0, 13, 0, 0, 0), ...
    avlc.model.createVehicleState(-20, 3.6, 14, 0, 0, 0)};
end
