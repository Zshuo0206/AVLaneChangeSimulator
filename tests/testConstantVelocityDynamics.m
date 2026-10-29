function tests = testConstantVelocityDynamics
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testPositiveVelocityAnalyticalStep(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 0, 0);
actual = avlc.dynamics.stepConstantVelocity(state, 0.5);
verifyState(testCase, actual, [2.5, 4, 3, 4, 0, 0]);
end

function testNegativeVelocityAnalyticalStep(testCase)
state = avlc.model.createVehicleState(-1, -2, -3, -4, 0, 0);
actual = avlc.dynamics.stepConstantVelocity(state, 2);
verifyState(testCase, actual, [-7, -10, -3, -4, 0, 0]);
end

function testStationaryVehicle(testCase)
state = avlc.model.createVehicleState(7, -3, 0, 0, 12, -15);
actual = avlc.dynamics.stepConstantVelocity(state, 10);
verifyState(testCase, actual, [7, -3, 0, 0, 0, 0]);
end

function testPreviousAccelerationSnapshotsIgnored(testCase)
first = avlc.model.createVehicleState(1, 2, 3, 4, 100, -200);
second = avlc.model.createVehicleState(1, 2, 3, 4, -500, 1000);
actual = avlc.dynamics.stepConstantVelocity(first, 0.5);
testCase.verifyEqual(actual, avlc.dynamics.stepConstantVelocity(second, 0.5));
verifyState(testCase, actual, [2.5, 4, 3, 4, 0, 0]);
end

function testIntegerTimeStepAndStateAvoidSaturation(testCase)
state = avlc.model.createVehicleState(120, -120, 20, -20, 0, 0);
for field = {'x', 'y', 'vx', 'vy'}
    state.(field{1}) = int8(state.(field{1}));
end
actual = avlc.dynamics.stepConstantVelocity(state, uint8(2));
verifyState(testCase, actual, [160, -160, 20, -20, 0, 0]);
testCase.verifyTrue(all(structfun(@(v) isa(v, 'double'), actual)));
end

function testInvalidTimeStepsRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 1, 1, 0, 0);
for value = {0, -1, NaN, Inf, -Inf, [1, 2], [], 1+1i, '1', true, {1}}
    testCase.verifyError(@() avlc.dynamics.stepConstantVelocity(state, value{1}), ...
        'AVLC:InvalidTimeStep');
end
end

function testInvalidVehicleStatesRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 1, 1, 0, 0);
bad = state;
bad.ay = NaN;
for value = {bad, rmfield(state, 'x'), [], [state, state]}
    testCase.verifyError(@() avlc.dynamics.stepConstantVelocity(value{1}, 1), ...
        'AVLC:InvalidVehicleState');
end
end

function testInputUnchangedAndOutputSchema(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 100, 200);
state.notes = "metadata";
before = state;
actual = avlc.dynamics.stepConstantVelocity(state, 0.5);
testCase.verifyEqual(state, before);
testCase.verifyEqual(fieldnames(actual), {'x'; 'y'; 'vx'; 'vy'; 'ax'; 'ay'});
avlc.model.validateVehicleState(actual);
end

function testNonfiniteOutputIsRejected(testCase)
state = avlc.model.createVehicleState(realmax, 0, realmax, 0, 0, 0);
testCase.verifyError(@() avlc.dynamics.stepConstantVelocity(state, 2), ...
    'AVLC:InvalidVehicleState');
end

function verifyState(testCase, actual, expected)
testCase.verifyEqual([actual.x, actual.y, actual.vx, actual.vy, actual.ax, actual.ay], ...
    expected, 'AbsTol', 1e-10);
avlc.model.validateVehicleState(actual);
end
