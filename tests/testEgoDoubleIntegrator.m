function tests = testEgoDoubleIntegrator
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testZeroControlMatchesConstantVelocity(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 100, -100);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, 0), params(), 0.5);
verifyState(testCase, actual, [2.5, 4, 3, 4, 0, 0]);
testCase.verifyEqual(actual, avlc.dynamics.stepConstantVelocity(state, 0.5));
end

function testPositiveLongitudinalAccelerationExactPosition(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 0, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(3, 0), params(), 2);
verifyState(testCase, actual, [13, 2, 9, 0, 3, 0]);
end

function testMaximumBrakingExactStep(testCase)
state = avlc.model.createVehicleState(10, 2, 8, 0, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(-4, 0), params(), 2);
verifyState(testCase, actual, [18, 2, 0, 0, -4, 0]);
end

function testMaximumPositiveLateralAcceleration(testCase)
state = avlc.model.createVehicleState(0, 1, 0, 0, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, 2), params(), 2);
verifyState(testCase, actual, [0, 5, 0, 4, 0, 2]);
end

function testLateralBrakingExactStep(testCase)
state = avlc.model.createVehicleState(0, 1, 0, 4, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, -2), params(), 1);
verifyState(testCase, actual, [0, 4, 0, 2, 0, -2]);
end

function testNegativeLateralAndLongitudinalVelocityAllowed(testCase)
state = avlc.model.createVehicleState(-2, 1, -3, -2, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, -2), params(), 0.5);
verifyState(testCase, actual, [-3.5, -0.25, -3, -3, 0, -2]);
end

function testLateralVelocityCrossesZero(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 1, 0, 0);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, -2), params(), 1);
verifyState(testCase, actual, [0, 0, 0, -1, 0, -2]);
end

function testAllControlBoundaryCombinationsAccepted(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
for ax = [-4, 3]
    for ay = [-2, 2]
        actual = avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(ax, ay), params(), 1);
        avlc.model.validateVehicleState(actual);
        testCase.verifyEqual([actual.ax, actual.ay], [ax, ay], 'AbsTol', 1e-10);
    end
end
end

function testControlsJustOutsideEveryBoundRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
values = [-4-eps(4), 0; 3+eps(3), 0; 0, -2-eps(2); 0, 2+eps(2)];
for index = 1:size(values, 1)
    control = avlc.model.createControl(values(index, 1), values(index, 2));
    before = control;
    testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, control, params(), 1), ...
        'AVLC:ControlOutOfBounds');
    testCase.verifyEqual(control, before);
end
end

function testContradictorySnapshotsDoNotDriveTrajectory(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 100, -500);
control = avlc.model.createControl(1, -1);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, control, params(), 2);
verifyState(testCase, actual, [9, 8, 5, 2, 1, -1]);
state.ax = -200;
state.ay = 1000;
testCase.verifyEqual(actual, avlc.dynamics.stepEgoDoubleIntegrator(state, control, params(), 2));
end

function testIntegerInputsAndUnsignedBoundsAvoidSaturation(testCase)
state = avlc.model.createVehicleState(120, 2, 20, 3, 100, -100);
for field = {'x', 'y', 'vx', 'vy'}
    state.(field{1}) = int8(state.(field{1}));
end
control = struct('ax', int8(-4), 'ay', int8(-2));
p = params();
p.maxLateralAccel = uint8(2);
p.maxLongitudinalBrake = uint8(4);
p.maxLongitudinalAccel = uint8(3);
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, control, p, uint8(2));
verifyState(testCase, actual, [152, 4, 12, -1, -4, -2]);
testCase.verifyTrue(all(structfun(@(v) isa(v, 'double'), actual)));
end

function testInvalidStateRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
state.ax = NaN;
testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, 0), params(), 1), ...
    'AVLC:InvalidVehicleState');
end

function testInvalidControlRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
for control = {struct('ax', 0), struct('ax', NaN, 'ay', 0), []}
    testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, control{1}, params(), 1), ...
        'AVLC:InvalidControl');
end
end

function testInvalidParametersRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
p = params();
p.maxLateralAccel = 0;
testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, 0), p, 1), ...
    'AVLC:InvalidTheoryModelParameters');
end

function testInvalidTimeStepsRejected(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
control = avlc.model.createControl(0, 0);
for value = {0, -1, NaN, Inf, -Inf, [1, 2], [], 1+1i, '1', true, {1}}
    testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, control, params(), value{1}), ...
        'AVLC:InvalidTimeStep');
end
end

function testInputsUnchangedAndExactOutputSchema(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 100, -100);
state.notes = "state metadata";
control = avlc.model.createControl(1, -1);
control.notes = "control metadata";
p = params();
p.notes = "parameter metadata";
before = {state, control, p};
actual = avlc.dynamics.stepEgoDoubleIntegrator(state, control, p, 2);
testCase.verifyEqual({state, control, p}, before);
testCase.verifyEqual(fieldnames(actual), {'x'; 'y'; 'vx'; 'vy'; 'ax'; 'ay'});
avlc.model.validateVehicleState(actual);
end

function testNonfiniteOutputIsRejected(testCase)
state = avlc.model.createVehicleState(realmax, 0, realmax, 0, 0, 0);
testCase.verifyError(@() avlc.dynamics.stepEgoDoubleIntegrator(state, avlc.model.createControl(0, 0), params(), 2), ...
    'AVLC:InvalidVehicleState');
end

function p = params()
p = avlc.model.createTheoryModelParameters(2, 4, 3, 0, 0, 0, 0);
end

function verifyState(testCase, actual, expected)
testCase.verifyEqual([actual.x, actual.y, actual.vx, actual.vy, actual.ax, actual.ay], ...
    expected, 'AbsTol', 1e-10);
avlc.model.validateVehicleState(actual);
end
