function tests = testVehicleState
%TESTVEHICLESTATE Contract tests for Phase 0 vehicle-state data.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidStateAndExactFields(testCase)
state = avlc.model.createVehicleState(1, 2, 3, 4, 5, 6);
testCase.verifyEqual(fieldnames(state), {'x'; 'y'; 'vx'; 'vy'; 'ax'; 'ay'});
testCase.verifyEqual(cell2mat(struct2cell(state)).', 1:6, 'AbsTol', 1e-10);
avlc.model.validateVehicleState(state);
end

function testNumericClassesNormalizeToDouble(testCase)
state = avlc.model.createVehicleState(int8(-1), uint16(2), single(3), ...
    int32(-4), int64(5), uint64(6));
fields = fieldnames(state);
for index = 1:numel(fields)
    testCase.verifyClass(state.(fields{index}), 'double');
end
testCase.verifyEqual(cell2mat(struct2cell(state)).', [-1, 2, 3, -4, 5, 6], 'AbsTol', 1e-10);
avlc.model.validateVehicleState(state);
end

function testNegativeValuesRemainValid(testCase)
state = avlc.model.createVehicleState(-1, -2, -3, -4, -5, -6);
testCase.verifyEqual(cell2mat(struct2cell(state)).', -(1:6), 'AbsTol', 1e-10);
avlc.model.validateVehicleState(state);
end

function testEveryRequiredFieldIsRequired(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
fields = fieldnames(state);
for index = 1:numel(fields)
    invalid = rmfield(state, fields{index});
    testCase.verifyError(@() avlc.model.validateVehicleState(invalid), ...
        'AVLC:InvalidVehicleState');
end
end

function testScalarStructRequired(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
invalid = {[], struct([]), [state, state], 42, {state}};
for index = 1:numel(invalid)
    testCase.verifyError(@() avlc.model.validateVehicleState(invalid{index}), ...
        'AVLC:InvalidVehicleState');
end
end

function testNaNRejected(testCase)
verifyInvalidValue(testCase, NaN);
end

function testInfinityRejected(testCase)
verifyInvalidValue(testCase, Inf);
verifyInvalidValue(testCase, -Inf);
end

function testComplexRejected(testCase)
verifyInvalidValue(testCase, 1 + 2i);
end

function testNonscalarRejected(testCase)
verifyInvalidValue(testCase, [1, 2]);
verifyInvalidValue(testCase, []);
end

function testNonnumericRejected(testCase)
verifyInvalidValue(testCase, '1');
verifyInvalidValue(testCase, "1");
verifyInvalidValue(testCase, {1});
end

function testLogicalRejected(testCase)
verifyInvalidValue(testCase, true);
end

function testAdditionalFieldsAllowedWithoutMutation(testCase)
state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
state.annotation = "future-compatible metadata";
before = state;
avlc.model.validateVehicleState(state);
testCase.verifyEqual(state, before);
end

function verifyInvalidValue(testCase, value)
fields = {'x', 'y', 'vx', 'vy', 'ax', 'ay'};
for index = 1:numel(fields)
    inputs = num2cell(zeros(1, 6));
    inputs{index} = value;
    testCase.verifyError(@() avlc.model.createVehicleState(inputs{:}), ...
        'AVLC:InvalidVehicleState');
    state = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
    state.(fields{index}) = value;
    testCase.verifyError(@() avlc.model.validateVehicleState(state), ...
        'AVLC:InvalidVehicleState');
end
end
