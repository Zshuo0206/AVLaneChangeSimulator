function tests = testVehicle
%TESTVEHICLE Contract tests for Phase 0 vehicle data and nested validation.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidEgoAndExactFields(testCase)
state = validState();
vehicle = avlc.model.createVehicle("ego-01", "EGO", 4.5, 1.8, state);
testCase.verifyEqual(fieldnames(vehicle), ...
    {'id'; 'role'; 'geometry'; 'initialState'; 'dynamics'});
testCase.verifyEqual(fieldnames(vehicle.geometry), {'length'; 'width'});
testCase.verifyEqual(fieldnames(vehicle.dynamics), {'type'; 'parameters'});
testCase.verifyEqual(vehicle.id, "ego-01");
testCase.verifyEqual(vehicle.role, "EGO");
testCase.verifyEqual(vehicle.geometry.length, 4.5, 'AbsTol', 1e-10);
testCase.verifyEqual(vehicle.geometry.width, 1.8, 'AbsTol', 1e-10);
testCase.verifyEqual(vehicle.initialState, state);
avlc.model.validateVehicle(vehicle);
end

function testAllFourRoles(testCase)
for role = ["EGO", "FO", "RO", "RT"]
    vehicle = avlc.model.createVehicle("vehicle-01", role, 4, 2, validState());
    testCase.verifyEqual(vehicle.role, role);
    avlc.model.validateVehicle(vehicle);
end
end

function testCharacterInputsNormalizeToStrings(testCase)
vehicle = avlc.model.createVehicle('vehicle-01', 'FO', 4, 2, validState());
testCase.verifyClass(vehicle.id, 'string');
testCase.verifyClass(vehicle.role, 'string');
testCase.verifySize(vehicle.id, [1, 1]);
testCase.verifyEqual(vehicle.id, "vehicle-01");
testCase.verifyEqual(vehicle.role, "FO");
end

function testColumnCharacterVectorsNormalize(testCase)
vehicle = avlc.model.createVehicle(['i'; 'd'], ['r'; 't'], 4, 2, validState());
testCase.verifyEqual(vehicle.id, "id");
testCase.verifyEqual(vehicle.role, "RT");
end

function testLowercaseAndMixedCaseRolesNormalize(testCase)
for role = ["ego", "fo", "rO", "Rt"]
    vehicle = avlc.model.createVehicle("id", role, 4, 2, validState());
    testCase.verifyEqual(vehicle.role, upper(role));
    raw = vehicle;
    raw.role = role;
    avlc.model.validateVehicle(raw);
    testCase.verifyEqual(raw.role, role);
end
end

function testGeometryNormalizesToDouble(testCase)
vehicle = avlc.model.createVehicle("id", "EGO", int16(4), single(2), validState());
testCase.verifyClass(vehicle.geometry.length, 'double');
testCase.verifyClass(vehicle.geometry.width, 'double');
end

function testDefaultUnassignedDynamics(testCase)
vehicle = validVehicle();
testCase.verifyEqual(vehicle.dynamics.type, "UNASSIGNED");
testCase.verifyTrue(isstruct(vehicle.dynamics.parameters));
testCase.verifyEmpty(fieldnames(vehicle.dynamics.parameters));
end

function testInvalidRolesRejected(testCase)
values = {"FT", "NPC", "LEAD", "FOLLOWER", "", " EGO", ...
    1, true, ["EGO", "FO"], ['EG'; 'FO'], string(missing), {'EGO'}};
for index = 1:numel(values)
    role = values{index};
    testCase.verifyError(@() avlc.model.createVehicle("id", role, 4, 2, validState()), ...
        'AVLC:InvalidVehicle');
    vehicle = validVehicle();
    vehicle.role = role;
    testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
end
end

function testEmptyAndInvalidIdsRejected(testCase)
values = {"", '', [], 1, true, ["a", "b"], ['ab'; 'cd'], string(missing), {'id'}};
for index = 1:numel(values)
    id = values{index};
    testCase.verifyError(@() avlc.model.createVehicle(id, "EGO", 4, 2, validState()), ...
        'AVLC:InvalidVehicle');
    vehicle = validVehicle();
    vehicle.id = id;
    testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
end
end

function testInvalidDimensionsRejected(testCase)
values = {0, -1, NaN, Inf, 1 + 1i, [1, 2], [], '4', true, {4}};
for index = 1:numel(values)
    value = values{index};
    testCase.verifyError(@() avlc.model.createVehicle("id", "EGO", value, 2, validState()), ...
        'AVLC:InvalidVehicle');
    testCase.verifyError(@() avlc.model.createVehicle("id", "EGO", 4, value, validState()), ...
        'AVLC:InvalidVehicle');
    for field = {'length', 'width'}
        vehicle = validVehicle();
        vehicle.geometry.(field{1}) = value;
        testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
    end
end
end

function testInvalidInitialStateHasVehicleError(testCase)
bad = validState();
bad.ax = NaN;
values = {bad, rmfield(validState(), 'x'), [], "invalid"};
for index = 1:numel(values)
    state = values{index};
    testCase.verifyError(@() avlc.model.createVehicle("id", "EGO", 4, 2, state), ...
        'AVLC:InvalidVehicle');
    vehicle = validVehicle();
    vehicle.initialState = state;
    testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
end
end

function testRequiredTopLevelFields(testCase)
vehicle = validVehicle();
fields = fieldnames(vehicle);
for index = 1:numel(fields)
    invalid = rmfield(vehicle, fields{index});
    testCase.verifyError(@() avlc.model.validateVehicle(invalid), 'AVLC:InvalidVehicle');
end
end

function testRequiredNestedFields(testCase)
for container = {'geometry', 'dynamics'}
    vehicle = validVehicle();
    fields = fieldnames(vehicle.(container{1}));
    for index = 1:numel(fields)
        invalid = vehicle;
        invalid.(container{1}) = rmfield(invalid.(container{1}), fields{index});
        testCase.verifyError(@() avlc.model.validateVehicle(invalid), 'AVLC:InvalidVehicle');
    end
end
end

function testScalarStructContainersRequired(testCase)
vehicle = validVehicle();
for value = {[], 1, struct([]), [vehicle, vehicle]}
    testCase.verifyError(@() avlc.model.validateVehicle(value{1}), 'AVLC:InvalidVehicle');
end
for container = {'geometry', 'dynamics'}
    for value = {[], 1, struct([]), repmat(vehicle.(container{1}), 1, 2)}
        invalid = vehicle;
        invalid.(container{1}) = value{1};
        testCase.verifyError(@() avlc.model.validateVehicle(invalid), 'AVLC:InvalidVehicle');
    end
end
end

function testDynamicsTypeMustBeTextScalar(testCase)
for value = {1, true, ["a", "b"], ['ab'; 'cd'], string(missing), {'a'}}
    vehicle = validVehicle();
    vehicle.dynamics.type = value{1};
    testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
end
end

function testDynamicsParametersMustBeStruct(testCase)
for value = {[], 1, "parameters", {struct()}}
    vehicle = validVehicle();
    vehicle.dynamics.parameters = value{1};
    testCase.verifyError(@() avlc.model.validateVehicle(vehicle), 'AVLC:InvalidVehicle');
end
end

function testDynamicsMeaningIsNotInterpreted(testCase)
vehicle = validVehicle();
vehicle.dynamics.type = "future-model";
vehicle.dynamics.parameters = struct('arbitrary', -123);
avlc.model.validateVehicle(vehicle);
vehicle.dynamics.type = "";
vehicle.dynamics.parameters = struct([]);
avlc.model.validateVehicle(vehicle);
end

function testAdditionalFieldsAllowedWithoutMutation(testCase)
vehicle = validVehicle();
vehicle.notes = "metadata";
vehicle.geometry.notes = "metadata";
vehicle.initialState.notes = "metadata";
vehicle.dynamics.notes = "metadata";
before = vehicle;
avlc.model.validateVehicle(vehicle);
testCase.verifyEqual(vehicle, before);
end

function testUniquenessIsNotVehicleResponsibility(testCase)
first = validVehicle();
second = validVehicle();
avlc.model.validateVehicle(first);
avlc.model.validateVehicle(second);
testCase.verifyEqual(first.id, second.id);
testCase.verifyEqual(first.role, second.role);
end

function vehicle = validVehicle()
vehicle = avlc.model.createVehicle("vehicle-01", "EGO", 4.5, 1.8, validState());
end

function state = validState()
state = avlc.model.createVehicleState(0, 0, 10, 0, 0, 0);
end
