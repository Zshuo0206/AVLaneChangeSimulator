function tests = testScenario
%TESTSCENARIO Structural experiment validation, without traffic judgments.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidCreationAndExactSchema(testCase)
scenario = validScenario();
testCase.verifyEqual(fieldnames(scenario), ...
    {'name'; 'description'; 'road'; 'theoryParameters'; 'vehicles'; 'events'});
testCase.verifyEqual(fieldnames(scenario.vehicles), {'EGO'; 'FO'; 'RO'; 'RT'});
testCase.verifyEqual(scenario.name, "experiment");
testCase.verifyEqual(scenario.description, "structural fixture");
testCase.verifyTrue(isstruct(scenario.events));
testCase.verifySize(scenario.events, [0, 0]);
avlc.scenario.validateScenario(scenario);
end

function testCharacterTextNormalized(testCase)
scenario = validScenario();
scenario.name = 'character name';
scenario.description = 'character description';
created = recreate(scenario);
testCase.verifyClass(created.name, 'string');
testCase.verifyClass(created.description, 'string');
testCase.verifyEqual(created.name, "character name");
testCase.verifyEqual(created.description, "character description");
end

function testColumnCharacterVectorsNormalized(testCase)
scenario = validScenario();
scenario.name = ['i'; 'd'];
scenario.description = ['o'; 'k'];
created = recreate(scenario);
testCase.verifyEqual(created.name, "id");
testCase.verifyEqual(created.description, "ok");
end

function testEmptyDescriptionAccepted(testCase)
for value = {"", ''}
    scenario = validScenario();
    scenario.description = value{1};
    created = recreate(scenario);
    testCase.verifyEqual(created.description, "");
    avlc.scenario.validateScenario(created);
end
end

function testNestedInputsPreservedWithoutMutation(testCase)
scenario = validScenario();
scenario.road.notes = "retained";
scenario.theoryParameters.notes = "retained";
roles = {'EGO', 'FO', 'RO', 'RT'};
for index = 1:4
    % Valid, noncanonical nested representations must not be normalized here.
    scenario.vehicles.(roles{index}).id = char(scenario.vehicles.(roles{index}).id);
    scenario.vehicles.(roles{index}).initialState.x = int32(index);
    scenario.vehicles.(roles{index}).initialState.notes = "retained";
end
before = scenario;
created = recreate(scenario);
avlc.scenario.validateScenario(scenario);
testCase.verifyEqual(scenario, before);
testCase.verifyEqual(created.road, before.road);
testCase.verifyEqual(created.theoryParameters, before.theoryParameters);
testCase.verifyEqual(created.vehicles, before.vehicles);
end

function testCorrectRolesAndUniqueIdsAccepted(testCase)
scenario = validScenario();
for role = ["EGO", "FO", "RO", "RT"]
    testCase.verifyEqual(scenario.vehicles.(role).role, role);
end
avlc.scenario.validateScenario(scenario);
end

function testSwappedFoAndRtRejected(testCase)
scenario = validScenario();
originalFo = scenario.vehicles.FO;
scenario.vehicles.FO = scenario.vehicles.RT;
scenario.vehicles.RT = originalFo;
verifyInvalidBoth(testCase, scenario);
end

function testEgoInFoSlotRejected(testCase)
scenario = validScenario();
scenario.vehicles.FO = scenario.vehicles.EGO;
verifyInvalidBoth(testCase, scenario);
end

function testCorruptedAndWrongInternalRolesRejected(testCase)
for slot = {'EGO', 'FO', 'RO', 'RT'}
    for value = {"NPC", lower(string(slot{1})), "RO"}
        if isequal(value{1}, string(slot{1}))
            continue;
        end
        scenario = validScenario();
        scenario.vehicles.(slot{1}).role = value{1};
        verifyInvalidBoth(testCase, scenario);
    end
end
end

function testAllDuplicateIdPairsRejected(testCase)
roles = {'EGO', 'FO', 'RO', 'RT'};
for first = 1:3
    for second = first+1:4
        scenario = validScenario();
        scenario.vehicles.(roles{second}).id = char(scenario.vehicles.(roles{first}).id);
        verifyInvalidBoth(testCase, scenario);
    end
end
end

function testIdMatchingIsCaseSensitive(testCase)
scenario = validScenario();
scenario.vehicles.EGO.id = "ego01";
scenario.vehicles.FO.id = "EGO01";
avlc.scenario.validateScenario(scenario);
created = recreate(scenario);
testCase.verifyEqual(created.vehicles.FO.id, "EGO01");
end

function testInvalidRoadWrappedWithCause(testCase)
scenario = validScenario();
scenario.road.laneWidth = -1;
verifyNestedFailure(testCase, scenario, 'AVLC:InvalidRoad', 'Road');
end

function testInvalidTheoryParametersWrappedWithCause(testCase)
scenario = validScenario();
scenario.theoryParameters.maxLateralAccel = 0;
verifyNestedFailure(testCase, scenario, 'AVLC:InvalidTheoryModelParameters', 'theoryParameters');
end

function testEachInvalidVehicleWrappedWithCause(testCase)
for role = {'EGO', 'FO', 'RO', 'RT'}
    scenario = validScenario();
    scenario.vehicles.(role{1}).geometry.length = 0;
    verifyNestedFailure(testCase, scenario, 'AVLC:InvalidVehicle', role{1});
end
end

function testEveryRequiredTopLevelFieldRequired(testCase)
scenario = validScenario();
fields = fieldnames(scenario);
for index = 1:numel(fields)
    invalid = rmfield(scenario, fields{index});
    testCase.verifyError(@() avlc.scenario.validateScenario(invalid), 'AVLC:InvalidScenario');
end
end

function testEveryRequiredVehicleSlotRequired(testCase)
scenario = validScenario();
for role = {'EGO', 'FO', 'RO', 'RT'}
    invalid = scenario;
    invalid.vehicles = rmfield(invalid.vehicles, role{1});
    testCase.verifyError(@() avlc.scenario.validateScenario(invalid), 'AVLC:InvalidScenario');
end
end

function testMissingNestedVehicleFieldsWrappedWithCause(testCase)
for role = {'EGO', 'FO', 'RO', 'RT'}
    scenario = validScenario();
    scenario.vehicles.(role{1}) = rmfield(scenario.vehicles.(role{1}), 'id');
    verifyNestedFailure(testCase, scenario, 'AVLC:InvalidVehicle', role{1});
end
end

function testInvalidNameRejected(testCase)
for value = {"", '', ["a", "b"], 1, true, string(missing), ['ab'; 'cd'], {'name'}}
    scenario = validScenario();
    scenario.name = value{1};
    verifyInvalidBoth(testCase, scenario);
end
end

function testInvalidDescriptionRejected(testCase)
for value = {[], 1, true, ["a", "b"], string(missing), ['ab'; 'cd'], {'text'}}
    scenario = validScenario();
    scenario.description = value{1};
    verifyInvalidBoth(testCase, scenario);
end
end

function testEventsMustBeEmptyStruct(testCase)
scenario = validScenario();
for value = {struct([]), repmat(struct('futureField', []), 0, 2)}
    scenario.events = value{1};
    avlc.scenario.validateScenario(scenario);
end
for value = {[], {}, "", struct(), struct('time', 1)}
    scenario.events = value{1};
    testCase.verifyError(@() avlc.scenario.validateScenario(scenario), 'AVLC:InvalidScenario');
end
end

function testScalarContainersRequired(testCase)
scenario = validScenario();
for value = {[], struct([]), [scenario, scenario], {scenario}, 1}
    testCase.verifyError(@() avlc.scenario.validateScenario(value{1}), 'AVLC:InvalidScenario');
end
for value = {[], struct([]), [scenario.vehicles, scenario.vehicles], 1}
    invalid = scenario;
    invalid.vehicles = value{1};
    testCase.verifyError(@() avlc.scenario.validateScenario(invalid), 'AVLC:InvalidScenario');
end
end

function testUnusualLongitudinalOrderingAccepted(testCase)
scenario = validScenario();
scenario.vehicles.FO.initialState.x = -20;
scenario.vehicles.RO.initialState.x = 20;
scenario.vehicles.RT.initialState.x = 30;
avlc.scenario.validateScenario(scenario);
avlc.scenario.validateScenario(recreate(scenario));
end

function testOffCenterLateralPositionsAccepted(testCase)
scenario = validScenario();
scenario.vehicles.EGO.initialState.y = 1.8;
scenario.vehicles.FO.initialState.y = -2;
scenario.vehicles.RO.initialState.y = 5;
scenario.vehicles.RT.initialState.y = 0.5;
avlc.scenario.validateScenario(scenario);
avlc.scenario.validateScenario(recreate(scenario));
end

function testNegativePositionsAndVelocitiesAccepted(testCase)
scenario = validScenario();
for role = {'EGO', 'FO', 'RO', 'RT'}
    scenario.vehicles.(role{1}).initialState.x = -100;
    scenario.vehicles.(role{1}).initialState.vx = -10;
end
avlc.scenario.validateScenario(scenario);
avlc.scenario.validateScenario(recreate(scenario));
end

function testOverlapAndFutureNegativeSafetyGapAccepted(testCase)
scenario = validScenario();
% All centers coincide, lengths are 4.5 m and clearances are 5 m.
% This would give negative future safety gaps; no gap is computed here.
for role = {'EGO', 'FO', 'RO', 'RT'}
    scenario.vehicles.(role{1}).initialState.x = 0;
    scenario.vehicles.(role{1}).initialState.y = 0;
end
avlc.scenario.validateScenario(scenario);
avlc.scenario.validateScenario(recreate(scenario));
end

function testAccelerationSnapshotsAreNotBoundedByTheory(testCase)
scenario = validScenario();
scenario.vehicles.EGO.initialState.ax = 1000;
scenario.vehicles.EGO.initialState.ay = -1000;
scenario.vehicles.EGO.initialState.vy = -10;
avlc.scenario.validateScenario(scenario);
avlc.scenario.validateScenario(recreate(scenario));
end

function testAdditionalFieldsAreNonAuthoritative(testCase)
scenario = validScenario();
scenario.notes = "future metadata";
scenario.vehicles.FT = "not an authoritative participant";
before = scenario;
avlc.scenario.validateScenario(scenario);
testCase.verifyEqual(scenario, before);
created = recreate(scenario);
testCase.verifyFalse(isfield(created, 'notes'));
testCase.verifyFalse(isfield(created.vehicles, 'FT'));
end

function verifyInvalidBoth(testCase, scenario)
testCase.verifyError(@() avlc.scenario.validateScenario(scenario), 'AVLC:InvalidScenario');
testCase.verifyError(@() recreate(scenario), 'AVLC:InvalidScenario');
end

function verifyNestedFailure(testCase, scenario, causeIdentifier, label)
verifyInvalidBoth(testCase, scenario);
try
    avlc.scenario.validateScenario(scenario);
catch exception
    testCase.verifyEqual(exception.identifier, 'AVLC:InvalidScenario');
    testCase.assertNotEmpty(exception.cause);
    testCase.verifyEqual(exception.cause{1}.identifier, causeIdentifier);
    testCase.verifyTrue(contains(exception.message, label));
    return;
end
testCase.assertFail('Expected a Scenario error with a nested cause.');
end

function created = recreate(scenario)
created = avlc.scenario.createScenario(scenario.name, scenario.description, ...
    scenario.road, scenario.theoryParameters, scenario.vehicles.EGO, ...
    scenario.vehicles.FO, scenario.vehicles.RO, scenario.vehicles.RT);
end

function scenario = validScenario()
road = avlc.model.createRoad(3.6, [-100, 100]);
params = avlc.model.createTheoryModelParameters(2, 4, 3, 0.5, 5, 5, 5);
roles = {'EGO', 'FO', 'RO', 'RT'};
vehicles = cell(1, 4);
for index = 1:4
    state = avlc.model.createVehicleState(0, 0, 10, 0, 0, 0);
    vehicles{index} = avlc.model.createVehicle("id-" + index, roles{index}, 4.5, 1.8, state);
end
scenario = avlc.scenario.createScenario("experiment", "structural fixture", ...
    road, params, vehicles{:});
end
