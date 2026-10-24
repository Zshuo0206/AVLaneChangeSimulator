function tests = testRoad
%TESTROAD Contract tests for the Phase 0 two-lane road data model.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidRoadAndExactFields(testCase)
road = avlc.model.createRoad(3.6, [-50, 150]);
testCase.verifyEqual(fieldnames(road), {'numLanes'; 'laneWidth'; ...
    'originalLaneCenterY'; 'targetLaneCenterY'; 'displayXRange'});
testCase.verifyEqual(road.numLanes, 2, 'AbsTol', 1e-10);
testCase.verifyEqual(road.originalLaneCenterY, 0, 'AbsTol', 1e-10);
testCase.verifyEqual(road.targetLaneCenterY, road.laneWidth, 'AbsTol', 1e-10);
testCase.verifyEqual(road.displayXRange, [-50, 150], 'AbsTol', 1e-10);
avlc.model.validateRoad(road);
end

function testColumnRangeAndNumericNormalization(testCase)
road = avlc.model.createRoad(int16(4), int16([-10; 20]));
testCase.verifyClass(road.laneWidth, 'double');
testCase.verifyClass(road.targetLaneCenterY, 'double');
testCase.verifyClass(road.displayXRange, 'double');
testCase.verifySize(road.displayXRange, [1, 2]);
testCase.verifyEqual(road.displayXRange, [-10, 20], 'AbsTol', 1e-10);
avlc.model.validateRoad(road);
end

function testInvalidLaneWidthRejected(testCase)
for value = {0, -1, NaN, Inf, -Inf, 1 + 1i, [3, 4], [], '3.6', true, {3.6}}
    testCase.verifyError(@() avlc.model.createRoad(value{1}, [-10, 20]), ...
        'AVLC:InvalidRoad');
    road = validRoad();
    road.laneWidth = value{1};
    testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end
end

function testInvalidDisplayRangeRejected(testCase)
values = {[20, -10], [10, 10], [0, NaN], [0, Inf], [0, 1 + 1i], ...
    [], 1, [0, 1, 2], [0, 1; 2, 3], 'ab', "range", [false, true], {0, 1}};
for index = 1:numel(values)
    range = values{index};
    testCase.verifyError(@() avlc.model.createRoad(3.6, range), 'AVLC:InvalidRoad');
    road = validRoad();
    road.displayXRange = range;
    testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end
end

function testMissingFieldsRejected(testCase)
road = validRoad();
fields = fieldnames(road);
for index = 1:numel(fields)
    invalid = rmfield(road, fields{index});
    testCase.verifyError(@() avlc.model.validateRoad(invalid), 'AVLC:InvalidRoad');
end
end

function testScalarStructRequired(testCase)
road = validRoad();
for value = {[], struct([]), [road, road], 42, {road}}
    testCase.verifyError(@() avlc.model.validateRoad(value{1}), 'AVLC:InvalidRoad');
end
end

function testExactlyTwoLanesRequired(testCase)
for value = {0, 1, 3, 2.1}
    road = validRoad();
    road.numLanes = value{1};
    testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end
end

function testOriginalCenterMustBeZero(testCase)
road = validRoad();
road.originalLaneCenterY = 1e-12;
testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end

function testInconsistentTargetCenterRejected(testCase)
road = validRoad();
road.targetLaneCenterY = road.laneWidth + 0.1;
testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end

function testTargetCenterTolerance(testCase)
road = validRoad();
road.targetLaneCenterY = road.laneWidth + 1e-12;
avlc.model.validateRoad(road);
road.targetLaneCenterY = road.laneWidth + 1e-8;
testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end

function testScalarRoadFieldsRejectMalformedValues(testCase)
for field = {'numLanes', 'laneWidth', 'originalLaneCenterY', 'targetLaneCenterY'}
    for value = {NaN, Inf, 1 + 1i, [1, 2], [], true, "2"}
        road = validRoad();
        road.(field{1}) = value{1};
        testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
    end
end
end

function testSmallPositiveLaneWidthIsNotClipped(testCase)
road = avlc.model.createRoad(1e-12, [-1, 1]);
avlc.model.validateRoad(road);
testCase.verifyEqual(road.laneWidth, 1e-12, 'RelTol', 1e-10);
road.targetLaneCenterY = 2e-12;
testCase.verifyError(@() avlc.model.validateRoad(road), 'AVLC:InvalidRoad');
end

function testAdditionalFieldsAllowedWithoutMutation(testCase)
road = validRoad();
road.notes = "display metadata";
before = road;
avlc.model.validateRoad(road);
testCase.verifyEqual(road, before);
end

function road = validRoad()
road = avlc.model.createRoad(3.6, [-50, 150]);
end
