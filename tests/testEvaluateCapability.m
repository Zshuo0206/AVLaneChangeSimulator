function tests = testEvaluateCapability
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testExactResultSchemaAndLogicalFlags(testCase)
[s, z] = fixture();
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(fieldnames(cap), {'abort'; 'completion'; 'joint'});
testCase.verifyEqual(fieldnames(cap.abort), ...
    {'TRsafe'; 'TFsafe'; 'TG'; 'HO'; 'TOmin'; 'YO'; 'feasible'});
testCase.verifyEqual(fieldnames(cap.completion), ...
    {'HT'; 'DT'; 'D0'; 'Dphys'; 'TTmin'; 'Dplus'; 'Dminus'; 'YTin'; 'YTout'; 'feasible'});
testCase.verifyEqual(fieldnames(cap.joint), ...
    {'dualCapable'; 'finiteGeometryAvailable'; 'Yfwd'; 'Gamma'});
testCase.verifyClass(cap.abort.feasible, 'logical');
testCase.verifyClass(cap.completion.feasible, 'logical');
testCase.verifyClass(cap.joint.dualCapable, 'logical');
testCase.verifyClass(cap.joint.finiteGeometryAvailable, 'logical');
end

function testRearOpportunityBranchesIncludingClosedEquality(testCase)
[s, z] = fixture();
% C=2: equality at c=4,g=4 is infinite; c=5,g=4 gives (5-3)/2=1.
cases = [-1, 1, Inf; 0, 1, Inf; 2, 2, Inf; 4, 4, Inf; 5, 4, 1; 2, 0, 0];
for index = 1:size(cases, 1)
    z.cR = cases(index, 1);
    z.gR = cases(index, 2);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.abort.TRsafe, cases(index, 3));
end
end

function testFrontOpportunityBranchesIncludingClosedEquality(testCase)
[s, z] = fixture();
% B=4: equality at c=4,g=2 is infinite; c=5,g=2 gives (5-3)/4=0.5.
cases = [-1, 1, Inf; 0, 1, Inf; 2, 2, Inf; 4, 2, Inf; 5, 2, 0.5; 2, 0, 0];
for index = 1:size(cases, 1)
    z.cF = cases(index, 1);
    z.gF = cases(index, 2);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.abort.TFsafe, cases(index, 3));
end
end

function testTargetOpportunityBranchesIncludingClosedEquality(testCase)
[s, z] = fixture();
cases = [-1, 1, Inf; 0, 1, Inf; 2, 2, Inf; 4, 4, Inf; 5, 4, 1; 2, 0, 0];
for index = 1:size(cases, 1)
    z.cT = cases(index, 1);
    z.gT = cases(index, 2);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.completion.HT, cases(index, 3));
end
end

function testOpportunityDiscriminantsHaveNoBranchTolerance(testCase)
[s, z] = fixture();
% Move just below/above the exact safe stopping gap by a binary amount.
delta = 2^-40;
for name = {'R', 'F', 'T'}
    changed = z;
    changed.(['c' name{1}]) = 4;
    boundaryGap = 4;
    if strcmp(name{1}, 'F')
        boundaryGap = 2;
    end
    changed.(['g' name{1}]) = boundaryGap - delta;
    below = avlc.analysis.evaluateCapability(s, changed);
    changed.(['g' name{1}]) = boundaryGap + delta;
    above = avlc.analysis.evaluateCapability(s, changed);
    if strcmp(name{1}, 'T')
        testCase.verifyTrue(isfinite(below.completion.HT));
        testCase.verifyEqual(above.completion.HT, Inf);
    else
        field = ['T' name{1} 'safe'];
        testCase.verifyTrue(isfinite(below.abort.(field)));
        testCase.verifyEqual(above.abort.(field), Inf);
    end
end
end

function testTotalGapOpportunityBranches(testCase)
[s, z] = fixture();
z.gR = 6;
z.gF = 9;
cases = [1, -2, Inf; 2, -2, Inf; 2, 1, 5];
for index = 1:3
    z.cR = cases(index, 1);
    z.cF = cases(index, 2);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.abort.TG, cases(index, 3));
end
end

function testEachOpportunityCanIndependentlyDefineHO(testCase)
[s, z] = fixture();
% Columns: gR,cR,gF,cF,TRsafe,TFsafe,TG,HO.
cases = [4,5,100,-5,1,Inf,Inf,1; ...
    100,-5,2,5,Inf,0.5,Inf,0.5; 6,2,9,1,Inf,Inf,5,5];
for index = 1:3
    z.gR = cases(index, 1); z.cR = cases(index, 2);
    z.gF = cases(index, 3); z.cF = cases(index, 4);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual([cap.abort.TRsafe, cap.abort.TFsafe, cap.abort.TG, cap.abort.HO], ...
        cases(index, 5:8));
end
end

function testAbortMinimumTimeHandCalculation(testCase)
[s, z] = fixture();
z.y = 1; z.w = 2;
% A=2: (2+sqrt(8+8))/2=3.
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.abort.TOmin, 3);
end

function testAbortClosedTimeBoundaryAndFiniteFrontier(testCase)
[s, z] = fixture();
z.w = 2; z.gR = 8; z.cR = 2; z.gF = 8; z.cF = 2;
% HO=TG=4; YO=8-4-0.5=3.5.
positions = [3, 3.5, 4];
expected = [true, true, false];
for index = 1:3
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.abort.HO, 4);
    testCase.verifyEqual(cap.abort.YO, 3.5);
    testCase.verifyEqual(cap.abort.feasible, expected(index));
    if index == 1
        testCase.verifyLessThan(cap.abort.TOmin, 4);
    elseif index == 2
        testCase.verifyEqual(cap.abort.TOmin, 4);
    else
        testCase.verifyGreaterThan(cap.abort.TOmin, 4);
    end
end
end

function testNegativeFiniteAbortFrontierIsNotClipped(testCase)
[s, z] = fixture();
z.y = 0; z.w = 2; z.gR = 4; z.cR = 5;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.abort.HO, 1);
testCase.verifyEqual(cap.abort.YO, -1);
testCase.verifyFalse(cap.abort.feasible);
end

function testInfiniteAbortOpportunityHasUndefinedFiniteFrontier(testCase)
[s, z] = fixture();
z.y = 50; z.w = 3;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.abort.HO, Inf);
testCase.verifyTrue(isfinite(cap.abort.TOmin));
testCase.verifyTrue(isnan(cap.abort.YO));
testCase.verifyTrue(cap.abort.feasible);
end

function testNegativeAbortGapsDisableOnlyAbortBranch(testCase)
[s, z] = fixture();
for name = {'gR', 'gF'}
    changed = z;
    changed.(name{1}) = -2^-40;
    cap = avlc.analysis.evaluateCapability(s, changed);
    testCase.verifyTrue(all(isnan([cap.abort.TRsafe, cap.abort.TFsafe, ...
        cap.abort.TG, cap.abort.HO, cap.abort.YO])));
    testCase.verifyTrue(isfinite(cap.abort.TOmin));
    testCase.verifyFalse(cap.abort.feasible);
    testCase.verifyTrue(cap.completion.feasible);
    testCase.verifyFalse(cap.joint.dualCapable);
end
end

function testAllCompletionMinimumTimeBranches(testCase)
[s, z] = fixture();
z.w = 2;
% D0=1,Dphys=0. Include both exact switch equalities.
distances = [2, 1, 0.5, 0, -0.25];
expected = [sqrt(6)-1, 1, 2, 1+sqrt(2), Inf];
for index = 1:5
    z.y = 4 - distances(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual([cap.completion.DT, cap.completion.D0, cap.completion.Dphys], ...
        [distances(index), 1, 0]);
    testCase.verifyEqual(cap.completion.TTmin, expected(index), 'AbsTol', 1e-14);
end
end

function testCompletionSwitchesHaveNoBranchTolerance(testCase)
[s, z] = fixture();
z.w = 2;
z.y = 3 + 2^-30;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.completion.TTmin, 1 + sqrt(2^-29));
z.y = 4 + 2^-30;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.completion.TTmin, Inf);
testCase.verifyFalse(cap.completion.feasible);
end

function testFiniteCompletionTimeComparisonIncludingEquality(testCase)
[s, z] = fixture();
z.gT = 6; z.cT = 5;
positions = [2.5, 2, 1.5];
times = [sqrt(3), 2, sqrt(5)];
expected = [true, true, false];
for index = 1:3
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.completion.HT, 2);
    testCase.verifyEqual(cap.completion.TTmin, times(index), 'AbsTol', 1e-14);
    testCase.verifyEqual(cap.completion.feasible, expected(index));
end
end

function testInfiniteTargetOpportunityStillRequiresPhysicalRecovery(testCase)
[s, z] = fixture();
z.w = 2;
positions = [3.5, 4, 4.25];
expected = [true, true, false];
for index = 1:3
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.completion.HT, Inf);
    testCase.verifyEqual(cap.completion.feasible, expected(index));
    testCase.verifyTrue(all(isnan([cap.completion.Dplus, cap.completion.Dminus, ...
        cap.completion.YTin, cap.completion.YTout])));
    if index == 3
        testCase.verifyEqual(cap.completion.TTmin, Inf);
        testCase.verifyFalse(cap.joint.dualCapable);
    end
end
end

function testFiniteCompletionWindowAndBothClosedEndpoints(testCase)
[s, z] = fixture();
z.w = 2; z.gT = 6; z.cT = 5;
% A=2,H=2,w=2: Dplus=3.5,Dminus=0.5,YTin=0.5,YTout=3.5.
positions = [0.25, 0.5, 1, 3.5, 3.75];
expected = [false, true, true, true, false];
for index = 1:numel(positions)
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual([cap.completion.Dplus, cap.completion.Dminus, ...
        cap.completion.YTin, cap.completion.YTout], [3.5, 0.5, 0.5, 3.5]);
    testCase.verifyEqual(cap.completion.feasible, expected(index));
    testCase.verifyEqual(cap.completion.feasible, ...
        cap.completion.YTin <= z.y && z.y <= cap.completion.YTout);
end
end

function testPhysicalOvershootDefinesFiniteUpperBoundary(testCase)
[s, z] = fixture();
s.theoryParameters.targetOvershoot = 0.25;
z.w = 2; z.gT = 6; z.cT = 5;
positions = [3.25-2^-30, 3.25, 3.25+2^-30];
expected = [true, true, false];
for index = 1:3
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.completion.Dphys, 0.75);
    testCase.verifyEqual(cap.completion.Dminus, 0.5);
    testCase.verifyEqual(cap.completion.YTout, 3.25);
    testCase.verifyEqual(cap.completion.feasible, expected(index));
end
end

function testInsufficientFiniteOpportunityHasNoWindow(testCase)
[s, z] = fixture();
z.w = 2; z.gR = 8; z.cR = 2; z.gF = 8; z.cF = 2;
z.gT = 1.25; z.cT = 3;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.completion.HT, 0.5);
testCase.verifyFalse(cap.completion.feasible);
testCase.verifyTrue(all(isnan([cap.completion.Dplus, cap.completion.Dminus, ...
    cap.completion.YTin, cap.completion.YTout])));
testCase.verifyFalse(cap.joint.finiteGeometryAvailable);
testCase.verifyTrue(all(isnan([cap.joint.Yfwd, cap.joint.Gamma])));
end

function testOpportunityEqualToMinimumPossibleTimeIsClosedSingleton(testCase)
[s, z] = fixture();
z.w = 2; z.gT = 4; z.cT = 5;
positions = [3-2^-30, 3, 3+2^-30];
for index = 1:3
    z.y = positions(index);
    cap = avlc.analysis.evaluateCapability(s, z);
    testCase.verifyEqual(cap.completion.HT, 1);
    testCase.verifyEqual([cap.completion.Dplus, cap.completion.Dminus, ...
        cap.completion.YTin, cap.completion.YTout], [1, 1, 3, 3]);
    testCase.verifyEqual(cap.completion.feasible, index == 2);
end
end

function testNegativeTargetGapDisablesOnlyCompletion(testCase)
[s, z] = fixture();
z.gT = -2^-40;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyTrue(isnan(cap.completion.HT));
testCase.verifyTrue(all(isnan([cap.completion.Dplus, cap.completion.Dminus, ...
    cap.completion.YTin, cap.completion.YTout])));
testCase.verifyTrue(isfinite(cap.completion.TTmin));
testCase.verifyFalse(cap.completion.feasible);
testCase.verifyTrue(cap.abort.feasible);
testCase.verifyFalse(cap.joint.dualCapable);
testCase.verifyFalse(cap.joint.finiteGeometryAvailable);
end

function testFiniteJointGeometryGammaSignsAndClosedInterval(testCase)
[s, z] = fixture();
z.w = 2; z.cR = 2; z.cF = 2; z.gT = 6; z.cT = 5;
% Columns: HO,YO,Yfwd,Gamma. Include a defined but empty lateral slice.
cases = [1,-1,-1,4.5; 3,1,1,2.5; 4,3.5,3.5,0; 5,7,3.5,-3.5];
positions = [0, 0.25, 0.5, 0.75, 1, 1.25, 3, 3.5, 3.75, 5];
for index = 1:4
    z.gR = 2*cases(index, 1);
    z.gF = 2*cases(index, 1);
    for position = positions
        z.y = position;
        cap = avlc.analysis.evaluateCapability(s, z);
        testCase.verifyTrue(cap.joint.finiteGeometryAvailable);
        testCase.verifyEqual([cap.abort.HO, cap.abort.YO, cap.completion.YTout, ...
            cap.joint.Yfwd, cap.joint.Gamma], [cases(index, 1:2), 3.5, cases(index, 3:4)]);
        testCase.verifyEqual(cap.joint.dualCapable, position >= 0.5 && position <= cases(index, 3));
        testCase.verifyEqual(cap.joint.dualCapable, ...
            cap.completion.YTin <= position && position <= cap.joint.Yfwd);
        testCase.verifyEqual(cap.joint.dualCapable, cap.abort.feasible && cap.completion.feasible);
    end
end
end

function testNonfiniteJointGeometryKeepsDirectCapabilityBooleans(testCase)
[s, z] = fixture();
z.w = 2;
for index = 1:3
    changed = z;
    if index ~= 1
        changed.gR = 8; changed.cR = 2; changed.gF = 8; changed.cF = 2;
    end
    if index ~= 2
        changed.gT = 6; changed.cT = 5;
    end
    % index=1: HO Inf; index=2: HT Inf; index=3: both finite.
    cap = avlc.analysis.evaluateCapability(s, changed);
    testCase.verifyTrue(cap.joint.dualCapable);
    testCase.verifyEqual(cap.joint.finiteGeometryAvailable, index == 3);
    if index ~= 3
        testCase.verifyTrue(all(isnan([cap.joint.Yfwd, cap.joint.Gamma])));
    end
end
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyTrue(cap.joint.dualCapable);
testCase.verifyFalse(cap.joint.finiteGeometryAvailable);
testCase.verifyTrue(all(isnan([cap.joint.Yfwd, cap.joint.Gamma])));
end

function testPredecisionDomainIsEnforcedOnlyByEvaluator(testCase)
[s, z] = fixture();
for name = {'y', 'w'}
    changed = z;
    changed.(name{1}) = -2^-40;
    testCase.verifyError(@() avlc.analysis.evaluateCapability(s, changed), ...
        'AVLC:CapabilityDomainViolation');
end
physical = avlc.model.createVehicleState(0, -1, 0, -2, 0, 0);
surrounding = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
mapped = avlc.analysis.buildTheoryState(s, physical, surrounding, surrounding, surrounding);
testCase.verifyEqual([mapped.y, mapped.w], [-1, -2]);
end

function testZeroTimeClosedSafetyBoundaries(testCase)
[s, z] = fixture();
z.y = 0; z.w = 0; z.gR = 0; z.cR = 5; z.gF = 0;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual([cap.abort.HO, cap.abort.TOmin, cap.abort.YO], [0, 0, 0]);
testCase.verifyTrue(cap.abort.feasible);
z.y = 4; z.gT = 0; z.cT = 5;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual([cap.completion.HT, cap.completion.TTmin, ...
    cap.completion.YTin, cap.completion.YTout], [0, 0, 4, 4]);
testCase.verifyTrue(cap.completion.feasible);
end

function testZeroOvershootAtNaturalStoppingBoundary(testCase)
[s, z] = fixture();
s.theoryParameters.targetOvershoot = 0;
z.w = 2; z.y = 3;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual([cap.completion.DT, cap.completion.D0, cap.completion.Dphys, ...
    cap.completion.TTmin], [1, 1, 1, 1]);
testCase.verifyTrue(cap.completion.feasible);
z.y = 3 + 2^-30;
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.completion.TTmin, Inf);
testCase.verifyFalse(cap.completion.feasible);
end

function testMalformedTheoryStateUsesSpecifiedError(testCase)
[s, z] = fixture();
invalid = {[], struct(), repmat(z, 2, 1), rmfield(z, 'gT')};
values = {NaN, Inf, 1i, [1, 2], "bad", true, []};
for index = 1:numel(values)
    changed = z;
    changed.cR = values{index};
    invalid{end+1} = changed; %#ok<AGROW>
end
for index = 1:numel(invalid)
    testCase.verifyError(@() avlc.analysis.evaluateCapability(s, invalid{index}), ...
        'AVLC:InvalidTheoryState');
end
end

function testScenarioValidationIsReused(testCase)
[s, z] = fixture();
testCase.verifyError(@() avlc.analysis.evaluateCapability(struct(), z), 'AVLC:InvalidScenario');
s.road.laneWidth = -1;
testCase.verifyError(@() avlc.analysis.evaluateCapability(s, z), 'AVLC:InvalidScenario');
end

function testCompatibleExtraFieldsAndInputImmutability(testCase)
[s, z] = fixture();
s.notes = "Scenario metadata";
z.notes = "reduced-state metadata";
before = {s, z};
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyTrue(cap.joint.dualCapable);
testCase.verifyEqual({s, z}, before);
testCase.verifyFalse(isfield(s, 'W'));
testCase.verifyFalse(isfield(s.theoryParameters, 'W'));
end

function testSuppliedReducedStateAndRoadAreAuthoritative(testCase)
[s, z] = fixture();
expected = avlc.analysis.evaluateCapability(s, z);
for role = {'EGO', 'FO', 'RO', 'RT'}
    s.vehicles.(role{1}).initialState = avlc.model.createVehicleState(-1000, -2000, -30, -40, 1e100, -1e100);
    s.vehicles.(role{1}).geometry.length = 100;
end
testCase.verifyEqual(avlc.analysis.evaluateCapability(s, z), expected);
s.road = avlc.model.createRoad(8, [-100, 100]);
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual(cap.completion.DT, 7);
testCase.verifyEqual(cap.completion.TTmin, sqrt(14), 'AbsTol', 1e-14);
end

function testValidNumericRepresentationsUseDoubleArithmetic(testCase)
[s, z] = fixture();
s.theoryParameters.maxLateralAccel = uint8(2);
s.theoryParameters.maxLongitudinalBrake = uint8(4);
s.theoryParameters.maxLongitudinalAccel = uint8(2);
z.y = uint8(1); z.w = uint8(2);
z.gR = uint8(8); z.cR = uint8(2); z.gF = uint8(8); z.cF = uint8(2);
z.gT = single(6); z.cT = single(5);
cap = avlc.analysis.evaluateCapability(s, z);
testCase.verifyEqual([cap.abort.TOmin, cap.abort.HO, cap.abort.YO], [3, 4, 3.5]);
testCase.verifyEqual([cap.completion.Dplus, cap.completion.Dminus], [3.5, 0.5]);
testCase.verifyTrue(cap.joint.dualCapable);
end

function testSimulationToTheoryToCurrentCapabilityIntegration(testCase)
[s, ~] = fixture();
s.vehicles.EGO.initialState = avlc.model.createVehicleState(0, 0, 10, 0, 0, 0);
s.vehicles.FO.initialState = avlc.model.createVehicleState(16, 0, 8, 0, 0, 0);
s.vehicles.RO.initialState = avlc.model.createVehicleState(-16, 0, 12, 0, 0, 0);
s.vehicles.RT.initialState = avlc.model.createVehicleState(-18.5, 4, 15, 0, 0, 0);
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.5, 1), ...
    avlc.model.createControl(0, 2));
z = avlc.analysis.buildTheoryState(s, result.states.EGO(3), result.states.FO(3), ...
    result.states.RO(3), result.states.RT(3));
testCase.verifyEqual(cell2mat(struct2cell(z)).', [1, 2, 8, 2, 8, 2, 6, 5]);
cap = avlc.analysis.evaluateCapability(s, z);
% At t=1: HO=16/4=4, TOmin=3, HT=(5-1)/2=2, TTmin=2sqrt(2)-1.
expected.abort = struct('TRsafe', Inf, 'TFsafe', Inf, 'TG', 4, 'HO', 4, ...
    'TOmin', 3, 'YO', 3.5, 'feasible', true);
expected.completion = struct('HT', 2, 'DT', 3, 'D0', 1, 'Dphys', 0, ...
    'TTmin', 2*sqrt(2)-1, 'Dplus', 3.5, 'Dminus', 0.5, ...
    'YTin', 0.5, 'YTout', 3.5, 'feasible', true);
expected.joint = struct('dualCapable', true, 'finiteGeometryAvailable', true, 'Yfwd', 3.5, 'Gamma', 0);
testCase.verifyEqual(cap, expected, 'AbsTol', 1e-14);
end

function [scenario, z] = fixture()
road = avlc.model.createRoad(4, [-100, 100]);
params = avlc.model.createTheoryModelParameters(2, 4, 2, 1, 1, 1.5, 2);
roles = {'EGO', 'FO', 'RO', 'RT'};
lengths = [4, 6, 5, 7];
initial = avlc.model.createVehicleState(0, 0, 0, 0, 0, 0);
vehicles = cell(1, 4);
for index = 1:4
    vehicles{index} = avlc.model.createVehicle(string(index), roles{index}, lengths(index), 2, initial);
end
scenario = avlc.scenario.createScenario("capability fixture", "", road, params, vehicles{:});
z = struct('y', 1, 'w', 0, 'gR', 100, 'cR', 0, 'gF', 100, 'cF', 0, 'gT', 100, 'cT', 0);
end
