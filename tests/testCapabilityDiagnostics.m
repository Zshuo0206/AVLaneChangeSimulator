function tests = testCapabilityDiagnostics
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testExactSchemaAndHandCalculatedReserves(testCase)
[s, z] = fixture();
d = diagnose(s, z);
testCase.verifyEqual(fieldnames(d), {'reserves'; 'abort'; 'completion'});
testCase.verifyEqual(fieldnames(d.reserves), {'rO'; 'lT'; 'rT'});
testCase.verifyEqual(fieldnames(d.abort), {'activeSetDefined'; 'activeR'; 'activeF'; 'activeG'});
testCase.verifyEqual(fieldnames(d.completion), ...
    {'currentBranch'; 'upperActiveSetDefined'; 'upperActiveI'; 'upperActiveII'});
% A=2,w=2,HO=4,HT=2,W=4: YO=3.5,YTin=0.5,YTout=3.5,y=1.
testCase.verifyEqual([d.reserves.rO, d.reserves.lT, d.reserves.rT], [2.5, 0.5, 2.5]);
testCase.verifyClass(d.abort.activeR, 'logical');
testCase.verifyClass(d.abort.activeF, 'logical');
testCase.verifyClass(d.abort.activeG, 'logical');
testCase.verifyClass(d.completion.upperActiveI, 'logical');
testCase.verifyClass(d.completion.upperActiveII, 'logical');
testCase.verifyClass(d.completion.currentBranch, 'string');
end

function testAbortSignedReserveAndClosedBoundary(testCase)
[s, z] = fixture();
positions = [3, 3.5, 4];
expected = [0.5, 0, -0.5];
for index = 1:3
    z.y = positions(index);
    d = diagnose(s, z);
    testCase.verifyEqual(d.reserves.rO, expected(index));
end
end

function testCompletionEntrySignedReserveAndClosedBoundary(testCase)
[s, z] = fixture();
positions = [0.25, 0.5, 0.75];
expected = [-0.25, 0, 0.25];
for index = 1:3
    z.y = positions(index);
    d = diagnose(s, z);
    testCase.verifyEqual(d.reserves.lT, expected(index));
end
end

function testCompletionExitSignedReserveAndClosedBoundary(testCase)
[s, z] = fixture();
positions = [3, 3.5, 4];
expected = [0.5, 0, -0.5];
for index = 1:3
    z.y = positions(index);
    d = diagnose(s, z);
    testCase.verifyEqual(d.reserves.rT, expected(index));
end
end

function testNegativeFiniteFrontierRemainsSigned(testCase)
[s, z] = fixture();
z.y = 0; z.gR = 2; z.gF = 2;
[d, cap] = diagnose(s, z);
testCase.verifyEqual(cap.abort.YO, -1);
testCase.verifyEqual(d.reserves.rO, -1);
testCase.verifyTrue(d.abort.activeG);
end

function testUndefinedAbortFrontierDoesNotDisableCompletionDiagnostics(testCase)
[s, z] = fixture();
for index = 1:2
    changed = z;
    if index == 1
        changed.cR = 0; changed.cF = 0;
    else
        changed.gR = -1;
    end
    [d, cap] = diagnose(s, changed);
    testCase.verifyTrue(isnan(d.reserves.rO));
    testCase.verifyFalse(d.abort.activeSetDefined);
    testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [false, false, false]);
    testCase.verifyEqual([d.reserves.lT, d.reserves.rT], [0.5, 2.5]);
    testCase.verifyTrue(d.completion.upperActiveSetDefined);
    if index == 1
        testCase.verifyEqual(cap.abort.HO, Inf);
    else
        testCase.verifyTrue(isnan(cap.abort.HO));
    end
end
end

function testUndefinedCompletionWindowsPreserveNaNAndAbortDiagnostics(testCase)
[s, z] = fixture();
for index = 1:3
    changed = z;
    if index == 1
        changed.cT = 0; % Infinite HT.
    elseif index == 2
        changed.gT = 1.25; changed.cT = 3; % HT=0.5 < w/A=1.
    else
        changed.gT = -1;
    end
    d = diagnose(s, changed);
    testCase.verifyTrue(all(isnan([d.reserves.lT, d.reserves.rT])));
    testCase.verifyFalse(d.completion.upperActiveSetDefined);
    testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], [false, false]);
    testCase.verifyEqual(d.completion.currentBranch, "NON_OVERSHOOT");
    testCase.verifyEqual(d.reserves.rO, 2.5);
    testCase.verifyTrue(d.abort.activeG);
end
end

function testUniqueAbortOpportunityMechanisms(testCase)
[s, z] = fixture();
% gR,cR,gF,cF followed by the independently expected R/F/G masks.
cases = [4,5,100,-5,1,0,0; 100,-5,2,5,0,1,0; 8,2,8,2,0,0,1];
for index = 1:3
    z.gR = cases(index, 1); z.cR = cases(index, 2);
    z.gF = cases(index, 3); z.cF = cases(index, 4);
    d = diagnose(s, z);
    testCase.verifyTrue(d.abort.activeSetDefined);
    testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], logical(cases(index, 5:7)));
end
end

function testRearGapTiePreservesBothMechanisms(testCase)
[s, z] = fixture();
z.gR = 4; z.cR = 5; z.gF = 2; z.cF = 1;
[d, cap] = diagnose(s, z);
testCase.verifyEqual([cap.abort.TRsafe, cap.abort.TFsafe, cap.abort.TG, cap.abort.HO], [1, Inf, 1, 1]);
testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [true, false, true]);
end

function testFrontGapTiePreservesBothMechanisms(testCase)
[s, z] = fixture();
z.gR = 1; z.cR = 1; z.gF = 2; z.cF = 5;
[d, cap] = diagnose(s, z);
testCase.verifyEqual([cap.abort.TRsafe, cap.abort.TFsafe, cap.abort.TG, cap.abort.HO], [Inf, 0.5, 0.5, 0.5]);
testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [false, true, true]);
end

function testPositiveFrontRearEqualityIsShieldedByGapOpportunity(testCase)
[s, z] = fixture();
z.gR = 4; z.cR = 5; z.gF = 4; z.cF = 6;
[d, cap] = diagnose(s, z);
% Both one-sided roots equal 1, but TG=8/11<1 (Master section 55).
testCase.verifyEqual([cap.abort.TRsafe, cap.abort.TFsafe], [1, 1]);
testCase.verifyEqual(cap.abort.HO, 8/11);
testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [false, false, true]);
end

function testZeroTimeTripleTieRetainsFrontRearAndGap(testCase)
[s, z] = fixture();
z.y = 0; z.w = 0; z.gR = 0; z.cR = 1; z.gF = 0; z.cF = 1;
[d, cap] = diagnose(s, z);
testCase.verifyEqual([cap.abort.TRsafe, cap.abort.TFsafe, cap.abort.TG, cap.abort.HO], [0, 0, 0, 0]);
testCase.verifyTrue(d.abort.activeSetDefined);
testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [true, true, true]);
testCase.verifyEqual(d.reserves.rO, 0);
end

function testAbortNearTieDoesNotUseTolerance(testCase)
[s, z] = fixture();
z.gR = 4; z.cR = 5; z.cF = 1;
offsets = [-2^-38, 0, 2^-38];
expected = logical([0,0,1; 1,0,1; 1,0,0]);
for index = 1:3
    z.gF = 2 + offsets(index);
    d = diagnose(s, z);
    testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], expected(index, :));
end
end

function testCurrentCompletionBranchesAndClosedSwitches(testCase)
[s, z] = fixture();
distances = [2, 1, 0.5, 0, -0.25];
expected = ["NON_OVERSHOOT", "NON_OVERSHOOT", "OVERSHOOT_RETURN", ...
    "OVERSHOOT_RETURN", "PHYSICALLY_UNRECOVERABLE"];
for index = 1:5
    z.y = 4 - distances(index);
    [d, cap] = diagnose(s, z);
    testCase.verifyEqual([cap.completion.DT, cap.completion.D0, cap.completion.Dphys], ...
        [distances(index), 1, 0]);
    testCase.verifyEqual(d.completion.currentBranch, expected(index));
end
end

function testCurrentBranchSwitchesDoNotUseTolerance(testCase)
[s, z] = fixture();
positions = [3-2^-38, 3+2^-38, 4-2^-38, 4+2^-38];
expected = ["NON_OVERSHOOT", "OVERSHOOT_RETURN", "OVERSHOOT_RETURN", "PHYSICALLY_UNRECOVERABLE"];
for index = 1:4
    z.y = positions(index);
    d = diagnose(s, z);
    testCase.verifyEqual(d.completion.currentBranch, expected(index));
end
end

function testZeroOvershootCoincidentThresholdsUseFirstBranch(testCase)
[s, z] = fixture();
s.theoryParameters.targetOvershoot = 0;
z.y = 3;
[d, cap] = diagnose(s, z);
testCase.verifyEqual([cap.completion.DT, cap.completion.D0, cap.completion.Dphys], [1, 1, 1]);
testCase.verifyEqual(d.completion.currentBranch, "NON_OVERSHOOT");
z.y = 3 + 2^-38;
d = diagnose(s, z);
testCase.verifyEqual(d.completion.currentBranch, "PHYSICALLY_UNRECOVERABLE");
end

function testFiniteUpperMechanismsAndEqualityRetainBoth(testCase)
[s, z] = fixture();
overshoots = [1, 0.5, 0.25];
expected = logical([1,0; 1,1; 0,1]);
for index = 1:3
    s.theoryParameters.targetOvershoot = overshoots(index);
    [d, cap] = diagnose(s, z);
    testCase.verifyEqual(cap.completion.Dminus, 0.5);
    testCase.verifyTrue(d.completion.upperActiveSetDefined);
    testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], expected(index, :));
end
end

function testUpperMechanismNearTieDoesNotUseTolerance(testCase)
[s, z] = fixture();
offsets = [-2^-38, 0, 2^-38];
expected = logical([0,1; 1,1; 1,0]);
for index = 1:3
    s.theoryParameters.targetOvershoot = 0.5 + offsets(index);
    d = diagnose(s, z);
    testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], expected(index, :));
end
end

function testCurrentTimeBranchAndUpperMechanismAreIndependent(testCase)
[s, z] = fixture();
z.y = 2;
s.theoryParameters.targetOvershoot = 0.25;
d = diagnose(s, z);
testCase.verifyEqual(d.completion.currentBranch, "NON_OVERSHOOT");
testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], [false, true]);
z.y = 3.5;
s.theoryParameters.targetOvershoot = 1;
d = diagnose(s, z);
testCase.verifyEqual(d.completion.currentBranch, "OVERSHOOT_RETURN");
testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], [true, false]);
z.y = 4.25;
[d, cap] = diagnose(s, z);
testCase.verifyEqual(d.completion.currentBranch, "PHYSICALLY_UNRECOVERABLE");
testCase.verifyFalse(cap.completion.feasible);
testCase.verifyTrue(d.completion.upperActiveSetDefined);
testCase.verifyEqual([d.completion.upperActiveI, d.completion.upperActiveII], [true, false]);
end

function testSignedReservesAgreeWithFiniteExecutionCapabilitySlices(testCase)
[s, z] = fixture();
% Includes negative YO/empty joint slices, closed endpoints and outside states.
for opportunity = [1, 3, 4, 5]
    z.gR = 2*opportunity;
    z.gF = 2*opportunity;
    for position = [0, 0.25, 0.5, 1, 1.25, 3, 3.5, 3.75, 5]
        z.y = position;
        [d, cap] = diagnose(s, z);
        testCase.verifyTrue(cap.joint.finiteGeometryAvailable);
        testCase.verifyEqual(cap.abort.feasible, d.reserves.rO >= 0);
        testCase.verifyEqual(cap.completion.feasible, d.reserves.lT >= 0 && d.reserves.rT >= 0);
        testCase.verifyEqual(cap.joint.dualCapable, ...
            d.reserves.rO >= 0 && d.reserves.lT >= 0 && d.reserves.rT >= 0);
    end
end
end

function testSimulationCapabilityDiagnosticsPipelineAndImmutability(testCase)
[s, ~] = fixture();
s.vehicles.EGO.initialState = avlc.model.createVehicleState(0, 0, 10, 0, 0, 0);
s.vehicles.FO.initialState = avlc.model.createVehicleState(16, 0, 8, 0, 0, 0);
s.vehicles.RO.initialState = avlc.model.createVehicleState(-16, 0, 12, 0, 0, 0);
s.vehicles.RT.initialState = avlc.model.createVehicleState(-18.5, 4, 15, 0, 0, 0);
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.5, 1), ...
    avlc.model.createControl(0, 2));
z = avlc.analysis.buildTheoryState(s, result.states.EGO(3), result.states.FO(3), ...
    result.states.RO(3), result.states.RT(3));
cap = avlc.analysis.evaluateCapability(s, z);
cap.notes = "compatible capability metadata";
before = {s, z, cap, result};
d = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
testCase.verifyEqual([d.reserves.rO, d.reserves.lT, d.reserves.rT], [2.5, 0.5, 2.5]);
testCase.verifyEqual([d.abort.activeR, d.abort.activeF, d.abort.activeG], [false, false, true]);
testCase.verifyEqual({s, z, cap, result}, before);
testCase.verifyEqual(fieldnames(cap.abort), {'TRsafe'; 'TFsafe'; 'TG'; 'HO'; 'TOmin'; 'YO'; 'feasible'});
end

function testMalformedConsumedInputsFailExplicitly(testCase)
[s, z] = fixture();
cap = avlc.analysis.evaluateCapability(s, z);
identifier = 'AVLC:InvalidCapabilityDiagnosticsInput';
testCase.verifyError(@() avlc.analysis.evaluateCapabilityDiagnostics(NaN, cap), identifier);
testCase.verifyError(@() avlc.analysis.evaluateCapabilityDiagnostics(-1, cap), identifier);
testCase.verifyError(@() avlc.analysis.evaluateCapabilityDiagnostics(z.y, struct()), identifier);
changed = cap;
changed.completion = rmfield(changed.completion, 'Dminus');
testCase.verifyError(@() avlc.analysis.evaluateCapabilityDiagnostics(z.y, changed), identifier);
changed = cap;
changed.abort.YO = Inf;
testCase.verifyError(@() avlc.analysis.evaluateCapabilityDiagnostics(z.y, changed), identifier);
end

function testSignedReserveArithmeticWithIntegerLateralPosition(testCase)
[s, z] = fixture();
z.y = uint8(4);
cap = avlc.analysis.evaluateCapability(s, z);
d = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
testCase.verifyEqual([d.reserves.rO, d.reserves.lT, d.reserves.rT], [-0.5, 3.5, -0.5]);
testCase.verifyClass(d.reserves.rO, 'double');
testCase.verifyClass(d.reserves.lT, 'double');
testCase.verifyClass(d.reserves.rT, 'double');
end

function [diagnostics, cap] = diagnose(scenario, z)
cap = avlc.analysis.evaluateCapability(scenario, z);
diagnostics = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
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
scenario = avlc.scenario.createScenario("diagnostics fixture", "", road, params, vehicles{:});
z = struct('y', 1, 'w', 2, 'gR', 8, 'cR', 2, 'gF', 8, 'cF', 2, 'gT', 6, 'cT', 5);
end
