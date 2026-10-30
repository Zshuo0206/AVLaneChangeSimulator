function tests = testResearchVisualization
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testEveryHistoryIsAlignedWithAllNPlusOneStates(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(fieldnames(a), {'time'; 'z'; 'abort'; 'completion'; 'joint'; 'reserves'});
testCase.verifyEqual(a.time, result.time);
testCase.verifyEqual(numel(a.time), numel(result.controls.EGO)+1);
testCase.verifyEqual(fieldnames(a.z), {'y'; 'w'; 'gR'; 'cR'; 'gF'; 'cF'; 'gT'; 'cT'});
for group = {'z', 'abort', 'completion', 'joint', 'reserves'}
    for name = fieldnames(a.(group{1})).'
        testCase.verifySize(a.(group{1}).(name{1}), [5, 1]);
    end
end
testCase.verifyClass(a.abort.feasible, 'logical');
testCase.verifyClass(a.abort.activeR, 'logical');
testCase.verifyClass(a.completion.upperActiveII, 'logical');
testCase.verifyClass(a.completion.currentBranch, 'string');
end

function testSelectedSamplesExactlyEqualTheThreeFrozenCalls(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
for k = [1, 3, 5]
    z = avlc.analysis.buildTheoryState(s, result.states.EGO(k), result.states.FO(k), ...
        result.states.RO(k), result.states.RT(k));
    cap = avlc.analysis.evaluateCapability(s, z);
    d = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
    verifySample(testCase, a.z, z, k);
    verifySample(testCase, a.abort, cap.abort, k);
    verifySample(testCase, a.completion, cap.completion, k);
    verifySample(testCase, a.joint, cap.joint, k);
    verifySample(testCase, a.abort, d.abort, k);
    verifySample(testCase, a.completion, d.completion, k);
    verifySample(testCase, a.reserves, d.reserves, k);
end
end

function testInitialSamplePrecedesAnyControlStep(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(a.time(1), 0);
values = cellfun(@(name) a.z.(name)(1), fieldnames(a.z));
testCase.verifyEqual(values.', [0, 0, 10, 2, 10, 2, 11, 5]);
end

function testFinalSampleAndReservesHaveHandCalculatedValues(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
values = cellfun(@(name) a.z.(name)(end), fieldnames(a.z));
testCase.verifyEqual(a.time(end), 1);
testCase.verifyEqual(values.', [1, 2, 8, 2, 8, 2, 6, 5]);
testCase.verifyEqual([a.abort.HO(end), a.abort.YO(end), a.completion.HT(end)], [4, 3.5, 2]);
testCase.verifyEqual([a.reserves.rO(end), a.reserves.lT(end), a.reserves.rT(end)], [2.5, 0.5, 2.5]);
end

function testUndefinedTargetWindowsAreNotFilledAcrossTime(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(a.completion.HT, [Inf; Inf; Inf; Inf; 2]);
testCase.verifyTrue(all(isnan(a.completion.YTin(1:4))));
testCase.verifyTrue(all(isnan(a.completion.YTout(1:4))));
testCase.verifyTrue(all(isnan(a.reserves.lT(1:4))));
testCase.verifyTrue(all(isnan(a.reserves.rT(1:4))));
testCase.verifyEqual([a.completion.YTin(5), a.completion.YTout(5)], [0.5, 3.5]);
end

function testUnlimitedAbortOpportunityStaysInfWithNaNFrontiers(testCase)
[s, ~] = fixture();
s.vehicles.FO.initialState.vx = 10;
s.vehicles.RO.initialState.vx = 10;
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 1), avlc.model.createControl(0, 2));
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(a.abort.HO, repmat(Inf, 5, 1));
testCase.verifyTrue(all(isnan(a.abort.YO)));
testCase.verifyTrue(all(isnan(a.reserves.rO)));
testCase.verifyFalse(any(a.abort.activeSetDefined));
end

function testUnlimitedTargetOpportunityStaysInfWithNaNWindows(testCase)
[s, ~] = fixture();
s.vehicles.RT.initialState.vx = 10;
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 1), avlc.model.createControl(0, 2));
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(a.completion.HT, repmat(Inf, 5, 1));
testCase.verifyTrue(all(isnan(a.completion.YTin)));
testCase.verifyTrue(all(isnan(a.completion.YTout)));
testCase.verifyTrue(all(isnan(a.reserves.lT)));
testCase.verifyTrue(all(isnan(a.reserves.rT)));
end

function testActiveSetHistoriesRetainUniqueGapAndRearGapTie(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyTrue(all(a.abort.activeG));
testCase.verifyFalse(any(a.abort.activeR | a.abort.activeF));
s.vehicles.RO.initialState = avlc.model.createVehicleState(-10, 0, 15, 0, 0, 0);
s.vehicles.FO.initialState = avlc.model.createVehicleState(8, 0, 9, 0, 0, 0);
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.125, 0.25), avlc.model.createControl(0, 0));
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual([a.abort.activeR(1), a.abort.activeF(1), a.abort.activeG(1)], [true, false, true]);
end

function testCompletionUpperMechanismHistoriesRemainSeparate(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual([a.completion.upperActiveI(end), a.completion.upperActiveII(end)], [true, false]);
s.theoryParameters.targetOvershoot = 0.25;
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 1), avlc.model.createControl(0, 2));
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual([a.completion.upperActiveI(end), a.completion.upperActiveII(end)], [false, true]);
testCase.verifyEqual(a.completion.currentBranch(end), "NON_OVERSHOOT");
end

function testResultIndexingMismatchesFailExplicitly(testCase)
[s, result] = fixture();
bad = result;
bad.states.RT(end) = [];
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, bad), 'AVLC:InvalidTrajectoryResult');
bad = result;
bad.controls.EGO(end) = [];
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, bad), 'AVLC:InvalidTrajectoryResult');
bad = result;
bad.metadata.numSteps = 5;
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, bad), 'AVLC:InvalidTrajectoryResult');
bad = result;
bad.time(3) = bad.time(2);
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, bad), 'AVLC:InvalidTrajectoryResult');
end

function testUnsupportedTheoryDomainIsNotSkipped(testCase)
[s, ~] = fixture();
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 1), avlc.model.createControl(0, -1));
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, result), 'AVLC:CapabilityDomainViolation');
end

function testInvalidPhysicalSampleErrorPropagates(testCase)
[s, result] = fixture();
result.states.FO(3).vx = NaN;
testCase.verifyError(@() avlc.analysis.analyzeTrajectory(s, result), 'AVLC:InvalidVehicleState');
end

function testAdapterCreatesNoFiguresAndLeavesInputsUnchanged(testCase)
[s, result] = fixture();
before = {s, result};
figures = findall(groot, 'Type', 'figure');
avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyEqual(findall(groot, 'Type', 'figure'), figures);
testCase.verifyEqual({s, result}, before);
end

function testPlotCreatesFourPanelsAndPhysicalFinalRectangles(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
fig = avlc.visualization.plotResearchSummary(s, result, a, Visible="off");
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
testCase.verifyTrue(isgraphics(fig, 'figure'));
testCase.verifyNumElements(findall(fig, 'Type', 'axes'), 4);
original = findall(fig, 'Tag', 'lane-original');
target = findall(fig, 'Tag', 'lane-target');
testCase.verifyEqual([min(original.XData), max(original.XData)], [-2, 2]);
testCase.verifyEqual([min(target.XData), max(target.XData)], [2, 6]);
for role = {'EGO','FO','RO','RT'}
    box = findall(fig, 'Tag', ['vehicle-' role{1}]);
    testCase.verifyNumElements(box, 1);
    testCase.verifyEqual(mean(box.XData), result.states.(role{1})(end).y, 'AbsTol', 1e-14);
    testCase.verifyEqual(mean(box.YData), result.states.(role{1})(end).x, 'AbsTol', 1e-14);
    testCase.verifyEqual(max(box.XData)-min(box.XData), s.vehicles.(role{1}).geometry.width, 'AbsTol', 1e-14);
    testCase.verifyEqual(max(box.YData)-min(box.YData), s.vehicles.(role{1}).geometry.length, 'AbsTol', 1e-14);
    path = findobj(fig, 'Tag', ['trajectory-' role{1}]);
    testCase.verifyEqual(path.XData(:), [result.states.(role{1}).y].');
    testCase.verifyEqual(path.YData(:), [result.states.(role{1}).x].');
end
end

function testPlotUsesRawBoundariesReservesAndBooleanSamples(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
before = {s, result, a};
fig = avlc.visualization.plotResearchSummary(s, result, a, Visible="off");
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
tags = {'boundary-y','boundary-YO','boundary-YTin','boundary-YTout','reserve-rO','reserve-lT','reserve-rT'};
expected = {a.z.y,a.abort.YO,a.completion.YTin,a.completion.YTout,a.reserves.rO,a.reserves.lT,a.reserves.rT};
for index = 1:numel(tags)
    line = findobj(fig, 'Tag', tags{index});
    testCase.verifyEqual(line.XData(:), a.time);
    testCase.verifyEqual(line.YData(:), expected{index});
end
line = findobj(fig, 'Tag', 'capability-abort');
testCase.verifyEqual(line.YData(:), 2.4+0.65*double(a.abort.feasible));
line = findobj(fig, 'Tag', 'capability-completion');
testCase.verifyEqual(line.YData(:), 1.2+0.65*double(a.completion.feasible));
line = findobj(fig, 'Tag', 'capability-dual');
testCase.verifyEqual(line.YData(:), 0.65*double(a.joint.dualCapable));
testCase.verifyEqual({s, result, a}, before);
end

function testPlotRetainsNegativeReservesAndGaps(testCase)
[s, ~] = fixture();
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 2), avlc.model.createControl(0, 2));
a = avlc.analysis.analyzeTrajectory(s, result);
testCase.verifyTrue(any(a.reserves.rO < 0));
testCase.verifyTrue(any(isnan(a.completion.YTin)));
fig = avlc.visualization.plotResearchSummary(s, result, a, Visible="off");
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
line = findobj(fig, 'Tag', 'reserve-rO');
testCase.verifyEqual(line.YData(:), a.reserves.rO);
line = findobj(fig, 'Tag', 'boundary-YTin');
testCase.verifyEqual(isnan(line.YData(:)), isnan(a.completion.YTin));
end

function testPlotRejectsMisalignedAnalysisBeforeOpeningFigure(testCase)
[s, result] = fixture();
a = avlc.analysis.analyzeTrajectory(s, result);
a.time(end) = a.time(end)+0.1;
figures = findall(groot, 'Type', 'figure');
testCase.verifyError(@() avlc.visualization.plotResearchSummary(s, result, a, Visible="off"), ...
    'AVLC:InvalidResearchSummaryInput');
testCase.verifyEqual(findall(groot, 'Type', 'figure'), figures);
end

function verifySample(testCase, history, sample, k)
for name = fieldnames(sample).'
    testCase.verifyEqual(history.(name{1})(k), sample.(name{1}));
end
end

function [s, result] = fixture()
road = avlc.model.createRoad(4, [-100, 100]);
params = avlc.model.createTheoryModelParameters(2, 4, 2, 1, 1, 1.5, 2);
roles = {'EGO', 'FO', 'RO', 'RT'};
lengths = [4, 6, 5, 7];
states = {avlc.model.createVehicleState(0, 0, 10, 0, 0, 0), ...
    avlc.model.createVehicleState(16, 0, 8, 0, 0, 0), ...
    avlc.model.createVehicleState(-16, 0, 12, 0, 0, 0), ...
    avlc.model.createVehicleState(-18.5, 4, 15, 0, 0, 0)};
vehicles = cell(1, 4);
for index = 1:4
    vehicles{index} = avlc.model.createVehicle(string(index), roles{index}, lengths(index), 2, states{index});
end
s = avlc.scenario.createScenario("visualization test", "", road, params, vehicles{:});
result = avlc.core.runSimulation(s, avlc.model.createSimulationConfig(0.25, 1), avlc.model.createControl(0, 2));
end
