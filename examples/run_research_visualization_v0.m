%RUN_RESEARCH_VISUALIZATION_V0 One-command static research figure.
%   Run this script from any directory. It leaves researchScenario,
%   researchResult, researchAnalysis and researchFigure in the workspace.
%   This constant-control trial illustrates sampled reduced capability;
%   no controller or shared-option-preservation conclusion is introduced.

[researchScenario, researchResult, researchAnalysis, researchFigure] = ...
    runResearchVisualizationV0(fileparts(fileparts(mfilename('fullpath'))));

function [scenario, result, analysis, fig] = runResearchVisualizationV0(projectRoot)
originalPath = path;
restorePath = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(fullfile(projectRoot, 'src'));
road = avlc.model.createRoad(3.6, [-25, 65]);
parameters = avlc.model.createTheoryModelParameters(2, 4, 2, 0.5, 2, 2, 2.5);
roles = {'EGO', 'FO', 'RO', 'RT'};
lengths = [4.6, 5.0, 4.8, 5.2];
widths = [1.9, 2.0, 1.9, 2.0];
states = {avlc.model.createVehicleState(0, 0, 10, 0, 0, 0), ...
    avlc.model.createVehicleState(15.8, 0, 8, 0, 0, 0), ...
    avlc.model.createVehicleState(-15.7, 0, 12, 0, 0, 0), ...
    avlc.model.createVehicleState(-13.4, 3.6, 14, 0, 0, 0)};
vehicles = cell(1, 4);
for index = 1:4
    vehicles{index} = avlc.model.createVehicle(string(index), roles{index}, ...
        lengths(index), widths(index), states{index});
end
scenario = avlc.scenario.createScenario("Four-vehicle constant-control trial", ...
    "Sampled reduced execution capability; no shared-control inference.", road, parameters, vehicles{:});
result = avlc.core.runSimulation(scenario, avlc.model.createSimulationConfig(0.05, 4), ...
    avlc.model.createControl(0, 0.5));
analysis = avlc.analysis.analyzeTrajectory(scenario, result);
fig = avlc.visualization.plotResearchSummary(scenario, result, analysis);
end
