function fig = plotResearchSummary(scenario, result, analysis, options)
%PLOTRESEARCHSUMMARY Display physical motion and existing theory histories.
%   FIG = avlc.visualization.plotResearchSummary(SCENARIO, RESULT, ANALYSIS)
%   creates a static ordinary MATLAB figure. The left panel shows physical
%   trajectories and final vehicle rectangles, with lateral y horizontal and
%   longitudinal x vertical (meters). Right panels show sampled execution
%   capability, raw lateral boundaries and signed reserves on linked time axes.
%   Final active sets retain ties. No analysis, smoothing, NaN interpolation,
%   reserve clipping or capability reinterpretation occurs here.
%   Use Visible="off" for noninteractive rendering. Inputs are not modified.
%   RESULT/ANALYSIS must be the matching outputs for this SCENARIO.
arguments
    scenario (1,1) struct
    result (1,1) struct
    analysis (1,1) struct
    options.Visible (1,1) string {mustBeMember(options.Visible, ["on", "off"])} = "on"
end
avlc.scenario.validateScenario(scenario);
validateHistories(result, analysis);
fig = figure('Name', 'AVLC Research Visualization V0', 'NumberTitle', 'off', ...
    'Color', 'white', 'Position', [80, 60, 1450, 900], 'Visible', options.Visible);
layout = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
blue = [0.10, 0.36, 0.68];
orange = [0.83, 0.36, 0.08];
purple = [0.48, 0.30, 0.65];
colors = [blue; orange; 0.29,0.50,0.35; purple];
t = analysis.time;
k = numel(t);
road = nexttile(layout, 1, [3, 1]);
set(road, 'Tag', 'research-road');
drawRoad(road, scenario, result, colors);
title(road, {'Vehicle motion', sprintf('Final sample: t = %.2f s', t(k))});
subtitle(road, 'circles: initial; boxes: final', 'FontSize', 10);

status = nexttile(layout, 2, [1, 2]);
set(status, 'Tag', 'research-capability');
hold(status, 'on');
stairs(status, t, 2.4 + 0.65*double(analysis.abort.feasible), ...
    'Color', blue, 'LineWidth', 2, 'Tag', 'capability-abort');
stairs(status, t, 1.2 + 0.65*double(analysis.completion.feasible), ...
    'Color', orange, 'LineWidth', 2, 'Tag', 'capability-completion');
stairs(status, t, 0.65*double(analysis.joint.dualCapable), ...
    'Color', purple, 'LineWidth', 2, 'Tag', 'capability-dual');
yticks(status, [0, 0.65, 1.2, 1.85, 2.4, 3.05]);
yticklabels(status, {'dual 0', 'dual 1', 'complete 0', 'complete 1', 'abort 0', 'abort 1'});
ylim(status, [-0.2, 3.3]);
title(status, 'Current execution capability (separate 0 / 1 rows)');
grid(status, 'on');

boundaries = nexttile(layout, 5, [1, 2]);
set(boundaries, 'Tag', 'research-boundaries');
hold(boundaries, 'on');
plot(boundaries, t, analysis.z.y, '-', 'Color', [0.13,0.15,0.19], ...
    'LineWidth', 2.2, 'DisplayName', 'y', 'Tag', 'boundary-y');
plot(boundaries, t, analysis.abort.YO, '--', 'Color', blue, ...
    'LineWidth', 1.7, 'DisplayName', 'Y_O', 'Tag', 'boundary-YO');
plot(boundaries, t, analysis.completion.YTin, '-.', 'Color', orange, ...
    'LineWidth', 1.7, 'DisplayName', 'Y_T^{in}', 'Tag', 'boundary-YTin');
plot(boundaries, t, analysis.completion.YTout, '--', 'Color', purple, ...
    'LineWidth', 1.7, 'DisplayName', 'Y_T^{out}', 'Tag', 'boundary-YTout');
ylabel(boundaries, 'Lateral position [m]');
title(boundaries, 'Lateral state and finite capability boundaries');
legend(boundaries, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
grid(boundaries, 'on');

reserves = nexttile(layout, 8, [1, 2]);
set(reserves, 'Tag', 'research-reserves');
hold(reserves, 'on');
plot(reserves, t, analysis.reserves.rO, '-', 'Color', blue, ...
    'LineWidth', 1.8, 'DisplayName', '$r_O$', 'Tag', 'reserve-rO');
plot(reserves, t, analysis.reserves.lT, '-.', 'Color', orange, ...
    'LineWidth', 1.8, 'DisplayName', '$\ell_T$', 'Tag', 'reserve-lT');
plot(reserves, t, analysis.reserves.rT, '--', 'Color', purple, ...
    'LineWidth', 1.8, 'DisplayName', '$r_T$', 'Tag', 'reserve-rT');
yline(reserves, 0, ':', 'Color', [0.35,0.35,0.35], 'HandleVisibility', 'off');
ylabel(reserves, 'Signed reserve [m]');
xlabel(reserves, 'Time [s]');
title(reserves, 'Signed reserves (zero = corresponding boundary)');
legend(reserves, 'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off', 'Interpreter', 'latex');
grid(reserves, 'on');
linkaxes([status, boundaries, reserves], 'x');
xlim(status, [t(1), t(end)]);
set([status, boundaries], 'XTickLabel', []);
axesList = [road, status, boundaries, reserves];
set(axesList, 'FontName', 'Arial', 'FontSize', 11, 'Box', 'off', 'TickDir', 'out');
abortSet = activeText([analysis.abort.activeR(k), analysis.abort.activeF(k), analysis.abort.activeG(k)], ...
    ["R", "F", "G"], analysis.abort.activeSetDefined(k));
upperSet = activeText([analysis.completion.upperActiveI(k), analysis.completion.upperActiveII(k)], ...
    ["I", "II"], analysis.completion.upperActiveSetDefined(k));
title(layout, "Research Visualization V0 | " + string(reshape(scenario.name, 1, [])), ...
    'FontSize', 17, 'FontWeight', 'bold', 'Interpreter', 'none');
subtitle(layout, sprintf('Final finite mechanisms: abort %s | completion upper %s | current time case: %s', ...
    abortSet, upperSet, lower(strrep(analysis.completion.currentBranch(k), '_', ' '))), ...
    'FontSize', 11, 'Interpreter', 'none');
end

function drawRoad(ax, scenario, result, colors)
roles = {'EGO', 'FO', 'RO', 'RT'};
longitudinal = [];
lateral = [];
lengths = zeros(1,4);
widths = zeros(1,4);
for index = 1:4
    states = result.states.(roles{index});
    longitudinal = [longitudinal, double([states.x])]; %#ok<AGROW>
    lateral = [lateral, double([states.y])]; %#ok<AGROW>
    lengths(index) = double(scenario.vehicles.(roles{index}).geometry.length);
    widths(index) = double(scenario.vehicles.(roles{index}).geometry.width);
end
laneWidth = double(scenario.road.laneWidth);
original = double(scenario.road.originalLaneCenterY);
target = double(scenario.road.targetLaneCenterY);
longLimits = [min(longitudinal)-max(lengths)/2-3, max(longitudinal)+max(lengths)/2+3];
hold(ax, 'on');
patch(ax, original+laneWidth*[-0.5,0.5,0.5,-0.5], longLimits([1,1,2,2]), ...
    [0.96,0.97,0.98], 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Tag', 'lane-original');
patch(ax, target+laneWidth*[-0.5,0.5,0.5,-0.5], longLimits([1,1,2,2]), ...
    [0.92,0.95,0.98], 'EdgeColor', 'none', 'HandleVisibility', 'off', 'Tag', 'lane-target');
text(ax, original-laneWidth/2+0.25, mean(longLimits), 'Original lane', ...
    'Rotation', 90, 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.4,0.44,0.48]);
text(ax, target+laneWidth/2-0.25, mean(longLimits), 'Target lane', ...
    'Rotation', 90, 'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.4,0.44,0.48]);
for edge = [original-laneWidth/2, original+laneWidth/2, target+laneWidth/2]
    xline(ax, edge, '--', 'Color', [0.65,0.68,0.72], 'HandleVisibility', 'off');
end
for center = [original, target]
    xline(ax, center, ':', 'Color', [0.75,0.78,0.81], 'HandleVisibility', 'off');
end
paths = gobjects(1,4);
for index = 1:4
    role = roles{index};
    states = result.states.(role);
    paths(index) = plot(ax, [states.y], [states.x], '-', 'Color', colors(index,:), ...
        'LineWidth', 1.4, 'DisplayName', role, 'Tag', ['trajectory-' role]);
    plot(ax, states(1).y, states(1).x, 'o', 'Color', colors(index,:), ...
        'MarkerFaceColor', 'white', 'MarkerSize', 5, 'HandleVisibility', 'off', 'Tag', ['initial-' role]);
    left = double(states(end).y)-widths(index)/2;
    bottom = double(states(end).x)-lengths(index)/2;
    patch(ax, left+widths(index)*[0,1,1,0], bottom+lengths(index)*[0,0,1,1], ...
        colors(index,:), 'FaceAlpha', 0.75, 'EdgeColor', colors(index,:)*0.7, ...
        'HandleVisibility', 'off', 'Tag', ['vehicle-' role]);
    text(ax, left+widths(index)+0.25, states(end).x, role, 'Color', colors(index,:), ...
        'FontWeight', 'bold', 'FontSize', 10, 'Interpreter', 'none');
end
axis(ax, 'equal');
xlim(ax, [min([lateral-max(widths)/2, original-laneWidth/2])-1, ...
    max([lateral+max(widths)/2, target+laneWidth/2])+3]);
ylim(ax, longLimits);
xticks(ax, [original, target]);
xlabel(ax, 'Lateral y [m]');
ylabel(ax, 'Longitudinal x [m]');
legend(ax, paths, 'Location', 'southoutside', 'NumColumns', 2, 'Box', 'off');
end

function label = activeText(mask, names, defined)
if defined
    label = strjoin(names(mask), '+');
else
    label = "undefined";
end
end

function validateHistories(result, analysis)
identifier = 'AVLC:InvalidResearchSummaryInput';
if ~isfield(result, 'time') || ~isfield(result, 'states') || ...
        ~isfield(analysis, 'time') || ~isequal(result.time, analysis.time) || ...
        ~iscolumn(analysis.time) || numel(analysis.time) < 2
    error(identifier, 'Result and analysis must share the same N+1 time column.');
end
groups = {'z', 'abort', 'completion', 'joint', 'reserves'};
required = {{'y'}, {'feasible','YO','activeSetDefined','activeR','activeF','activeG'}, ...
    {'feasible','YTin','YTout','currentBranch','upperActiveSetDefined','upperActiveI','upperActiveII'}, ...
    {'dualCapable'}, {'rO','lT','rT'}};
for index = 1:numel(groups)
    if ~isfield(analysis, groups{index}) || ~isstruct(analysis.(groups{index})) || ...
            ~isscalar(analysis.(groups{index})) || ~all(isfield(analysis.(groups{index}), required{index}))
        error(identifier, 'Analysis is missing required %s histories.', groups{index});
    end
    for name = required{index}
        values = analysis.(groups{index}).(name{1});
        if ~iscolumn(values) || numel(values) ~= numel(analysis.time)
            error(identifier, 'Every displayed history must align with the time column.');
        end
    end
end
for role = {'EGO','FO','RO','RT'}
    if ~isfield(result.states, role{1}) || numel(result.states.(role{1})) ~= numel(analysis.time)
        error(identifier, 'Four matching physical state histories are required.');
    end
end
end
