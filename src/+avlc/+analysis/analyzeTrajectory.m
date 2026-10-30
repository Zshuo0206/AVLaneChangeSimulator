function analysis = analyzeTrajectory(scenario, result)
%ANALYZETRAJECTORY Collect the frozen single-state analyses at every sample.
%   ANALYSIS = avlc.analysis.analyzeTrajectory(SCENARIO, RESULT) calls
%   buildTheoryState, evaluateCapability and evaluateCapabilityDiagnostics
%   once per state sample, including initial and final states. All histories
%   are M-by-1, M=N+1, aligned with RESULT.time. Groups are time, z, abort,
%   completion, joint, reserves. Capability and diagnostic fields are copied
%   unchanged; abort/completion include their respective diagnostic flags.
%   No theory formula, interpolation, sentinel replacement or figure is
%   introduced. Inputs are unchanged. SCENARIO must match the simulation.
%   Invalid result indexing raises AVLC:InvalidTrajectoryResult. Existing
%   state/theory validation errors propagate, including the pre-decision
%   domain restriction y>=0,w>=0: unsupported samples are never skipped.

avlc.scenario.validateScenario(scenario);
validateResultIndexing(result);
sampleCount = numel(result.time);
analysis.time = result.time;
groupNames = {'z', 'abort', 'completion', 'joint', 'reserves'};
for k = 1:sampleCount
    z = avlc.analysis.buildTheoryState(scenario, result.states.EGO(k), ...
        result.states.FO(k), result.states.RO(k), result.states.RT(k));
    cap = avlc.analysis.evaluateCapability(scenario, z);
    diagnostics = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
    abort = cap.abort;
    abort.activeSetDefined = diagnostics.abort.activeSetDefined;
    abort.activeR = diagnostics.abort.activeR;
    abort.activeF = diagnostics.abort.activeF;
    abort.activeG = diagnostics.abort.activeG;
    completion = cap.completion;
    completion.currentBranch = diagnostics.completion.currentBranch;
    completion.upperActiveSetDefined = diagnostics.completion.upperActiveSetDefined;
    completion.upperActiveI = diagnostics.completion.upperActiveI;
    completion.upperActiveII = diagnostics.completion.upperActiveII;
    samples = {z, abort, completion, cap.joint, diagnostics.reserves};
    for groupIndex = 1:numel(groupNames)
        group = groupNames{groupIndex};
        sample = samples{groupIndex};
        fields = fieldnames(sample);
        for fieldIndex = 1:numel(fields)
            name = fields{fieldIndex};
            if k == 1
                analysis.(group).(name) = repmat(sample.(name), sampleCount, 1);
            else
                analysis.(group).(name)(k, 1) = sample.(name);
            end
        end
    end
end
end

function validateResultIndexing(result)
identifier = 'AVLC:InvalidTrajectoryResult';
required = {'time', 'states', 'controls', 'metadata'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, required))
    error(identifier, 'Supply a SimulationResult with time, states, controls and metadata.');
end
time = result.time;
if ~(isa(time, 'double') && isreal(time) && iscolumn(time) && numel(time) >= 2 && ...
        all(isfinite(time)) && time(1) == 0 && all(diff(time) > 0))
    error(identifier, 'Result.time must be a finite increasing double column starting at zero.');
end
roles = {'EGO', 'FO', 'RO', 'RT'};
if ~isstruct(result.states) || ~isscalar(result.states) || ~all(isfield(result.states, roles))
    error(identifier, 'Result.states must contain the four frozen roles.');
end
for index = 1:numel(roles)
    states = result.states.(roles{index});
    if ~isstruct(states) || ~iscolumn(states) || numel(states) ~= numel(time)
        error(identifier, 'Each vehicle history must have exactly one state per time sample.');
    end
end
if ~isstruct(result.metadata) || ~isscalar(result.metadata) || ~isfield(result.metadata, 'numSteps')
    error(identifier, 'Result.metadata.numSteps is required to verify N+1 indexing.');
end
numSteps = result.metadata.numSteps;
if ~(isnumeric(numSteps) && isreal(numSteps) && isscalar(numSteps) && isfinite(numSteps) && ...
        numSteps >= 1 && double(numSteps) == numel(time)-1)
    error(identifier, 'Result.metadata.numSteps must equal the number of state intervals.');
end
if ~isstruct(result.controls) || ~isscalar(result.controls) || ~isfield(result.controls, 'EGO') || ...
        ~isstruct(result.controls.EGO) || ~iscolumn(result.controls.EGO) || ...
        numel(result.controls.EGO) ~= double(numSteps)
    error(identifier, 'Result must have N EGO controls aligned with N+1 states.');
end
end
