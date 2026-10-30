function result = searchLongitudinalSafePrefixWitness(scenario,z0,T,options)
%SEARCHLONGITUDINALSAFEPREFIXWITNESS Search one finite uniform control class.
%   R = avlc.analysis.searchLongitudinalSafePrefixWitness(SCENARIO,Z0,T,OPTIONS)
%   consumes only B/C and gR,cR,gF,cF,gT,cT. OPTIONS has numSegments (default
%   8, positive integer), maxCutIterations (default 20, nonnegative integer).
%   LINPROG proposes candidates with zero objective. Every usable candidate
%   is sent unchanged to the frozen verifyLongitudinalSafePrefixWitness.
%   Only its valid certificate permits WITNESS_FOUND. Other outcomes are
%   UNRESOLVED, never a continuous-control infeasibility statement.
%   Initial cuts are partition nodes; failed certificates supply additional
%   exact minimum times. No clipping, tolerance merging, moment objective,
%   adaptive N, simulation or lateral/capability dependency is used.
%   T=0 calls only the verifier, without LINPROG/OPTIMOPTIONS. Missing solver
%   or numerical failures return UNRESOLVED with diagnostics. Malformed
%   consumed inputs raise AVLC:InvalidLongitudinalWitnessSearchInput.
%   Inputs are unchanged. Source: Master Part VIII sections 111-112,125;
%   certificate authority: frozen P0.3.2a verifier.

if nargin < 4
    options = struct();
end
[B,C,state,T,N,cutLimit] = validateInputs(scenario,z0,T,options);
available = solverAvailable();
search = struct('numSegments',N,'maxCutIterations',cutLimit, ...
    'solverAvailable',available,'partitionTimes',zeros(0,1),'cutTimes',zeros(0,1), ...
    'solveCount',0,'verifiedCandidateCount',0,'cutIterationCount',0, ...
    'solverReportedSearchClassInfeasible',false,'finalExitflag',NaN, ...
    'finalSolverOutput',struct(),'exceptionIdentifier',"",'exceptionMessage',"");
result = struct('status',"UNRESOLVED",'witnessFound',false,'reason',"", ...
    'witness',struct(),'certificate',struct(),'search',search, ...
    'attempts',repmat(emptyAttempt(),0,1));

if T == 0
    result.search.partitionTimes = 0;
    result.search.cutTimes = 0;
    attempt = emptyAttempt();
    attempt.index = 1;
    attempt.source = "ZERO_HORIZON";
    witness = struct('times',0,'ax',zeros(0,1));
    [attempt,certificate,arithmeticFailure] = verifyAttempt(attempt,scenario,z0,T,witness);
    result.attempts = attempt;
    if arithmeticFailure
        result = verifierFailure(result,attempt);
        return;
    end
    result.search.verifiedCandidateCount = 1;
    if certificate.validWitness
        result = foundWitness(result,certificate);
    else
        result.reason = "INITIAL_STATE_UNSAFE";
    end
    return;
end

partition = linspace(0,T,N+1).';
result.search.partitionTimes = partition;
result.search.cutTimes = partition;
if any(~isfinite(partition)) || any(diff(partition) <= 0)
    result.reason = "PARTITION_NOT_REPRESENTABLE";
    return;
end
if ~available
    result.reason = "SOLVER_UNAVAILABLE";
    return;
end
try
    solverOptions = optimoptions('linprog','Display','none');
catch exception
    result = solverFailure(result,exception);
    return;
end

while true
    [A,b] = cutConstraints(partition,result.search.cutTimes,state);
    if any(~isfinite(A(:))) || any(~isfinite(b))
        result.reason = "SEARCH_ARITHMETIC_FAILURE";
        return;
    end
    attempt = emptyAttempt();
    attempt.index = result.search.solveCount+1;
    result.search.solveCount = result.search.solveCount+1;
    try
        [candidate,~,exitflag,output] = linprog(zeros(N,1),A,b,[],[], ...
            -B*ones(N,1),C*ones(N,1),solverOptions);
    catch exception
        attempt.exceptionIdentifier = string(exception.identifier);
        attempt.exceptionMessage = string(exception.message);
        result.attempts(end+1,1) = attempt;
        result = solverFailure(result,exception);
        return;
    end
    attempt.exitflag = exitflag;
    attempt.solverOutput = output;
    attempt.ax = candidate;
    result.search.finalExitflag = exitflag;
    result.search.finalSolverOutput = output;
    result.search.solverReportedSearchClassInfeasible = ...
        result.search.solverReportedSearchClassInfeasible || reportsInfeasible(output);
    positiveSuccess = isnumeric(exitflag) && isreal(exitflag) && ...
        isscalar(exitflag) && isfinite(exitflag) && exitflag > 0;
    usable = isnumeric(candidate) && isreal(candidate) && isvector(candidate) && ...
        numel(candidate) == N && all(isfinite(candidate(:)));
    if ~positiveSuccess || ~usable
        result.attempts(end+1,1) = attempt;
        result.reason = "SOLVER_NO_USABLE_CANDIDATE";
        return;
    end

    witness = struct('times',partition,'ax',candidate);
    [attempt,certificate,arithmeticFailure] = verifyAttempt(attempt,scenario,z0,T,witness);
    if arithmeticFailure
        result.attempts(end+1,1) = attempt;
        result = verifierFailure(result,attempt);
        return;
    end
    result.search.verifiedCandidateCount = result.search.verifiedCandidateCount+1;
    if certificate.validWitness
        result.attempts(end+1,1) = attempt;
        result = foundWitness(result,certificate);
        return;
    end
    if ~certificate.control.withinBounds
        result.attempts(end+1,1) = attempt;
        result.reason = "SOLVER_CANDIDATE_OUT_OF_BOUNDS";
        return;
    end

    minima = [certificate.gaps.minimumR;certificate.gaps.minimumF;certificate.gaps.minimumT];
    times = [certificate.gaps.timeOfMinimumR;certificate.gaps.timeOfMinimumF; ...
        certificate.gaps.timeOfMinimumT];
    attempt.violationTimes = unique(times(minima < 0));
    alreadyPresent = ismember(attempt.violationTimes,result.search.cutTimes);
    if any(alreadyPresent) || isempty(attempt.violationTimes)
        result.attempts(end+1,1) = attempt;
        result.reason = "SOLVER_CONSTRAINT_MISMATCH";
        return;
    end
    if result.search.cutIterationCount >= cutLimit
        result.attempts(end+1,1) = attempt;
        result.reason = "CUT_LIMIT_REACHED";
        return;
    end
    attempt.newCutTimes = attempt.violationTimes;
    result.search.cutTimes = unique([result.search.cutTimes;attempt.newCutTimes]);
    result.search.cutIterationCount = result.search.cutIterationCount+1;
    result.attempts(end+1,1) = attempt;
end
end

function [A,b] = cutConstraints(partition,cutTimes,z)
N = numel(partition)-1;
A = zeros(3*numel(cutTimes),N);
b = zeros(3*numel(cutTimes),1);
for k = 1:numel(cutTimes)
    t = cutTimes(k);
    q = zeros(1,N);
    for j = 1:N
        s = partition(j);
        e = partition(j+1);
        if t <= s
            q(j) = 0;
        elseif t < e
            q(j) = (t-s)^2/2;
        else
            h = e-s;
            q(j) = h*(t-s-h/2);
        end
    end
    rows = 3*k-2:3*k;
    A(rows,:) = [-q;-q;q];
    b(rows) = [z(1)-z(2)*t;z(5)-z(6)*t;z(3)-z(4)*t];
end
end

function attempt = emptyAttempt()
attempt = struct('index',0,'source',"LINPROG",'exitflag',NaN, ...
    'solverOutput',struct(),'ax',zeros(0,1),'verifierCalled',false, ...
    'certificateStatus',"",'validWitness',false,'minimumR',NaN,'minimumF',NaN, ...
    'minimumT',NaN,'violationTimes',zeros(0,1),'newCutTimes',zeros(0,1), ...
    'certificate',struct(),'exceptionIdentifier',"",'exceptionMessage',"");
end

function [attempt,certificate,arithmeticFailure] = verifyAttempt(attempt,scenario,z0,T,witness)
attempt.verifierCalled = true;
certificate = struct();
arithmeticFailure = false;
try
    certificate = avlc.analysis.verifyLongitudinalSafePrefixWitness(scenario,z0,T,witness);
catch exception
    if ~strcmp(exception.identifier,'AVLC:LongitudinalWitnessArithmeticFailure')
        rethrow(exception);
    end
    arithmeticFailure = true;
    attempt.exceptionIdentifier = string(exception.identifier);
    attempt.exceptionMessage = string(exception.message);
    return;
end
attempt.certificate = certificate;
attempt.certificateStatus = certificate.status;
attempt.validWitness = certificate.validWitness;
attempt.minimumR = certificate.gaps.minimumR;
attempt.minimumF = certificate.gaps.minimumF;
attempt.minimumT = certificate.gaps.minimumT;
end

function result = foundWitness(result,certificate)
result.status = "WITNESS_FOUND";
result.witnessFound = true;
result.reason = "CERTIFIED_WITNESS";
result.witness = struct('times',certificate.control.times,'ax',certificate.control.ax);
result.certificate = certificate;
end

function result = verifierFailure(result,attempt)
result.reason = "VERIFIER_ARITHMETIC_FAILURE";
result.search.exceptionIdentifier = attempt.exceptionIdentifier;
result.search.exceptionMessage = attempt.exceptionMessage;
end

function result = solverFailure(result,exception)
result.reason = "SOLVER_FAILURE";
identifier = lower(string(exception.identifier));
if contains(identifier,'undefinedfunction') || contains(identifier,'license')
    result.reason = "SOLVER_UNAVAILABLE";
    result.search.solverAvailable = false;
end
result.search.exceptionIdentifier = string(exception.identifier);
result.search.exceptionMessage = string(exception.message);
end

function available = solverAvailable()
available = ~isempty(which('linprog')) && ~isempty(which('optimoptions'));
if available
    try
        available = logical(license('test','Optimization_Toolbox'));
    catch
        available = false;
    end
end
end

function reported = reportsInfeasible(output)
% Best-effort message diagnostic only; no exitflag-number mapping or truth claim.
reported = false;
if isstruct(output) && isscalar(output) && isfield(output,'message') && ...
        (ischar(output.message) || (isstring(output.message) && isscalar(output.message)))
    reported = any(contains(lower(string(output.message)), ...
        ["infeasible","no feasible","no point satisfies","不可行"]));
end
end

function [B,C,state,T,N,limit] = validateInputs(scenario,z,T,options)
identifier = 'AVLC:InvalidLongitudinalWitnessSearchInput';
requireFields(scenario,{'theoryParameters'},'scenario',identifier);
requireFields(scenario.theoryParameters,{'maxLongitudinalBrake', ...
    'maxLongitudinalAccel'},'theoryParameters',identifier);
names = {'gR','cR','gF','cF','gT','cT'};
requireFields(z,names,'z0',identifier);
requireFields(options,{},'options',identifier);
if ~all(ismember(fieldnames(options),{'numSegments','maxCutIterations'}))
    error(identifier,'Only numSegments and maxCutIterations are supported options.');
end
B = scalarValue(scenario.theoryParameters.maxLongitudinalBrake,'B',identifier);
C = scalarValue(scenario.theoryParameters.maxLongitudinalAccel,'C',identifier);
T = scalarValue(T,'T',identifier);
if B <= 0 || C <= 0 || T < 0
    error(identifier,'Require B>0, C>0 and T>=0.');
end
state = zeros(6,1);
for k = 1:6
    state(k) = scalarValue(z.(names{k}),names{k},identifier);
end
N = 8;
limit = 20;
if isfield(options,'numSegments')
    N = scalarValue(options.numSegments,'numSegments',identifier);
end
if isfield(options,'maxCutIterations')
    limit = scalarValue(options.maxCutIterations,'maxCutIterations',identifier);
end
if N < 1 || N ~= fix(N) || limit < 0 || limit ~= fix(limit)
    error(identifier,'numSegments must be a positive integer; maxCutIterations a nonnegative integer.');
end
end

function requireFields(value,fields,name,identifier)
if ~isstruct(value) || ~isscalar(value) || (~isempty(fields) && ~all(isfield(value,fields)))
    error(identifier,'%s must be a scalar struct containing all consumed fields.',name);
end
end

function value = scalarValue(value,name,identifier)
if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
    error(identifier,'%s must be a finite real numeric scalar.',name);
end
value = double(value);
end
