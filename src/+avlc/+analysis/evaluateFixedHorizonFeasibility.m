function decision = evaluateFixedHorizonFeasibility(scenario,z0,T,options)
%EVALUATEFIXEDHORIZONFEASIBILITY Certified inner finite-chart preservation.
%   D = avlc.analysis.evaluateFixedHorizonFeasibility(SCENARIO,Z0,T,OPTIONS)
%   reuses frozen global membership, GF2, longitudinal search/certificate,
%   lateral reachability and capability geometry. One longitudinal witness
%   and a deterministic finite rho grid are tried. Numerical failure is
%   UNRESOLVED, never an infeasibility proof. Exact negatives come only from
%   initial global nonmembership or defined initial GF2 NOT_PRESERVABLE.
%   T=0 requires only initial closed membership, without search or Lift.
%   OPTIONS: numSegments=8,maxCutIterations=20 (forwarded unchanged to the
%   frozen searcher when needed), numRhoSamples=33 (integer>=2). Forwarded
%   option value validation is deferred until search actually consumes it.
%   No input is modified. No propagation, frontier, opportunity-time, LP or
%   lateral reachable-set formula is duplicated. No explicit ay is built.
%   Source: frozen Master P5b-Lift and Part IX sections 136-145, P5b-T3.
%   Malformed consumed data: AVLC:InvalidFixedHorizonFeasibilityInput.

if nargin < 4
    options = struct();
end
[T,longOptions,numRhoSamples] = parseRequest(T,options);
decision = emptyDecision();
stage = "INITIAL_GLOBAL_MEMBERSHIP";
try
    membership0 = avlc.analysis.evaluateGlobalPreservationMembership(scenario,z0);
    decision.initial.membership = membership0;
    if ~membership0.preservation.inSet
        decision.status = "INFEASIBILITY_CERTIFIED";
        decision.infeasibleCertified = true;
        decision.reason = "INITIAL_STATE_OUTSIDE_GLOBAL_PRESERVATION_SET";
        decision.infeasibilityCertificate = struct('type',"INITIAL_GLOBAL_NONMEMBERSHIP", ...
            'membership0',membership0);
        return;
    end
    if T == 0
        decision.status = "CERTIFIED_FEASIBLE";
        decision.feasibleCertified = true;
        decision.reason = "ZERO_DURATION_GLOBAL_MEMBERSHIP";
        decision.feasibilityCertificate = struct('type',"ZERO_DURATION_GLOBAL_MEMBERSHIP", ...
            'T',T,'terminalState',z0,'terminalMembership',membership0, ...
            'theorem',struct('zeroDuration',true,'initialGlobalMembership',true));
        return;
    end

    stage = "INITIAL_CAPABILITY";
    cap0 = avlc.analysis.evaluateCapability(scenario,z0);
    d0 = avlc.analysis.evaluateCapabilityDiagnostics(z0.y,cap0);
    local0 = avlc.analysis.evaluateLocalCompatibility(scenario,z0,cap0,d0);
    decision.initial.capability = cap0;
    decision.initial.diagnostics = d0;
    decision.initial.localCompatibility = local0;
    if local0.defined && local0.status == "NOT_PRESERVABLE"
        decision.status = "INFEASIBILITY_CERTIFIED";
        decision.infeasibleCertified = true;
        decision.reason = "INITIAL_LOCAL_SHARED_CONTROL_INCOMPATIBLE";
        decision.infeasibilityCertificate = struct('type',"INITIAL_LOCAL_INCOMPATIBILITY", ...
            'cap0',cap0,'diagnostics0',d0,'localCompatibility0',local0);
        return;
    end

    lift = liftDomain(scenario,z0,cap0);
    decision.domain.p5bLift = lift;
    if ~lift.applicable
        decision.status = "OUTSIDE_PROVED_DOMAIN";
        decision.reason = lift.reason;
        return;
    end
    stage = "LONGITUDINAL_SEARCH";
    longitudinal = avlc.analysis.searchLongitudinalSafePrefixWitness(scenario,z0,T,longOptions);
    decision.longitudinalSearch = longitudinal;
    if longitudinal.status ~= "WITNESS_FOUND"
        decision.reason = "LONGITUDINAL_SEARCH_UNRESOLVED";
        return;
    end
    if ~isstruct(longitudinal.certificate) || ~isscalar(longitudinal.certificate) || ...
            ~all(isfield(longitudinal.certificate,{'validWitness','status'})) || ...
            ~isequal(longitudinal.certificate.validWitness,true) || ...
            longitudinal.certificate.status ~= "SAFE_WITNESS"
        decision.reason = "LONGITUDINAL_CERTIFICATE_INCONSISTENT";
        return;
    end

    stage = "LATERAL_REACHABILITY";
    upperRho = double(scenario.theoryParameters.maxLateralAccel)*T;
    if ~isfinite(upperRho)
        decision.reason = "LATERAL_REACHABILITY_ARITHMETIC_FAILURE";
        return;
    end
    bounds = avlc.analysis.evaluateLateralEndpointReachability(scenario,z0,T,upperRho);
    if ~bounds.defined || ~bounds.rhoAdmissible
        decision.reason = "LATERAL_REACHABILITY_INCONSISTENT";
        return;
    end
    rhoValues = unique(linspace(bounds.rhoLower,bounds.rhoUpper,numRhoSamples).');
    decision.rhoSearch.rhoLower = bounds.rhoLower;
    decision.rhoSearch.rhoUpper = bounds.rhoUpper;
    decision.rhoSearch.rhoValues = rhoValues;
    decision.rhoSearch.numPlannedCandidates = numel(rhoValues);
    for k = 1:numel(rhoValues)
        candidate = emptyCandidate();
        candidate.rho = rhoValues(k);
        reach = avlc.analysis.evaluateLateralEndpointReachability(scenario,z0,T,candidate.rho);
        candidate.reachability = reach;
        if ~reach.defined || ~reach.rhoAdmissible || reach.yMinus < 0 || reach.wT < 0
            candidate.reason = "LATERAL_REACHABILITY_INCONSISTENT";
            decision = recordCandidate(decision,candidate);
            decision.reason = candidate.reason;
            return;
        end

        stage = "TERMINAL_GEOMETRY";
        zProbe = longitudinal.certificate.terminal;
        zProbe.y = reach.yMinus;
        zProbe.w = reach.wT;
        capProbe = avlc.analysis.evaluateCapability(scenario,zProbe);
        candidate.probeState = zProbe;
        candidate.probeCapability = capProbe;
        candidate.YTin = capProbe.completion.YTin;
        candidate.YTout = capProbe.completion.YTout;
        candidate.YO = capProbe.abort.YO;
        candidate.Yfwd = capProbe.joint.Yfwd;
        candidate.geometryAvailable = capProbe.joint.finiteGeometryAvailable && ...
            all(isfinite([candidate.YTin,candidate.YTout,candidate.YO,candidate.Yfwd]));
        if ~candidate.geometryAvailable
            candidate.reason = "FINITE_TERMINAL_GEOMETRY_UNAVAILABLE";
            decision = recordCandidate(decision,candidate);
            continue;
        end
        L = candidate.YTin;
        U = candidate.Yfwd;
        candidate.M_corr = U-L;
        candidate.M_dead = reach.yPlus-L;
        candidate.M_fwd = U-reach.yMinus;
        candidate.minMargin = min([candidate.M_corr,candidate.M_dead,candidate.M_fwd]);
        midpoint = L+(U-L)/2;
        distance = max([reach.yMinus-midpoint,midpoint-reach.yPlus,0]);
        candidate.phiDual = (U-L)/2-distance;
        if ~all(isfinite([candidate.M_corr,candidate.M_dead,candidate.M_fwd,candidate.phiDual]))
            candidate.reason = "TERMINAL_MARGIN_ARITHMETIC_FAILURE";
            decision = recordCandidate(decision,candidate);
            decision.reason = candidate.reason;
            return;
        end
        candidate.overlap = candidate.M_corr >= 0 && candidate.M_dead >= 0 && candidate.M_fwd >= 0;
        candidate.yIntersectionLow = max(reach.yMinus,L);
        candidate.yIntersectionHigh = min(reach.yPlus,U);
        if ~candidate.overlap
            candidate.reason = "NO_INTERVAL_OVERLAP";
            decision = recordCandidate(decision,candidate);
            continue;
        end

        zT = longitudinal.certificate.terminal;
        zT.y = candidate.yIntersectionLow;
        zT.w = reach.wT;
        candidate.terminalState = zT;
        stage = "TERMINAL_CAPABILITY_CROSSCHECK";
        capT = avlc.analysis.evaluateCapability(scenario,zT);
        candidate.terminalCapability = capT;
        sameGeometry = isequal([capT.abort.YO,capT.completion.YTin, ...
            capT.completion.YTout,capT.joint.Yfwd], ...
            [candidate.YO,candidate.YTin,candidate.YTout,candidate.Yfwd]);
        if ~capT.joint.dualCapable || ~capT.joint.finiteGeometryAvailable || ~sameGeometry
            candidate.reason = "TERMINAL_CAPABILITY_CROSSCHECK_FAILED";
            decision = recordCandidate(decision,candidate);
            decision.reason = candidate.reason;
            return;
        end
        stage = "TERMINAL_GLOBAL_MEMBERSHIP_CROSSCHECK";
        membershipT = avlc.analysis.evaluateGlobalPreservationMembership(scenario,zT);
        candidate.terminalMembership = membershipT;
        if ~membershipT.preservation.inSet
            candidate.reason = "TERMINAL_GLOBAL_MEMBERSHIP_CROSSCHECK_FAILED";
            decision = recordCandidate(decision,candidate);
            decision.reason = candidate.reason;
            return;
        end
        candidate.reason = "CERTIFIED_INTERVAL_OVERLAP";
        decision = recordCandidate(decision,candidate);
        decision.status = "CERTIFIED_FEASIBLE";
        decision.feasibleCertified = true;
        decision.reason = "P5B_T3_CERTIFICATE";
        decision.feasibilityCertificate = struct('type',"P5B_T3_FINITE_CHART",'T',T, ...
            'longitudinalWitness',longitudinal.witness, ...
            'longitudinalCertificate',longitudinal.certificate, ...
            'rho',candidate.rho,'reachability',reach,'YTin',L,'YTout',candidate.YTout, ...
            'YO',candidate.YO,'Yfwd',U,'M_corr',candidate.M_corr, ...
            'M_dead',candidate.M_dead,'M_fwd',candidate.M_fwd,'phiDual',candidate.phiDual, ...
            'yTerminal',zT.y,'betaTerminal',reach.yBarT-zT.y,'terminalState',zT, ...
            'terminalCapability',capT,'terminalMembership',membershipT, ...
            'theorem',struct('p5bLiftApplicable',lift.applicable, ...
            'longitudinalWholePrefixSafe',longitudinal.certificate.validWitness, ...
            'progressiveLateralReachable',reach.defined && reach.rhoAdmissible, ...
            'terminalDualCapable',capT.joint.dualCapable,'intervalOverlap',candidate.overlap));
        return;
    end
    decision.reason = "NO_CERTIFIED_JOINT_CANDIDATE";
catch exception
    identifier = string(exception.identifier);
    if startsWith(identifier,"AVLC:Invalid")
        error('AVLC:InvalidFixedHorizonFeasibilityInput','%s (%s)', ...
            exception.message,exception.identifier);
    elseif any(identifier == ["AVLC:GlobalPreservationArithmeticFailure", ...
            "AVLC:LateralReachabilityArithmeticFailure"])
        decision.reason = stage+"_ARITHMETIC_FAILURE";
        decision.numerical.exceptionIdentifier = identifier;
        decision.numerical.exceptionMessage = string(exception.message);
    else
        rethrow(exception);
    end
end
end

function lift = liftDomain(scenario,z,cap)
B = double(scenario.theoryParameters.maxLongitudinalBrake);
C = double(scenario.theoryParameters.maxLongitudinalAccel);
lift = struct('finiteTR',isfinite(cap.abort.TRsafe),'finiteTF',isfinite(cap.abort.TFsafe), ...
    'finiteTG',isfinite(cap.abort.TG),'finiteHT',isfinite(cap.completion.HT), ...
    'deltaF',NaN,'deltaR',NaN,'deltaT',NaN,'strictDiscriminants',false, ...
    'cO0',double(z.cR)+double(z.cF),'cOPositive',false, ...
    'yNonnegative',double(z.y)>=0,'wNonnegative',double(z.w)>=0, ...
    'controlBoundsFrozen',true,'baseDefinitionFrozen',true,'applicable',false,'reason',"");
if lift.finiteTF,lift.deltaF = double(z.cF)-B*cap.abort.TFsafe;end
if lift.finiteTR,lift.deltaR = double(z.cR)-C*cap.abort.TRsafe;end
if lift.finiteHT,lift.deltaT = double(z.cT)-C*cap.completion.HT;end
lift.strictDiscriminants = all(isfinite([lift.deltaF,lift.deltaR,lift.deltaT])) && ...
    all([lift.deltaF,lift.deltaR,lift.deltaT] > 0);
lift.cOPositive = isfinite(lift.cO0) && lift.cO0 > 0;
finiteClocks = lift.finiteTR && lift.finiteTF && lift.finiteTG && lift.finiteHT;
lift.applicable = finiteClocks && lift.strictDiscriminants && lift.cOPositive && ...
    lift.yNonnegative && lift.wNonnegative && lift.controlBoundsFrozen && lift.baseDefinitionFrozen;
if ~lift.yNonnegative || ~lift.wNonnegative
    lift.reason = "OUTSIDE_PROGRESSIVE_DOMAIN";
elseif ~lift.cOPositive
    lift.reason = "NONPOSITIVE_INITIAL_CO";
elseif ~finiteClocks
    lift.reason = "INFINITE_INITIAL_OPPORTUNITY";
elseif ~lift.strictDiscriminants
    lift.reason = "NONSTRICT_INITIAL_DISCRIMINANT";
else
    lift.reason = "APPLICABLE";
end
end

function decision = emptyDecision()
decision = struct('status',"UNRESOLVED",'feasibleCertified',false,'infeasibleCertified',false, ...
    'reason',"",'initial',struct('membership',struct(),'capability',struct(), ...
    'diagnostics',struct(),'localCompatibility',struct()), ...
    'domain',struct('p5bLift',struct()),'longitudinalSearch',struct(), ...
    'rhoSearch',struct('rhoLower',NaN,'rhoUpper',NaN,'numCandidates',0, ...
    'numPlannedCandidates',0,'rhoValues',zeros(0,1),'candidates',repmat(emptyCandidate(),0,1)), ...
    'feasibilityCertificate',struct(),'infeasibilityCertificate',struct(), ...
    'numerical',struct('exceptionIdentifier',"",'exceptionMessage',""));
end

function candidate = emptyCandidate()
candidate = struct('rho',NaN,'reachability',struct(),'geometryAvailable',false, ...
    'YTin',NaN,'YTout',NaN,'YO',NaN,'Yfwd',NaN,'M_corr',NaN,'M_dead',NaN, ...
    'M_fwd',NaN,'minMargin',NaN,'phiDual',NaN,'overlap',false, ...
    'yIntersectionLow',NaN,'yIntersectionHigh',NaN,'reason',"", ...
    'probeState',struct(),'probeCapability',struct(),'terminalState',struct(), ...
    'terminalCapability',struct(),'terminalMembership',struct());
end

function decision = recordCandidate(decision,candidate)
decision.rhoSearch.candidates(end+1,1) = candidate;
decision.rhoSearch.numCandidates = numel(decision.rhoSearch.candidates);
end

function [T,longOptions,numRho] = parseRequest(T,options)
identifier = 'AVLC:InvalidFixedHorizonFeasibilityInput';
T = scalarValue(T,'T',identifier);
if T < 0,error(identifier,'T must be nonnegative.');end
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(ismember(fieldnames(options),{'numSegments','maxCutIterations','numRhoSamples'}))
    error(identifier,'options must be a scalar struct with only numSegments,maxCutIterations,numRhoSamples.');
end
longOptions = struct('numSegments',8,'maxCutIterations',20);
for field = {'numSegments','maxCutIterations'}
    if isfield(options,field{1}),longOptions.(field{1}) = options.(field{1});end
end
numRho = 33;
if isfield(options,'numRhoSamples')
    numRho = scalarValue(options.numRhoSamples,'numRhoSamples',identifier);
end
if numRho < 2 || numRho ~= fix(numRho)
    error(identifier,'numRhoSamples must be a finite integer >=2.');
end
end

function value = scalarValue(value,name,identifier)
if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
    error(identifier,'%s must be a finite real numeric scalar.',name);
end
value = double(value);
end
