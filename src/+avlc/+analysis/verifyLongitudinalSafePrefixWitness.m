function certificate = verifyLongitudinalSafePrefixWitness(scenario, z0, T, witness)
%VERIFYLONGITUDINALSAFEPREFIXWITNESS Analytically verify one supplied control.
%   C = avlc.analysis.verifyLongitudinalSafePrefixWitness(SCENARIO,Z0,T,WITNESS)
%   consumes only B/C and z0.gR/cR/gF/cF/gT/cT. WITNESS.times defines a
%   strictly increasing partition [0,T]; WITNESS.ax is constant per interval.
%   T=0 requires times=0,ax=[]. Row/column inputs are accepted; certificate
%   vectors are columns. No supplied eta/zeta is consumed. Inputs are unchanged.
%   Segment gaps are exact quadratics: endpoints and every interior convex
%   stationary minimum are checked. Safety and acceleration bounds use closed
%   comparisons with no tolerance. A failed witness does not prove absence
%   of another safe control. No search, simulation, GF1 or lateral dependency.
%   Scale-aware roundoff allowances apply ONLY to internal moment/terminal
%   consistency audits, never to gap safety or acceleration bounds.
%   Malformed inputs raise AVLC:InvalidLongitudinalWitnessInput; arithmetic
%   range/consistency failures raise AVLC:LongitudinalWitnessArithmeticFailure.
%   Source: frozen Master Part VIII sections 107-112 and 125.

identifier = 'AVLC:InvalidLongitudinalWitnessInput';
requireFields(scenario, {'theoryParameters'}, 'scenario', identifier);
requireFields(scenario.theoryParameters, {'maxLongitudinalBrake', ...
    'maxLongitudinalAccel'}, 'theoryParameters', identifier);
names = {'gR', 'cR', 'gF', 'cF', 'gT', 'cT'};
requireFields(z0, names, 'z0', identifier);
requireFields(witness, {'times', 'ax'}, 'witness', identifier);
B = scalarValue(scenario.theoryParameters.maxLongitudinalBrake, 'B', identifier);
C = scalarValue(scenario.theoryParameters.maxLongitudinalAccel, 'C', identifier);
T = scalarValue(T, 'T', identifier);
if B <= 0 || C <= 0 || T < 0
    error(identifier, 'Require B>0, C>0 and T>=0.');
end
initial = zeros(6,1);
for k = 1:6
    initial(k) = scalarValue(z0.(names{k}), names{k}, identifier);
end
times = vectorValue(witness.times, false, 'times', identifier);
ax = vectorValue(witness.ax, true, 'ax', identifier);
N = numel(ax);
if T == 0
    partitionValid = isscalar(times) && times == 0 && N == 0;
else
    partitionValid = numel(times) == N+1 && N > 0 && times(1) == 0 && ...
        times(end) == T && all(diff(times) > 0);
end
if ~partitionValid
    error(identifier, 'Require a partition from exactly 0 to T and one ax per positive-duration segment; T=0 requires times=0,ax=[].');
end
K = B+C;
assertFiniteArithmetic(K);
h = diff(times);
withinBounds = all(ax >= -B & ax <= C);
mu = (ax+B)/K;
etaTerms = mu.*h;
% Equivalent integral weight T-(t0+t1)/2, avoiding t0+t1 overflow.
zetaTerms = etaTerms.*(T-times(1:end-1)-h/2);
eta = sum(etaTerms);
zeta = sum(zetaTerms);
assertFiniteArithmetic([mu; etaTerms; zetaTerms; eta; zeta]);

state = initial;
minimum = initial([1,3,5]);
timeOfMinimum = zeros(3,1);
recursiveScale = abs(initial);
template = struct('t0',0,'t1',0,'ax',0,'start',stateStruct(initial), ...
    'terminal',stateStruct(initial),'localMinR',0,'localMinF',0,'localMinT',0, ...
    'localMinTauR',0,'localMinTauF',0,'localMinTauT',0);
segments = repmat(template,N,1);
for k = 1:N
    a = ax(k);
    duration = h(k);
    [minR,tauR,endR] = quadraticMinimum(state(1),state(2),a,duration);
    [minF,tauF,endF] = quadraticMinimum(state(3),state(4),-a,duration);
    [minT,tauT,endT] = quadraticMinimum(state(5),state(6),a,duration);
    accelerationVelocity = a*duration;
    accelerationPosition = a*duration^2/2;
    ending = [endR;state(2)-accelerationVelocity;endF; ...
        state(4)+accelerationVelocity;endT;state(6)-accelerationVelocity];
    recursiveScale = recursiveScale + [abs(state(2)*duration)+abs(accelerationPosition); ...
        abs(accelerationVelocity);abs(state(4)*duration)+abs(accelerationPosition); ...
        abs(accelerationVelocity);abs(state(6)*duration)+abs(accelerationPosition); ...
        abs(accelerationVelocity)];
    assertFiniteArithmetic([ending;recursiveScale]);
    localMinimum = [minR;minF;minT];
    localTime = [tauR;tauF;tauT];
    improved = localMinimum < minimum;
    minimum(improved) = localMinimum(improved);
    timeOfMinimum(improved) = times(k)+localTime(improved);
    segments(k) = struct('t0',times(k),'t1',times(k+1),'ax',a, ...
        'start',stateStruct(state),'terminal',stateStruct(ending), ...
        'localMinR',minR,'localMinF',minF,'localMinT',minT, ...
        'localMinTauR',tauR,'localMinTauF',tauF,'localMinTauT',tauT);
    state = ending;
end

% Independent terminal evaluation from normalized endpoint moments.
BT = B*T;
halfBT2 = B*T^2/2;
Keta = K*eta;
Kzeta = K*zeta;
fromMoments = [initial(1)-initial(2)*T-halfBT2+Kzeta; ...
    initial(2)+BT-Keta;initial(3)-initial(4)*T+halfBT2-Kzeta; ...
    initial(4)-BT+Keta;initial(5)-initial(6)*T-halfBT2+Kzeta; ...
    initial(6)+BT-Keta];
momentScale = abs(initial) + [abs(initial(2)*T)+abs(halfBT2)+abs(Kzeta); ...
    abs(BT)+abs(Keta);abs(initial(4)*T)+abs(halfBT2)+abs(Kzeta); ...
    abs(BT)+abs(Keta);abs(initial(6)*T)+abs(halfBT2)+abs(Kzeta); ...
    abs(BT)+abs(Keta)];
factor = 64*(N+1);
terminalScale = max([ones(6,1),recursiveScale,momentScale,abs(state),abs(fromMoments)],[],2);
terminalTolerance = factor*eps(terminalScale);
terminalResidual = state-fromMoments;
eta2half = eta^2/2;
upperZeta = T*eta-eta2half;
lensMargins = [eta;T-eta;zeta-eta2half;upperZeta-zeta];
etaScale = max([1,abs(T),sum(abs(etaTerms))]);
zetaScale = max([1,abs(zeta),sum(abs(zetaTerms)),abs(eta2half),abs(T*eta)]);
lensTolerance = factor*[eps(etaScale);eps(etaScale);eps(zetaScale);eps(zetaScale)];
assertFiniteArithmetic([BT;halfBT2;Keta;Kzeta;fromMoments;momentScale; ...
    terminalResidual;terminalTolerance;lensMargins;lensTolerance]);
terminalConsistent = all(abs(terminalResidual) <= terminalTolerance);
rawLensExact = all(lensMargins >= 0);
rawLensSatisfied = all(lensMargins >= -lensTolerance);
if ~terminalConsistent || (withinBounds && ~rawLensSatisfied)
    error('AVLC:LongitudinalWitnessArithmeticFailure', ...
        'Recursive propagation and moment/raw-lens audit disagree beyond the documented roundoff allowance.');
end

safe = minimum >= 0;
valid = withinBounds && all(safe);
reason = "WHOLE_PREFIX_SAFE";
if ~withinBounds
    reason = "CONTROL_OUT_OF_BOUNDS";
elseif any(initial([1,3,5]) < 0)
    reason = "INITIAL_GAP_UNSAFE";
elseif ~all(safe)
    reason = "PREFIX_GAP_UNSAFE";
end
status = "UNSAFE_WITNESS";
if valid
    status = "SAFE_WITNESS";
end
certificate.status = status;
certificate.validWitness = valid;
certificate.reason = reason;
certificate.control = struct('times',times,'ax',ax,'mu',mu,'withinBounds',withinBounds);
certificate.moments = struct('eta',eta,'zeta',zeta,'rawMomentLensSatisfied',rawLensSatisfied);
certificate.gaps = struct('minimumR',minimum(1),'minimumF',minimum(2),'minimumT',minimum(3), ...
    'timeOfMinimumR',timeOfMinimum(1),'timeOfMinimumF',timeOfMinimum(2), ...
    'timeOfMinimumT',timeOfMinimum(3),'safeR',safe(1),'safeF',safe(2),'safeT',safe(3));
certificate.terminal = stateStruct(state);
certificate.segments = segments;
certificate.arithmeticAudit = struct('coordinateOrder',{names}, ...
    'terminalFromMoments',stateStruct(fromMoments),'terminalResidual',terminalResidual, ...
    'terminalTolerance',terminalTolerance,'terminalConsistent',terminalConsistent, ...
    'rawMomentLensExact',rawLensExact,'rawMomentLensMargins',lensMargins, ...
    'rawMomentLensTolerance',lensTolerance,'consistencyFactor',factor);
end

function [minimum,tauOfMinimum,endGap] = quadraticMinimum(g,c,curvature,h)
% q(tau)=g-c*tau+curvature*tau^2/2; candidates ordered by time for exact ties.
endGap = g-c*h+curvature*h^2/2;
candidates = [g,endGap];
tau = [0,h];
if curvature > 0
    stationary = c/curvature;
    if stationary > 0 && stationary < h
        interiorGap = g-c*stationary+curvature*stationary^2/2;
        candidates = [g,interiorGap,endGap];
        tau = [0,stationary,h];
    end
end
assertFiniteArithmetic(candidates);
[minimum,index] = min(candidates);
tauOfMinimum = tau(index);
end

function state = stateStruct(v)
state = struct('gR',v(1),'cR',v(2),'gF',v(3),'cF',v(4),'gT',v(5),'cT',v(6));
end

function requireFields(value,fields,name,identifier)
if ~isstruct(value) || ~isscalar(value) || ~all(isfield(value,fields))
    error(identifier,'%s must be a scalar struct containing all consumed fields.',name);
end
end

function value = scalarValue(value,name,identifier)
if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
    error(identifier,'%s must be a finite real numeric scalar.',name);
end
value = double(value);
end

function value = vectorValue(value,allowEmpty,name,identifier)
emptyVector = allowEmpty && ismatrix(value) && isempty(value) && ...
    size(value,1) <= 1 && size(value,2) <= 1;
if ~(isnumeric(value) && isreal(value) && (isvector(value) || emptyVector) && ...
        all(isfinite(value(:))))
    error(identifier,'%s must be a finite real numeric vector.',name);
end
value = double(value(:));
end

function assertFiniteArithmetic(values)
if ~isreal(values) || any(~isfinite(values))
    error('AVLC:LongitudinalWitnessArithmeticFailure', ...
        'Derived witness quantities exceed finite real double arithmetic.');
end
end
