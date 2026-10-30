function reach = evaluateLateralEndpointReachability(scenario, z0, T, rho)
%EVALUATELATERALENDPOINTREACHABILITY Exact Part-VII progressive endpoint map.
%   R = avlc.analysis.evaluateLateralEndpointReachability(SCENARIO,Z0,T,RHO)
%   consumes only theoryParameters.maxLateralAccel, Z0.y, Z0.w, T and RHO.
%   For w0>=0,T>=0, defined is true and rhoLower/rhoUpper specify the exact
%   allocation domain. Only admissible RHO produces sigma,wT,yBarT,betaMax,
%   yMinus,yPlus; otherwise these six fields are NaN. For w0<0, defined is
%   false, rho bounds are NaN and rhoAdmissible is false. Negative y0 is a
%   valid translation of the lateral theorem, not a global-set test.
%   The returned interval is exact analytically, with no control sampling,
%   trajectory simulation, optimizer or witness construction. Comparisons
%   have no tolerance. Inputs are never modified. T=0 allows only RHO=0.
%   Malformed consumed inputs or T<0 raise AVLC:InvalidLateralReachabilityInput.
%   Unrepresentable results raise AVLC:LateralReachabilityArithmeticFailure.
%   Source: frozen Master Part VII sections 90-105, P5b-L1 and P5b-T1.

identifier = 'AVLC:InvalidLateralReachabilityInput';
requireFields(scenario, {'theoryParameters'}, 'scenario', identifier);
requireFields(scenario.theoryParameters, {'maxLateralAccel'}, 'theoryParameters', identifier);
requireFields(z0, {'y', 'w'}, 'z0', identifier);
A = scalarValue(scenario.theoryParameters.maxLateralAccel, 'maxLateralAccel', identifier);
y0 = scalarValue(z0.y, 'y', identifier);
w0 = scalarValue(z0.w, 'w', identifier);
T = scalarValue(T, 'T', identifier);
rho = scalarValue(rho, 'rho', identifier);
if A <= 0 || T < 0
    error(identifier, 'Require maxLateralAccel>0 and T>=0.');
end

reach = struct('defined', w0 >= 0, 'reason', "OUTSIDE_PROGRESSIVE_DOMAIN", ...
    'T', T, 'rhoLower', NaN, 'rhoUpper', NaN, 'rhoAdmissible', false, ...
    'sigma', NaN, 'wT', NaN, 'yBarT', NaN, 'betaMax', NaN, 'yMinus', NaN, 'yPlus', NaN);
if ~reach.defined
    return;
end
AT = A*T;
assertFiniteArithmetic(AT);
reach.rhoLower = max(0, (AT - w0)/2);
reach.rhoUpper = AT;
reach.rhoAdmissible = rho >= reach.rhoLower && rho <= reach.rhoUpper;
reach.reason = "RHO_OUTSIDE_INTERVAL";
if ~reach.rhoAdmissible
    return;
end
reach.reason = "EXACT_REACHABLE_INTERVAL";
reach.sigma = AT - rho;
% Algebraic evaluation of w0+2*rho-AT preserves zero at the active lower edge.
if AT > w0
    reach.wT = 2*(rho - reach.rhoLower);
else
    reach.wT = (w0 - AT) + 2*rho;
end
reach.yBarT = y0 + w0*T + 2*T*rho - A*T^2/2 - rho^2/A;
reach.betaMax = (2/A)*(rho*reach.sigma - max(reach.sigma - w0, 0)^2/2);
reach.yMinus = reach.yBarT - reach.betaMax;
reach.yPlus = reach.yBarT;
assertFiniteArithmetic([reach.sigma, reach.wT, reach.yBarT, ...
    reach.betaMax, reach.yMinus, reach.yPlus]);
end

function requireFields(value, fields, name, identifier)
if ~isstruct(value) || ~isscalar(value) || ~all(isfield(value, fields))
    error(identifier, '%s must be a scalar struct containing all consumed fields.', name);
end
end

function value = scalarValue(value, name, identifier)
if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
    error(identifier, '%s must be a finite real numeric scalar.', name);
end
value = double(value);
end

function assertFiniteArithmetic(values)
if ~isreal(values) || any(~isfinite(values))
    error('AVLC:LateralReachabilityArithmeticFailure', ...
        'Derived lateral endpoint values exceed finite real double arithmetic.');
end
end
