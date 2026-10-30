function membership = evaluateGlobalPreservationMembership(scenario, z)
%EVALUATEGLOBALPRESERVATIONMEMBERSHIP Evaluate the Part-VI closed state set.
%   M = avlc.analysis.evaluateGlobalPreservationMembership(SCENARIO,Z) uses
%   only A/B/C, targetOvershoot, the two Road lane centers and the eight
%   reduced coordinates y,w,gR,cR,gF,cF,gT,cT. It evaluates the closed Psi/F
%   representation directly, without opportunity clocks, finite frontiers,
%   reserves or calls to other capability evaluators. Inputs are unchanged.
%   All set comparisons are non-strict and have no tolerance. Negative state
%   values are valid inputs. Outside y>=0,w>=0 the abort intermediates are
%   NaN and abort.inSet is false; outside physical completion recoverability
%   thetaT/psiT/FTC are NaN and completion.inSet is false. A negative y also
%   fails the base set even if the completion physical inequalities hold.
%   Malformed consumed inputs raise AVLC:InvalidGlobalPreservationInput.
%   Unrepresentable derived values raise AVLC:GlobalPreservationArithmeticFailure.
%   Source: frozen Master Part I section 2 and Part VI sections 72-77.

identifier = 'AVLC:InvalidGlobalPreservationInput';
requireFields(scenario, {'theoryParameters', 'road'}, 'scenario', identifier);
p = scenario.theoryParameters;
requireFields(p, {'maxLateralAccel', 'maxLongitudinalBrake', ...
    'maxLongitudinalAccel', 'targetOvershoot'}, 'theoryParameters', identifier);
requireFields(scenario.road, {'originalLaneCenterY', 'targetLaneCenterY'}, ...
    'road', identifier);
fields = {'y', 'w', 'gR', 'cR', 'gF', 'cF', 'gT', 'cT'};
requireFields(z, fields, 'z', identifier);
for k = 1:numel(fields)
    values.(fields{k}) = scalarValue(z.(fields{k}), fields{k}, identifier);
end
A = scalarValue(p.maxLateralAccel, 'maxLateralAccel', identifier);
B = scalarValue(p.maxLongitudinalBrake, 'maxLongitudinalBrake', identifier);
C = scalarValue(p.maxLongitudinalAccel, 'maxLongitudinalAccel', identifier);
overshoot = scalarValue(p.targetOvershoot, 'targetOvershoot', identifier);
originalY = scalarValue(scenario.road.originalLaneCenterY, 'originalLaneCenterY', identifier);
targetY = scalarValue(scenario.road.targetLaneCenterY, 'targetLaneCenterY', identifier);
if A <= 0 || B <= 0 || C <= 0 || overshoot < 0 || targetY <= originalY
    error(identifier, 'Require A,B,C>0, targetOvershoot>=0 and targetLaneCenterY>originalLaneCenterY.');
end
W = targetY - originalY;
assertFiniteArithmetic(W);
y = values.y;
w = values.w;

base.yNonnegative = y >= 0;
base.wNonnegative = w >= 0;
base.gRNonnegative = values.gR >= 0;
base.gFNonnegative = values.gF >= 0;
base.gTNonnegative = values.gT >= 0;
base.inSet = base.yNonnegative && base.wNonnegative && ...
    base.gRNonnegative && base.gFNonnegative && base.gTNonnegative;

abort = struct('domainDefined', y >= 0 && w >= 0, 'thetaO', NaN, ...
    'psiR', NaN, 'psiF', NaN, 'FRO', NaN, 'FFO', NaN, 'FGO', NaN, 'inSet', false);
if abort.domainDefined
    abort.thetaO = (w + sqrt(2*w^2 + 4*A*y))/A;
    assertFiniteArithmetic(abort.thetaO);
    abort.psiR = oneSidedViability(values.cR, C, abort.thetaO);
    abort.psiF = oneSidedViability(values.cF, B, abort.thetaO);
    abort.FRO = values.gR - abort.psiR;
    abort.FFO = values.gF - abort.psiF;
    abort.FGO = values.gR + values.gF - (values.cR + values.cF)*abort.thetaO;
    assertFiniteArithmetic([abort.psiR, abort.psiF, abort.FRO, abort.FFO, abort.FGO]);
    abort.inSet = abort.FRO >= 0 && abort.FFO >= 0 && abort.FGO >= 0;
end

completion.physicalRecoverable = false;
completion.DT = W - y;
completion.D0 = w^2/(2*A);
completion.Dphys = completion.D0 - overshoot;
assertFiniteArithmetic([completion.DT, completion.D0, completion.Dphys]);
completion.thetaT = NaN;
completion.timeBranch = "PHYSICALLY_UNRECOVERABLE";
completion.psiT = NaN;
completion.FTC = NaN;
completion.inSet = false;
if w < 0
    completion.timeBranch = "OUTSIDE_PROGRESSIVE_DOMAIN";
elseif completion.DT >= completion.Dphys
    completion.physicalRecoverable = true;
    if completion.DT >= completion.D0
        completion.timeBranch = "NON_OVERSHOOT";
        if completion.DT == completion.D0
            completion.thetaT = w/A;
        else
            completion.thetaT = (sqrt(2*w^2 + 4*A*completion.DT) - w)/A;
        end
    else
        completion.timeBranch = "OVERSHOOT_RETURN";
        completion.thetaT = (w + sqrt(2*w^2 - 4*A*completion.DT))/A;
    end
    assertFiniteArithmetic(completion.thetaT);
    completion.psiT = oneSidedViability(values.cT, C, completion.thetaT);
    completion.FTC = values.gT - completion.psiT;
    assertFiniteArithmetic([completion.psiT, completion.FTC]);
    completion.inSet = completion.FTC >= 0;
end

membership.base = base;
membership.abort = abort;
membership.completion = completion;
membership.preservation.inSet = base.inSet && abort.inSet && completion.inSet;
end

function psi = oneSidedViability(c, a, H)
% Frozen exact branches; no finite/infinite opportunity-time construction.
if c <= 0
    psi = 0;
elseif c <= a*H
    psi = c^2/(2*a);
else
    psi = c*H - a*H^2/2;
end
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
    error('AVLC:GlobalPreservationArithmeticFailure', ...
        'Derived closed-membership values exceed finite real double arithmetic.');
end
end
