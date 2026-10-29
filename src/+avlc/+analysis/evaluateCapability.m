function cap = evaluateCapability(scenario, z)
%EVALUATECAPABILITY Evaluate frozen Parts I-III current execution capability.
%   CAP = avlc.analysis.evaluateCapability(SCENARIO, Z) uses Scenario model
%   constants and Road geometry with current reduced coordinates
%   y,w,gR,cR,gF,cF,gT,cT. It does not rebuild Z or advance physical states.
%   Invalid Scenario uses its existing validation errors. Malformed Z raises
%   AVLC:InvalidTheoryState; y<0 or w<0 raises AVLC:CapabilityDomainViolation.
%   Extra fields are accepted. Inputs are never modified.
%   Negative gaps disable only their respective execution branch. Inf is an
%   infinite opportunity or impossible completion time; NaN marks quantities
%   undefined on the current branch, including finite-only frontiers/windows.
%   All branches use exact closed comparisons without tolerance or clipping.
%   CAP contains exactly abort, completion, joint (see P0.2.7 contract).
%   joint.dualCapable means current coexistence, not positive-time shared
%   preservation. Gamma selects frozen forward-boundary geometry only.

avlc.scenario.validateScenario(scenario);
validateTheoryState(z);
y = double(z.y);
w = double(z.w);
if y < 0 || w < 0
    error('AVLC:CapabilityDomainViolation', ...
        'Frozen Parts I-III require the pre-decision domain y >= 0 and w >= 0.');
end
gR = double(z.gR);
cR = double(z.cR);
gF = double(z.gF);
cF = double(z.cF);
gT = double(z.gT);
cT = double(z.cT);
A = double(scenario.theoryParameters.maxLateralAccel);
B = double(scenario.theoryParameters.maxLongitudinalBrake);
C = double(scenario.theoryParameters.maxLongitudinalAccel);
W = double(scenario.road.targetLaneCenterY) - double(scenario.road.originalLaneCenterY);
overshoot = double(scenario.theoryParameters.targetOvershoot);

abortSafe = gR >= 0 && gF >= 0;
if abortSafe
    abort.TRsafe = opportunityTime(gR, cR, C);
    abort.TFsafe = opportunityTime(gF, cF, B);
    cO = cR + cF;
    abort.TG = Inf;
    if cO > 0
        abort.TG = (gR + gF)/cO;
    end
    abort.HO = min([abort.TRsafe, abort.TFsafe, abort.TG]);
else
    abort.TRsafe = NaN;
    abort.TFsafe = NaN;
    abort.TG = NaN;
    abort.HO = NaN;
end
abort.TOmin = (w + sqrt(2*w^2 + 4*A*y))/A;
abort.YO = NaN;
if abortSafe && isfinite(abort.HO)
    abort.YO = A*abort.HO^2/4 - abort.HO*w/2 - w^2/(4*A);
end
abort.feasible = abortSafe && abort.TOmin <= abort.HO;

targetSafe = gT >= 0;
completion.HT = NaN;
if targetSafe
    completion.HT = opportunityTime(gT, cT, C);
end
completion.DT = W - y;
completion.D0 = w^2/(2*A);
completion.Dphys = completion.D0 - overshoot;
if completion.DT >= completion.D0
    completion.TTmin = (sqrt(2*w^2 + 4*A*completion.DT) - w)/A;
elseif completion.DT >= completion.Dphys
    completion.TTmin = (w + sqrt(2*w^2 - 4*A*completion.DT))/A;
else
    completion.TTmin = Inf;
end
completion.Dplus = NaN;
completion.Dminus = NaN;
completion.YTin = NaN;
completion.YTout = NaN;
if targetSafe && isfinite(completion.HT) && completion.HT >= w/A
    completion.Dplus = A*completion.HT^2/4 + completion.HT*w/2 - w^2/(4*A);
    completion.Dminus = -A*completion.HT^2/4 + completion.HT*w/2 + w^2/(4*A);
    completion.YTin = W - completion.Dplus;
    completion.YTout = W - max(completion.Dphys, completion.Dminus);
end
% Master Part II, section 12 Case III and section 17: an infinite minimum
% time represents no admissible stable completion. MATLAB Inf<=Inf alone
% would incorrectly declare that impossible maneuver feasible.
completion.feasible = targetSafe && isfinite(completion.TTmin) && ...
    completion.TTmin <= completion.HT;

joint.dualCapable = abort.feasible && completion.feasible;
joint.finiteGeometryAvailable = abortSafe && targetSafe && ...
    isfinite(abort.HO) && isfinite(completion.HT) && completion.HT >= w/A;
joint.Yfwd = NaN;
joint.Gamma = NaN;
if joint.finiteGeometryAvailable
    joint.Yfwd = min(abort.YO, completion.YTout);
    joint.Gamma = completion.YTout - abort.YO;
end
cap.abort = abort;
cap.completion = completion;
cap.joint = joint;
end

function validateTheoryState(z)
fields = {'y', 'w', 'gR', 'cR', 'gF', 'cF', 'gT', 'cT'};
if ~isstruct(z) || ~isscalar(z) || ~all(isfield(z, fields))
    error('AVLC:InvalidTheoryState', ...
        'Theory state must be a scalar struct with y,w,gR,cR,gF,cF,gT,cT.');
end
for index = 1:numel(fields)
    value = z.(fields{index});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
        error('AVLC:InvalidTheoryState', ...
            'Theory state %s must be a finite real numeric scalar.', fields{index});
    end
end
end

function time = opportunityTime(gap, closingSpeed, acceleration)
% Called only on the branch's required initial closed longitudinal safe set.
if closingSpeed <= 0 || closingSpeed^2 <= 2*acceleration*gap
    time = Inf;
else
    time = (closingSpeed - sqrt(closingSpeed^2 - 2*acceleration*gap))/acceleration;
end
end
