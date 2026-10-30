function c = evaluateLocalCompatibility(scenario, z, cap, d)
%EVALUATELOCALCOMPATIBILITY Frozen Master §§49-67 local shared-control test.
%   C = avlc.analysis.evaluateLocalCompatibility(SCENARIO,Z,CAP,D) consumes
%   matching current-state capability and diagnostic results without
%   recomputing them. STATUS is authoritative: PRESERVABLE,
%   NOT_PRESERVABLE or UNDEFINED. localPreservable and intersection have
%   theorem meaning only when defined=true; undefined truth is NaN.
%   Exact reserve equalities select boundaries, independently of mechanisms.
%   Switch signatures preserve switch identity for positive local time.
%   No simulation, controller, optimization or witness selection is included.

avlc.scenario.validateScenario(scenario);
validateConsumed(z, cap, d);
A = double(scenario.theoryParameters.maxLateralAccel);
B = double(scenario.theoryParameters.maxLongitudinalBrake);
C = double(scenario.theoryParameters.maxLongitudinalAccel);
w = double(z.w);
r = double([d.reserves.rO, d.reserves.lT, d.reserves.rT]);
times = double([cap.abort.TRsafe, cap.abort.TFsafe, cap.abort.TG]);
HO = double(cap.abort.HO); HT = double(cap.completion.HT);
mask = logical([d.abort.activeR, d.abort.activeF, d.abort.activeG]);
upper = logical([d.completion.upperActiveI, d.completion.upperActiveII]);
c.status = "UNDEFINED";
c.defined = false;
c.domainCase = "UNSUPPORTED";
c.reason = "The current state is outside the certified finite local chart.";
checks.wPositive = w > 0;
checks.wZero = w == 0;
checks.finiteChartAvailable = all(isfinite([HO,HT,r,cap.completion.Dminus,cap.completion.Dphys]));
checks.pO = HO-w/A; checks.qO = HO+w/A;
checks.pT = HT-w/A; checks.qT = HT+w/A;
checks.strictPQ = all([checks.pO,checks.qO,checks.pT,checks.qT] > 0);
checks.reserveAdmissible = cap.joint.dualCapable && all(r >= 0);
checks.inactiveReserveStrict = all(r(r ~= 0) > 0);
abortCase = abortIdentity(mask);
upperCase = upperIdentity(upper);
checks.branchConfigurationSupported = abortCase ~= "UNSUPPORTED" && upperCase ~= "UNSUPPORTED" && ...
    d.abort.activeSetDefined && d.completion.upperActiveSetDefined && ...
    isequal(mask, times == HO) && all(times >= HO) && ...
    isequal(upper, [cap.completion.Dminus >= cap.completion.Dphys, ...
    cap.completion.Dphys >= cap.completion.Dminus]);
checks.inactiveBranchStrict = all(times(~mask) > HO);
checks.strictDiscriminants = z.cT > 0 && double(z.cT)^2-2*C*double(z.gT) > 0;
if mask(1)
    checks.strictDiscriminants = checks.strictDiscriminants && z.cR > 0 && double(z.cR)^2-2*C*double(z.gR) > 0;
end
if mask(2)
    checks.strictDiscriminants = checks.strictDiscriminants && z.cF > 0 && double(z.cF)^2-2*B*double(z.gF) > 0;
end
% Explicit sufficient auxiliary chart: physical gap boundaries and y=0
% are excluded; the represented completion overshoot boundary is not.
checks.auxiliaryStrict = z.y > 0 && all([z.gR,z.gF,z.gT] > 0);
c.domainChecks = checks;
c.activeBoundaries = struct('abort',r(1)==0,'lower',r(2)==0,'upper',r(3)==0);
c.boundaryCases = struct('abortCase',abortCase,'upperCase',upperCase);
names = {'abortF','abortG','abortR','abortFGSwitch','abortRGSwitch', ...
    'completionLower','completionUpperI','completionUpperII','completionUpperSwitch'};
c.activeConstraints = cell2struct(num2cell(false(1,9)),names,2);
c.constraints = struct('name',{},'axLower',{},'axUpper',{},'ayLower',{},'ayUpper',{});
c.intersection = struct('axLower',NaN,'axUpper',NaN,'ayLower',NaN,'ayUpper',NaN,'nonempty',NaN);
c.classification = struct('typeL',false,'typeX',false);
c.localPreservable = NaN;
gates = [checks.finiteChartAvailable,checks.strictPQ,checks.reserveAdmissible, ...
    checks.inactiveReserveStrict,checks.branchConfigurationSupported, ...
    checks.inactiveBranchStrict,checks.strictDiscriminants,checks.auxiliaryStrict];
if ~(checks.wPositive || checks.wZero) || ~all(gates)
    failed = {'finiteChartAvailable','strictPQ','reserveAdmissible','inactiveReserveStrict', ...
        'branchConfigurationSupported','inactiveBranchStrict','strictDiscriminants','auxiliaryStrict'};
    c.reason = "Failed chart checks: " + strjoin(string(failed(~gates)), ', ');
    if ~(checks.wPositive || checks.wZero), c.reason = "Progressive theory requires w >= 0."; end
    return;
end
c.defined = true;
box = [-B,C,-A,A];
if checks.wZero
    c.domainCase = "PROGRESSIVE_W_ZERO";
    box(3) = 0;
else
    c.domainCase = "REGULAR_W_POSITIVE";
end
c.constraints(end+1) = constraint("ADMISSIBLE_BOX",box);
if c.activeBoundaries.abort
    switch abortCase
        case "F_STRICT", bounds = [-B,-B,-A,-A]; flag = 'abortF';
        case "G_STRICT", bounds = [-B,C,-A,-A]; flag = 'abortG';
        case "R_STRICT", bounds = [C,C,-A,-A]; flag = 'abortR';
        case "FG_SWITCH", bounds = [-B,-B,-A,-A]; flag = 'abortFGSwitch';
        case "RG_SWITCH", bounds = [C,C,-A,-A]; flag = 'abortRGSwitch';
    end
    c.activeConstraints.(flag) = true;
    c.constraints(end+1) = constraint(abortCase,bounds);
end
if c.activeBoundaries.lower
    c.activeConstraints.completionLower = true;
    c.constraints(end+1) = constraint("COMPLETION_LOWER",[C,C,A,A]);
end
if c.activeBoundaries.upper
    switch upperCase
        case "I_STRICT", bounds = [C,C,-A,-A]; flag = 'completionUpperI';
        case "II_STRICT"
            bounds = [-B,C,-A,-A]; flag = 'completionUpperII';
            if checks.wZero, bounds(3:4) = 0; end
        case "I_II_SWITCH", bounds = [C,C,-A,-A]; flag = 'completionUpperSwitch';
    end
    c.activeConstraints.(flag) = true;
    c.constraints(end+1) = constraint(upperCase,bounds);
end
loX = max([c.constraints.axLower]); hiX = min([c.constraints.axUpper]);
loY = max([c.constraints.ayLower]); hiY = min([c.constraints.ayUpper]);
nonempty = loX <= hiX && loY <= hiY;
c.intersection = struct('axLower',loX,'axUpper',hiX,'ayLower',loY,'ayUpper',hiY,'nonempty',nonempty);
c.localPreservable = nonempty;
c.status = "NOT_PRESERVABLE";
c.reason = "The intersection of all active local requirements is empty.";
if nonempty
    c.status = "PRESERVABLE";
    c.reason = "All active requirements have a common local preserving control.";
end
% Type-L/X diagnose the regular theorem's configurations, never set truth.
if checks.wPositive
    c.classification.typeL = r(1)==0 && r(2)==0 && r(3)>0;
    c.classification.typeX = r(1)==0 && r(3)==0 && r(2)>0 && mask(2) && upper(1);
end
end

function s = constraint(name,b)
s = struct('name',name,'axLower',b(1),'axUpper',b(2),'ayLower',b(3),'ayUpper',b(4));
end

function name = abortIdentity(m)
name = "UNSUPPORTED";
if isequal(m,[true,false,false]), name="R_STRICT"; end
if isequal(m,[false,true,false]), name="F_STRICT"; end
if isequal(m,[false,false,true]), name="G_STRICT"; end
if isequal(m,[true,false,true]), name="RG_SWITCH"; end
if isequal(m,[false,true,true]), name="FG_SWITCH"; end
end

function name = upperIdentity(m)
name = "UNSUPPORTED";
if isequal(m,[true,false]), name="I_STRICT"; end
if isequal(m,[false,true]), name="II_STRICT"; end
if isequal(m,[true,true]), name="I_II_SWITCH"; end
end

function validateConsumed(z,cap,d)
id = 'AVLC:InvalidLocalCompatibilityInput';
try
    values = {z.y,z.w,z.gR,z.cR,z.gF,z.cF,z.gT,z.cT, ...
        cap.abort.TRsafe,cap.abort.TFsafe,cap.abort.TG,cap.abort.HO,cap.completion.HT, ...
        cap.completion.Dminus,cap.completion.Dphys,d.reserves.rO,d.reserves.lT,d.reserves.rT};
    flags = {cap.joint.dualCapable,d.abort.activeSetDefined,d.abort.activeR,d.abort.activeF, ...
        d.abort.activeG,d.completion.upperActiveSetDefined,d.completion.upperActiveI,d.completion.upperActiveII};
    valid = isstruct(z) && isscalar(z) && isstruct(cap) && isscalar(cap) && isstruct(d) && isscalar(d);
    for k=1:numel(values), valid = valid && isnumeric(values{k}) && isreal(values{k}) && isscalar(values{k}); end
    for k=1:numel(flags), valid = valid && islogical(flags{k}) && isscalar(flags{k}); end
    valid = valid && all(isfinite(double([z.y,z.w,z.gR,z.cR,z.gF,z.cF,z.gT,z.cT])));
catch
    valid = false;
end
if ~valid, error(id,'Supply matching scalar Scenario, z, capability and diagnostic inputs.'); end
end
