function diagnostics = evaluateCapabilityDiagnostics(y, cap)
%EVALUATECAPABILITYDIAGNOSTICS Diagnose an existing current-capability result.
%   DIAGNOSTICS = avlc.analysis.evaluateCapabilityDiagnostics(Y, CAP) consumes
%   current Ego lateral position Y and the matching P0.2.7 CAP. It does not
%   recompute capability, opportunity times or boundaries. Inputs are not
%   modified. The caller must supply CAP evaluated at that same state.
%   reserves.rO/lT/rT are signed distances to YO/YTin/YTout; NaN propagates.
%   abort.activeR/F/G retain every exact finite HO co-minimizer, including
%   zero-time ties. activeSetDefined is false for infinite/undefined HO;
%   there is then no finite limiting mechanism and all three flags are false.
%   completion.currentBranch describes the current TTmin case, separately
%   from upperActiveI/II, the finite YTout mechanisms in Master section 16.
%   Dminus=Dphys activates both upper mechanisms. No tolerance or clipping
%   changes branch membership. Flags select mechanisms, not whether a signed
%   reserve is zero. No derivatives, controls or compatibility are computed.
%   Malformed consumed inputs raise AVLC:InvalidCapabilityDiagnosticsInput.

validateInputs(y, cap);
y = double(y);
diagnostics.reserves.rO = double(cap.abort.YO) - y;
diagnostics.reserves.lT = y - double(cap.completion.YTin);
diagnostics.reserves.rT = double(cap.completion.YTout) - y;

HO = double(cap.abort.HO);
diagnostics.abort.activeSetDefined = isfinite(HO);
diagnostics.abort.activeR = diagnostics.abort.activeSetDefined && double(cap.abort.TRsafe) == HO;
diagnostics.abort.activeF = diagnostics.abort.activeSetDefined && double(cap.abort.TFsafe) == HO;
diagnostics.abort.activeG = diagnostics.abort.activeSetDefined && double(cap.abort.TG) == HO;

DT = double(cap.completion.DT);
D0 = double(cap.completion.D0);
Dphys = double(cap.completion.Dphys);
if DT >= D0
    diagnostics.completion.currentBranch = "NON_OVERSHOOT";
elseif DT >= Dphys
    diagnostics.completion.currentBranch = "OVERSHOOT_RETURN";
else
    diagnostics.completion.currentBranch = "PHYSICALLY_UNRECOVERABLE";
end
Dminus = double(cap.completion.Dminus);
diagnostics.completion.upperActiveSetDefined = isfinite(Dminus) && isfinite(cap.completion.YTout);
diagnostics.completion.upperActiveI = diagnostics.completion.upperActiveSetDefined && Dminus >= Dphys;
diagnostics.completion.upperActiveII = diagnostics.completion.upperActiveSetDefined && Dphys >= Dminus;
end

function validateInputs(y, cap)
identifier = 'AVLC:InvalidCapabilityDiagnosticsInput';
if ~(isnumeric(y) && isreal(y) && isscalar(y) && isfinite(y) && y >= 0)
    error(identifier, 'Current lateral position y must be a finite nonnegative real numeric scalar.');
end
parts = {'abort', 'completion'};
required = {{'TRsafe', 'TFsafe', 'TG', 'HO', 'YO'}, ...
    {'DT', 'D0', 'Dphys', 'Dminus', 'YTin', 'YTout'}};
if ~isstruct(cap) || ~isscalar(cap) || ~all(isfield(cap, parts))
    error(identifier, 'Supply the matching P0.2.7 capability result.');
end
for partIndex = 1:numel(parts)
    part = cap.(parts{partIndex});
    if ~isstruct(part) || ~isscalar(part) || ~all(isfield(part, required{partIndex}))
        error(identifier, 'Capability %s is missing consumed scalar fields.', parts{partIndex});
    end
    for name = required{partIndex}
        value = part.(name{1});
        if ~(isnumeric(value) && isreal(value) && isscalar(value))
            error(identifier, 'Capability %s.%s must be a real numeric scalar.', parts{partIndex}, name{1});
        end
    end
end
if ~all(isfinite([double(cap.completion.DT), double(cap.completion.D0), double(cap.completion.Dphys)]))
    error(identifier, 'The completion lateral quantities DT, D0 and Dphys must be finite.');
end
frontiers = [double(cap.abort.YO), double(cap.completion.Dminus), ...
    double(cap.completion.YTin), double(cap.completion.YTout)];
if any(isinf(frontiers))
    error(identifier, 'Finite frontier/window fields must contain finite values or NaN, never Inf.');
end
end
