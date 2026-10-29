function params = createTheoryModelParameters(maxLateralAccel, ...
    maxLongitudinalBrake, maxLongitudinalAccel, targetOvershoot, ...
    frontSafetyClearance, rearOriginalSafetyClearance, rearTargetSafetyClearance)
%CREATETHEORYMODELPARAMETERS Create seven scalar reduced-model constants.
%   PARAMS = avlc.model.createTheoryModelParameters(A, B, C, OVERSHOOT,
%   FRONTCLEARANCE, REARORIGINALCLEARANCE, REARTARGETCLEARANCE) returns a
%   scalar struct with the seven descriptive fields named in the signature.
%   All inputs must be finite real numeric scalars. A/B/C are positive
%   acceleration magnitudes (m/s^2); overshoot and clearances are nonnegative
%   distances (m). Values are stored as doubles. No Road, geometry, initial
%   conditions, solver settings or derived theory quantities are added.
%   Invalid data raises AVLC:InvalidTheoryModelParameters.

params = struct();
params.maxLateralAccel = maxLateralAccel;
params.maxLongitudinalBrake = maxLongitudinalBrake;
params.maxLongitudinalAccel = maxLongitudinalAccel;
params.targetOvershoot = targetOvershoot;
params.frontSafetyClearance = frontSafetyClearance;
params.rearOriginalSafetyClearance = rearOriginalSafetyClearance;
params.rearTargetSafetyClearance = rearTargetSafetyClearance;
avlc.model.validateTheoryModelParameters(params);
fields = fieldnames(params);
for index = 1:numel(fields)
    params.(fields{index}) = double(params.(fields{index}));
end
end
