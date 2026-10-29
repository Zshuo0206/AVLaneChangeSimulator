function validateTheoryModelParameters(params)
%VALIDATETHEORYMODELPARAMETERS Check reduced-model constants without mutation.
%   avlc.model.validateTheoryModelParameters(PARAMS) requires a scalar struct
%   with maxLateralAccel, maxLongitudinalBrake, maxLongitudinalAccel (>0,
%   m/s^2), targetOvershoot and frontSafetyClearance,
%   rearOriginalSafetyClearance, rearTargetSafetyClearance (>=0, m).
%   Required values are finite real numeric scalars. Extra compatible fields
%   are allowed. Returns no output on success; invalid input raises
%   AVLC:InvalidTheoryModelParameters. Does not derive quantities, evaluate
%   capability, insert defaults or clip values.

fields = {'maxLateralAccel', 'maxLongitudinalBrake', 'maxLongitudinalAccel', ...
    'targetOvershoot', 'frontSafetyClearance', ...
    'rearOriginalSafetyClearance', 'rearTargetSafetyClearance'};
if ~isstruct(params) || ~isscalar(params) || ~all(isfield(params, fields))
    error('AVLC:InvalidTheoryModelParameters', ...
        'TheoryModelParameters must be a scalar struct with all seven required fields.');
end
for index = 1:numel(fields)
    value = params.(fields{index});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
        error('AVLC:InvalidTheoryModelParameters', ...
            'TheoryModelParameters.%s must be a finite real numeric scalar.', fields{index});
    end
    if (index <= 3 && value <= 0) || (index > 3 && value < 0)
        error('AVLC:InvalidTheoryModelParameters', ...
            'TheoryModelParameters.%s violates its required sign constraint.', fields{index});
    end
end
end
