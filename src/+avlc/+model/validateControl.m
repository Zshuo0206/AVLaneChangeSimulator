function validateControl(control)
%VALIDATECONTROL Check acceleration-command data without modification.
%   avlc.model.validateControl(CONTROL) requires a scalar struct containing
%   ax and ay, each a finite real numeric scalar in m/s^2. Extra compatible
%   fields are accepted. Returns no output on success; invalid data raises
%   AVLC:InvalidControl. Does not clip, normalize, supply defaults, or apply
%   model-specific A/B/C bounds. Logical inputs are not numeric scalars.

fields = {'ax', 'ay'};
if ~isstruct(control) || ~isscalar(control) || ~all(isfield(control, fields))
    error('AVLC:InvalidControl', 'Control must be a scalar struct containing ax and ay.');
end
for index = 1:numel(fields)
    value = control.(fields{index});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
        error('AVLC:InvalidControl', 'Control.%s must be a finite real numeric scalar.', fields{index});
    end
end
end
