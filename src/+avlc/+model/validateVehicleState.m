function validateVehicleState(state)
%VALIDATEVEHICLESTATE Check a Phase 0 vehicle-state struct without mutation.
%   avlc.model.validateVehicleState(STATE) requires a scalar struct with
%   x, y (geometric-center position, m), vx, vy (m/s), and ax, ay (m/s^2).
%   Every required value must be a finite real numeric scalar; negative
%   values and additional fields are allowed. Returns no output on success;
%   invalid data raises AVLC:InvalidVehicleState. No defaults are inserted.

fields = {'x', 'y', 'vx', 'vy', 'ax', 'ay'};
if ~isstruct(state) || ~isscalar(state) || ~all(isfield(state, fields))
    error('AVLC:InvalidVehicleState', ...
        'VehicleState must be a scalar struct containing x, y, vx, vy, ax, ay.');
end
for index = 1:numel(fields)
    value = state.(fields{index});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
        error('AVLC:InvalidVehicleState', ...
            'VehicleState.%s must be a finite real numeric scalar.', fields{index});
    end
end
end
