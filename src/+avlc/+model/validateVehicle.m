function validateVehicle(vehicle)
%VALIDATEVEHICLE Check a Phase 0 vehicle struct without changing its data.
%   avlc.model.validateVehicle(VEHICLE) requires id, role, geometry.length,
%   geometry.width, initialState, dynamics.type, and dynamics.parameters.
%   ID is nonempty text; ROLE is EGO/FO/RO/RT, compared case-insensitively.
%   Text is a nonmissing string scalar or character vector. Dimensions are
%   positive finite real numeric scalars in m. INITIALSTATE is checked by
%   validateVehicleState. Dynamics TYPE is text and PARAMETERS is a struct;
%   their meaning and vehicle uniqueness are not checked in this phase.
%   Extra fields are allowed. Returns no output on success; invalid data,
%   including invalid nested state, raises AVLC:InvalidVehicle.

fields = {'id', 'role', 'geometry', 'initialState', 'dynamics'};
if ~isstruct(vehicle) || ~isscalar(vehicle) || ~all(isfield(vehicle, fields))
    error('AVLC:InvalidVehicle', ...
        'Vehicle must be a scalar struct containing id, role, geometry, initialState, dynamics.');
end
if ~isTextScalar(vehicle.id) || strlength(string(reshape(vehicle.id, 1, []))) == 0
    error('AVLC:InvalidVehicle', 'Vehicle.id must be a nonempty text scalar.');
end
if ~isTextScalar(vehicle.role) || ...
        ~any(upper(string(reshape(vehicle.role, 1, []))) == ["EGO", "FO", "RO", "RT"])
    error('AVLC:InvalidVehicle', 'Vehicle.role must be EGO, FO, RO, or RT.');
end
geometry = vehicle.geometry;
if ~isstruct(geometry) || ~isscalar(geometry) || ...
        ~all(isfield(geometry, {'length', 'width'}))
    error('AVLC:InvalidVehicle', 'Vehicle.geometry must contain length and width.');
end
for field = {'length', 'width'}
    value = geometry.(field{1});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && ...
            isfinite(value) && value > 0)
        error('AVLC:InvalidVehicle', ...
            'Vehicle.geometry.%s must be a positive finite real numeric scalar.', field{1});
    end
end
try
    avlc.model.validateVehicleState(vehicle.initialState);
catch exception
    if ~strcmp(exception.identifier, 'AVLC:InvalidVehicleState')
        rethrow(exception);
    end
    error('AVLC:InvalidVehicle', 'Vehicle.initialState is invalid: %s', exception.message);
end
dynamics = vehicle.dynamics;
if ~isstruct(dynamics) || ~isscalar(dynamics) || ...
        ~all(isfield(dynamics, {'type', 'parameters'})) || ...
        ~isTextScalar(dynamics.type) || ~isstruct(dynamics.parameters)
    error('AVLC:InvalidVehicle', ...
        'Vehicle.dynamics must contain text scalar type and struct parameters.');
end
end

function valid = isTextScalar(value)
valid = (isstring(value) && isscalar(value) && ~ismissing(value)) || ...
    (ischar(value) && (isvector(value) || isequal(size(value), [0, 0])));
end
