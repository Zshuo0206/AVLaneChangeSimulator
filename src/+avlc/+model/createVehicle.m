function vehicle = createVehicle(id, role, length, width, initialState)
%CREATEVEHICLE Create a Phase 0 vehicle struct with unassigned dynamics.
%   VEHICLE = avlc.model.createVehicle(ID, ROLE, LENGTH, WIDTH, INITIALSTATE)
%   accepts a nonempty text ID, a case-insensitive EGO/FO/RO/RT text role,
%   positive finite real numeric scalar dimensions in m, and a valid
%   VehicleState. Text inputs may be string scalars or character vectors.
%   Stores ID and uppercase ROLE as strings and dimensions as doubles;
%   retains the validated INITIALSTATE. Dynamics is only the placeholder
%   type "UNASSIGNED" with struct() parameters. Invalid input raises
%   AVLC:InvalidVehicle. ID/role uniqueness belongs to future Scenario code.

vehicle = struct();
vehicle.id = id;
vehicle.role = role;
vehicle.geometry = struct();
vehicle.geometry.length = length;
vehicle.geometry.width = width;
vehicle.initialState = initialState;
vehicle.dynamics = struct('type', "UNASSIGNED", 'parameters', struct());
avlc.model.validateVehicle(vehicle);

vehicle.id = string(reshape(id, 1, []));
vehicle.role = upper(string(reshape(role, 1, [])));
vehicle.geometry.length = double(length);
vehicle.geometry.width = double(width);
end
