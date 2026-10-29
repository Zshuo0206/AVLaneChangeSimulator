function validateScenario(scenario)
%VALIDATESCENARIO Validate only the structural Phase 0 experiment contract.
%   avlc.scenario.validateScenario(SCENARIO) requires a scalar struct with
%   name, description, road, theoryParameters, vehicles, events. NAME is
%   nonempty text; DESCRIPTION is text, possibly empty. VEHICLES is a scalar
%   struct requiring EGO/FO/RO/RT, each with an exactly matching internal
%   role and a pairwise distinct, case-sensitive normalized string ID.
%   Road, TheoryModelParameters and Vehicle validation is reused. Nested
%   data (SI units and geometric-center coordinates) is never normalized or
%   mutated here. EVENTS must be an empty struct array. Compatible extra
%   fields are allowed, but do not become authoritative participants.
%   Returns no output on success; invalid data raises AVLC:InvalidScenario.
%   Nested validation errors retain their cause and identify the object.
%   No ordering, lane occupancy, gaps, acceleration bounds, collision,
%   feasibility or capability conditions are checked or computed.

fields = {'name', 'description', 'road', 'theoryParameters', 'vehicles', 'events'};
if ~isstruct(scenario) || ~isscalar(scenario) || ~all(isfield(scenario, fields))
    error('AVLC:InvalidScenario', 'Scenario must be a scalar struct with all six required fields.');
end
if ~isTextScalar(scenario.name) || strlength(string(reshape(scenario.name, 1, []))) == 0
    error('AVLC:InvalidScenario', 'Scenario.name must be a nonempty text scalar.');
end
if ~isTextScalar(scenario.description)
    error('AVLC:InvalidScenario', 'Scenario.description must be a text scalar.');
end
validateNested(scenario.road, @avlc.model.validateRoad, 'Road');
validateNested(scenario.theoryParameters, ...
    @avlc.model.validateTheoryModelParameters, 'theoryParameters');
roles = {'EGO', 'FO', 'RO', 'RT'};
vehicles = scenario.vehicles;
if ~isstruct(vehicles) || ~isscalar(vehicles) || ~all(isfield(vehicles, roles))
    error('AVLC:InvalidScenario', 'Scenario.vehicles must be a scalar struct containing EGO, FO, RO, RT.');
end
ids = strings(1, 4);
for index = 1:numel(roles)
    role = roles{index};
    vehicle = vehicles.(role);
    validateNested(vehicle, @avlc.model.validateVehicle, [role ' vehicle']);
    if string(reshape(vehicle.role, 1, [])) ~= string(role)
        error('AVLC:InvalidScenario', 'Scenario %s slot requires the exact internal role %s.', role, role);
    end
    ids(index) = string(reshape(vehicle.id, 1, []));
end
if numel(unique(ids)) ~= numel(ids)
    error('AVLC:InvalidScenario', 'Scenario vehicle IDs must be pairwise distinct (case-sensitive).');
end
if ~isstruct(scenario.events) || ~isempty(scenario.events)
    error('AVLC:InvalidScenario', 'Scenario.events must be an empty struct array; events are reserved.');
end
end

function valid = isTextScalar(value)
valid = (isstring(value) && isscalar(value) && ~ismissing(value)) || ...
    (ischar(value) && (isvector(value) || isequal(size(value), [0, 0])));
end

function validateNested(value, validator, label)
try
    validator(value);
catch cause
    exception = MException('AVLC:InvalidScenario', 'Invalid Scenario: %s is invalid.', label);
    exception = addCause(exception, cause);
    throw(exception);
end
end
