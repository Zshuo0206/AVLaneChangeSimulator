function scenario = createScenario(name, description, road, theoryParameters, ego, fo, ro, rt)
%CREATESCENARIO Create a structurally validated four-vehicle Phase 0 Scenario.
%   SCENARIO = avlc.scenario.createScenario(NAME, DESCRIPTION, ROAD,
%   THEORYPARAMETERS, EGO, FO, RO, RT) returns name, description, road,
%   theoryParameters, vehicles.EGO/FO/RO/RT, and events. NAME is nonempty
%   text; DESCRIPTION may be empty. Character vectors and nonmissing string
%   scalars are stored as string scalars. Nested inputs pass their existing
%   validators and are retained unchanged; roles must match their slots and
%   IDs must be distinct using case-sensitive normalized string equality.
%   Events is struct([]), with no event argument or semantics. Errors use
%   AVLC:InvalidScenario. This constructs an experiment, not a safety or
%   capability judgment; no derived theory values or dynamics are computed.

scenario = struct();
scenario.name = name;
scenario.description = description;
scenario.road = road;
scenario.theoryParameters = theoryParameters;
scenario.vehicles = struct();
scenario.vehicles.EGO = ego;
scenario.vehicles.FO = fo;
scenario.vehicles.RO = ro;
scenario.vehicles.RT = rt;
scenario.events = struct([]);
avlc.scenario.validateScenario(scenario);
scenario.name = string(reshape(name, 1, []));
scenario.description = string(reshape(description, 1, []));
end
