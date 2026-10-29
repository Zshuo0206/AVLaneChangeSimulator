function z = buildTheoryState(scenario, egoState, foState, roState, rtState)
%BUILDTHEORYSTATE Map current physical states to frozen theory coordinates.
%   Z = avlc.analysis.buildTheoryState(SCENARIO, EGOSTATE, FOSTATE,
%   ROSTATE, RTSTATE) returns exactly y, w, gR, cR, gF, cF, gT, cT.
%   Argument order assigns current states to Scenario roles; these states
%   need not equal the Scenario initial states. Geometry and clearances come
%   from Scenario. Positions use vehicle centers and all quantities use SI.
%   Positive c means the corresponding safety-adjusted gap is closing.
%   y and w are Ego y and vy. Acceleration snapshots and surrounding y/vy
%   are structurally validated but do not participate in this mapping.
%   Required numeric values are evaluated as double without clipping,
%   rounding or tolerance corrections. Negative gaps and w are preserved.
%   This coordinate/model transformation does not determine safety,
%   theoretical applicability or capability, and never modifies inputs.

avlc.scenario.validateScenario(scenario);
avlc.model.validateVehicleState(egoState);
avlc.model.validateVehicleState(foState);
avlc.model.validateVehicleState(roState);
avlc.model.validateVehicleState(rtState);

egoLength = double(scenario.vehicles.EGO.geometry.length);
foLength = double(scenario.vehicles.FO.geometry.length);
roLength = double(scenario.vehicles.RO.geometry.length);
rtLength = double(scenario.vehicles.RT.geometry.length);
params = scenario.theoryParameters;
dF = (egoLength + foLength)/2 + double(params.frontSafetyClearance);
dR = (egoLength + roLength)/2 + double(params.rearOriginalSafetyClearance);
dT = (egoLength + rtLength)/2 + double(params.rearTargetSafetyClearance);

z.y = double(egoState.y);
z.w = double(egoState.vy);
z.gR = double(egoState.x) - double(roState.x) - dR;
z.cR = double(roState.vx) - double(egoState.vx);
z.gF = double(foState.x) - double(egoState.x) - dF;
z.cF = double(egoState.vx) - double(foState.vx);
z.gT = double(egoState.x) - double(rtState.x) - dT;
z.cT = double(rtState.vx) - double(egoState.vx);
end
