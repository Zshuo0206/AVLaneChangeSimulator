function nextState = stepConstantVelocity(state, dt)
%STEPCONSTANTVELOCITY Propagate a VehicleState at constant velocity for dt.
%   NEXTSTATE = avlc.dynamics.stepConstantVelocity(STATE, DT) uses center
%   positions (m), velocities (m/s), and positive finite real numeric scalar
%   DT (s). Exact update: x+=vx*dt, y+=vy*dt; velocity is unchanged.
%   Output acceleration snapshots are zero (m/s^2); input ax/ay are ignored.
%   Both velocity signs are allowed. Inputs are validated and arithmetic is
%   performed in double. Returns a new six-field VehicleState, without
%   changing STATE. Invalid state/output raises AVLC:InvalidVehicleState;
%   invalid DT raises AVLC:InvalidTimeStep. No role binding or traffic checks.

avlc.model.validateVehicleState(state);
if ~(isnumeric(dt) && isreal(dt) && isscalar(dt) && isfinite(dt) && dt > 0)
    error('AVLC:InvalidTimeStep', 'dt must be a positive finite real numeric scalar in seconds.');
end
dt = double(dt);
vx = double(state.vx);
vy = double(state.vy);
nextState = avlc.model.createVehicleState( ...
    double(state.x) + vx * dt, double(state.y) + vy * dt, vx, vy, 0, 0);
end
