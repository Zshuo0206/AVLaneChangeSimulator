function state = createVehicleState(x, y, vx, vy, ax, ay)
%CREATEVEHICLESTATE Create a six-field Phase 0 vehicle-state struct.
%   STATE = avlc.model.createVehicleState(X, Y, VX, VY, AX, AY) accepts
%   finite real numeric scalars and stores each as double. Position is the
%   vehicle geometric center in m; velocity is in m/s and acceleration in
%   m/s^2. +x is forward; +y points toward the target lane. Negative values
%   are allowed. Invalid data raises AVLC:InvalidVehicleState.

state = struct();
state.x = x;
state.y = y;
state.vx = vx;
state.vy = vy;
state.ax = ax;
state.ay = ay;
avlc.model.validateVehicleState(state);

fields = fieldnames(state);
for index = 1:numel(fields)
    state.(fields{index}) = double(state.(fields{index}));
end
end
