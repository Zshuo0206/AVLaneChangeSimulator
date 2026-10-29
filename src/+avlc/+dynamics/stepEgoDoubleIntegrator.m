function nextState = stepEgoDoubleIntegrator(state, control, theoryParameters, dt)
%STEPEGODOUBLEINTEGRATOR Exact one-step Ego kinematics with held control.
%   NEXTSTATE = avlc.dynamics.stepEgoDoubleIntegrator(STATE, CONTROL,
%   THEORYPARAMETERS, DT) validates the existing three data contracts and
%   positive finite real numeric scalar DT (s). CONTROL.ax/ay (m/s^2) are
%   authoritative: x+=vx*dt+0.5*ax*dt^2; vx+=ax*dt, likewise y/vy.
%   Requires -B<=ax<=C and -A<=ay<=A, including endpoints, using parameters
%   maxLongitudinalBrake, maxLongitudinalAccel, maxLateralAccel. Rejects
%   excess control with AVLC:ControlOutOfBounds; never clamps. Input ax/ay
%   snapshots do not drive propagation; the output records applied control.
%   Arithmetic/output are double. Inputs are unchanged. Negative vx/vy and
%   vy crossing zero are allowed. No progressive or traffic checks occur.
%   Invalid nested inputs retain their validator identifiers; invalid DT
%   raises AVLC:InvalidTimeStep. Invalid/nonfinite output raises
%   AVLC:InvalidVehicleState. No engine, dispatcher or vehicle assignment.

avlc.model.validateVehicleState(state);
avlc.model.validateControl(control);
avlc.model.validateTheoryModelParameters(theoryParameters);
if ~(isnumeric(dt) && isreal(dt) && isscalar(dt) && isfinite(dt) && dt > 0)
    error('AVLC:InvalidTimeStep', 'dt must be a positive finite real numeric scalar in seconds.');
end
dt = double(dt);
ax = double(control.ax);
ay = double(control.ay);
A = double(theoryParameters.maxLateralAccel);
B = double(theoryParameters.maxLongitudinalBrake);
C = double(theoryParameters.maxLongitudinalAccel);
if ax < -B || ax > C || ay < -A || ay > A
    error('AVLC:ControlOutOfBounds', 'Control must satisfy -B<=ax<=C and -A<=ay<=A.');
end
vx = double(state.vx);
vy = double(state.vy);
nextState = avlc.model.createVehicleState( ...
    double(state.x) + vx * dt + (0.5 * ax * dt) * dt, ...
    double(state.y) + vy * dt + (0.5 * ay * dt) * dt, ...
    vx + ax * dt, vy + ay * dt, ax, ay);
end
