function control = createControl(ax, ay)
%CREATECONTROL Create a scalar struct containing acceleration commands.
%   CONTROL = avlc.model.createControl(AX, AY) accepts finite real numeric
%   scalars in m/s^2 and returns exactly ax and ay, stored as doubles.
%   Both signs and zero are valid. No Scenario-specific acceleration bounds
%   are imposed here. Invalid data raises AVLC:InvalidControl. Admissibility
%   under the frozen model is checked by stepEgoDoubleIntegrator instead.

control = struct();
control.ax = ax;
control.ay = ay;
avlc.model.validateControl(control);
control.ax = double(ax);
control.ay = double(ay);
end
