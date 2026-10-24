function config = createSimulationConfig(dt, endTime)
%CREATESIMULATIONCONFIG Create the Phase 0 simulation-configuration struct.
%   CONFIG = avlc.model.createSimulationConfig(DT, ENDTIME) accepts positive
%   finite real numeric scalar times in s and stores them as doubles.
%   Defaults: integrationMethod="fixed-step", storeEveryStep=true, and
%   enableCapabilityEvaluation=false. No integral endTime/dt requirement or
%   final-step policy is imposed. Does not run simulation or capability
%   evaluation. Invalid input raises AVLC:InvalidSimulationConfig.

config = struct();
config.dt = dt;
config.endTime = endTime;
config.integrationMethod = "fixed-step";
config.storeEveryStep = true;
config.enableCapabilityEvaluation = false;
avlc.model.validateSimulationConfig(config);

config.dt = double(dt);
config.endTime = double(endTime);
end
