function capability = evaluateCapability(currentState, scenario, modelParameters)
%EVALUATECAPABILITY Reserved interface for future theoretical integration.
%   CAPABILITY = avlc.analysis.evaluateCapability(CURRENTSTATE, SCENARIO,
%   MODELPARAMETERS) reserves a separate analysis entry point only.
%   K_T, K_O_abort, commitment boundary, and capability margin are neither
%   defined nor implemented. No theoretical formulas are assumed here.
%   Calling this function always raises AVLC:CapabilityNotImplemented.

error('AVLC:CapabilityNotImplemented', ...
    ['Capability evaluator is reserved for future theoretical integration ' ...
     'and has not been implemented.']);
end
