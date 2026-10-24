function validateSimulationConfig(config)
%VALIDATESIMULATIONCONFIG Check a Phase 0 configuration without mutation.
%   avlc.model.validateSimulationConfig(CONFIG) requires positive finite
%   real numeric scalar dt and endTime (s), integrationMethod equal to
%   "fixed-step" (string scalar or character row), and scalar logical
%   storeEveryStep and enableCapabilityEvaluation. Either logical value is
%   valid; capability evaluation is never called. No integral endTime/dt
%   constraint is imposed. Extra fields are allowed. Returns no output on
%   success; invalid data raises AVLC:InvalidSimulationConfig.

fields = {'dt', 'endTime', 'integrationMethod', ...
    'storeEveryStep', 'enableCapabilityEvaluation'};
if ~isstruct(config) || ~isscalar(config) || ~all(isfield(config, fields))
    error('AVLC:InvalidSimulationConfig', ...
        'SimulationConfig must be a scalar struct containing all five required fields.');
end
for field = {'dt', 'endTime'}
    value = config.(field{1});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && ...
            isfinite(value) && value > 0)
        error('AVLC:InvalidSimulationConfig', ...
            'SimulationConfig.%s must be a positive finite real numeric scalar.', field{1});
    end
end
method = config.integrationMethod;
validText = (isstring(method) && isscalar(method) && ~ismissing(method)) || ...
    (ischar(method) && isrow(method));
if ~validText || ~strcmp(method, 'fixed-step')
    error('AVLC:InvalidSimulationConfig', 'SimulationConfig.integrationMethod must equal "fixed-step".');
end
for field = {'storeEveryStep', 'enableCapabilityEvaluation'}
    value = config.(field{1});
    if ~(islogical(value) && isscalar(value))
        error('AVLC:InvalidSimulationConfig', ...
            'SimulationConfig.%s must be a scalar logical.', field{1});
    end
end
end
