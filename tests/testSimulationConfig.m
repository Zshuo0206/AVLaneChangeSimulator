function tests = testSimulationConfig
%TESTSIMULATIONCONFIG Contract tests for Phase 0 simulation configuration.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidConfigAndExactFields(testCase)
config = avlc.model.createSimulationConfig(0.1, 5);
testCase.verifyEqual(fieldnames(config), {'dt'; 'endTime'; ...
    'integrationMethod'; 'storeEveryStep'; 'enableCapabilityEvaluation'});
testCase.verifyEqual(config.dt, 0.1, 'AbsTol', 1e-10);
testCase.verifyEqual(config.endTime, 5, 'AbsTol', 1e-10);
avlc.model.validateSimulationConfig(config);
end

function testNumericTimesNormalizeToDouble(testCase)
config = avlc.model.createSimulationConfig(single(0.25), uint32(5));
testCase.verifyClass(config.dt, 'double');
testCase.verifyClass(config.endTime, 'double');
testCase.verifyEqual(config.dt, 0.25, 'AbsTol', 1e-10);
avlc.model.validateSimulationConfig(config);
end

function testPhaseZeroDefaults(testCase)
config = validConfig();
testCase.verifyEqual(config.integrationMethod, "fixed-step");
testCase.verifyClass(config.storeEveryStep, 'logical');
testCase.verifyTrue(config.storeEveryStep);
testCase.verifyClass(config.enableCapabilityEvaluation, 'logical');
testCase.verifyFalse(config.enableCapabilityEvaluation);
end

function testNonintegralEndTimeRatioAllowed(testCase)
config = avlc.model.createSimulationConfig(0.3, 1);
avlc.model.validateSimulationConfig(config);
testCase.verifyEqual(config.dt, 0.3, 'AbsTol', 1e-10);
testCase.verifyEqual(config.endTime, 1, 'AbsTol', 1e-10);
end

function testTimeStepGreaterThanEndTimeAllowed(testCase)
config = avlc.model.createSimulationConfig(2, 1);
avlc.model.validateSimulationConfig(config);
testCase.verifyEqual(config.dt, 2, 'AbsTol', 1e-10);
end

function testInvalidDtRejected(testCase)
for value = {0, -1, NaN, Inf, -Inf, 1 + 1i, [0.1, 0.2], [], "0.1", true, {0.1}}
    testCase.verifyError(@() avlc.model.createSimulationConfig(value{1}, 5), ...
        'AVLC:InvalidSimulationConfig');
    config = validConfig();
    config.dt = value{1};
    testCase.verifyError(@() avlc.model.validateSimulationConfig(config), ...
        'AVLC:InvalidSimulationConfig');
end
end

function testInvalidEndTimeRejected(testCase)
for value = {0, -1, NaN, Inf, -Inf, 1 + 1i, [1, 2], [], '5', true, {5}}
    testCase.verifyError(@() avlc.model.createSimulationConfig(0.1, value{1}), ...
        'AVLC:InvalidSimulationConfig');
    config = validConfig();
    config.endTime = value{1};
    testCase.verifyError(@() avlc.model.validateSimulationConfig(config), ...
        'AVLC:InvalidSimulationConfig');
end
end

function testMissingFieldsRejected(testCase)
config = validConfig();
fields = fieldnames(config);
for index = 1:numel(fields)
    invalid = rmfield(config, fields{index});
    testCase.verifyError(@() avlc.model.validateSimulationConfig(invalid), ...
        'AVLC:InvalidSimulationConfig');
end
end

function testScalarStructRequired(testCase)
config = validConfig();
for value = {[], struct([]), [config, config], 42, {config}}
    testCase.verifyError(@() avlc.model.validateSimulationConfig(value{1}), ...
        'AVLC:InvalidSimulationConfig');
end
end

function testInvalidIntegrationMethodsRejected(testCase)
for value = {"euler", "Fixed-step", "", 1, true, ["fixed-step", "fixed-step"], ...
        ['ab'; 'cd'], string(missing), {'fixed-step'}}
    config = validConfig();
    config.integrationMethod = value{1};
    testCase.verifyError(@() avlc.model.validateSimulationConfig(config), ...
        'AVLC:InvalidSimulationConfig');
end
end

function testCharacterIntegrationMethodAccepted(testCase)
config = validConfig();
config.integrationMethod = 'fixed-step';
avlc.model.validateSimulationConfig(config);
end

function testBooleanFieldsRequireScalarLogicals(testCase)
for field = {'storeEveryStep', 'enableCapabilityEvaluation'}
    for value = {0, 1, [], [true, false], "true", 'false', {true}}
        config = validConfig();
        config.(field{1}) = value{1};
        testCase.verifyError(@() avlc.model.validateSimulationConfig(config), ...
            'AVLC:InvalidSimulationConfig');
    end
end
end

function testBothLogicalValuesAreStructurallyValid(testCase)
for store = [false, true]
    for enable = [false, true]
        config = validConfig();
        config.storeEveryStep = store;
        config.enableCapabilityEvaluation = enable;
        avlc.model.validateSimulationConfig(config);
        testCase.verifyEqual(config.enableCapabilityEvaluation, enable);
    end
end
end

function testAdditionalFieldsAllowedWithoutMutation(testCase)
config = validConfig();
config.notes = "metadata";
before = config;
avlc.model.validateSimulationConfig(config);
testCase.verifyEqual(config, before);
end

function config = validConfig()
config = avlc.model.createSimulationConfig(0.1, 5);
end
