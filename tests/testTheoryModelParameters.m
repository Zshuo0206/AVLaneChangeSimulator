function tests = testTheoryModelParameters
%TESTTHEORYMODELPARAMETERS Positive and boundary tests for seven constants.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidCreationAndExactSchema(testCase)
params = avlc.model.createTheoryModelParameters(2, 4, 3, 0.5, 5, 6, 7);
testCase.verifyEqual(fieldnames(params), parameterFields());
testCase.verifyEqual(cell2mat(struct2cell(params)).', [2, 4, 3, 0.5, 5, 6, 7], 'AbsTol', 1e-10);
avlc.model.validateTheoryModelParameters(params);
testCase.verifyFalse(any(isfield(params, {'W', 'K', 'dF', 'dR', 'dT', 'd_F', 'd_R', 'd_T'})));
end

function testIntegerAndSingleInputsNormalizeToDouble(testCase)
params = avlc.model.createTheoryModelParameters(int8(1), int16(2), uint32(3), ...
    single(0.5), int64(4), uint64(5), int32(6));
fields = fieldnames(params);
for index = 1:numel(fields)
    testCase.verifyClass(params.(fields{index}), 'double');
end
avlc.model.validateTheoryModelParameters(params);
end

function testPositiveAccelerationMagnitudesAccepted(testCase)
params = avlc.model.createTheoryModelParameters(1e-12, 1e-12, 1e-12, 0, 0, 0, 0);
avlc.model.validateTheoryModelParameters(params);
end

function testZeroAccelerationMagnitudesRejected(testCase)
for index = 1:3
    verifyInvalidAt(testCase, index, 0);
end
end

function testNegativeAccelerationMagnitudesRejected(testCase)
for index = 1:3
    verifyInvalidAt(testCase, index, -1);
end
end

function testZeroOvershootAndClearancesAccepted(testCase)
params = avlc.model.createTheoryModelParameters(1, 2, 3, 0, 0, 0, 0);
avlc.model.validateTheoryModelParameters(params);
testCase.verifyEqual(cell2mat(struct2cell(params)).', [1, 2, 3, 0, 0, 0, 0], 'AbsTol', 1e-10);
end

function testPositiveOvershootAndClearancesAccepted(testCase)
params = avlc.model.createTheoryModelParameters(1, 2, 3, 1, 2, 3, 4);
avlc.model.validateTheoryModelParameters(params);
end

function testNegativeOvershootAndClearancesRejected(testCase)
for index = 4:7
    verifyInvalidAt(testCase, index, -1);
end
end

function testNonfiniteValuesRejectedInEveryField(testCase)
for index = 1:7
    for value = {NaN, Inf, -Inf}
        verifyInvalidAt(testCase, index, value{1});
    end
end
end

function testComplexNonscalarAndNonnumericRejected(testCase)
for index = 1:7
    for value = {1+1i, [1, 2], [], '1', "1", true, {1}}
        verifyInvalidAt(testCase, index, value{1});
    end
end
end

function testEveryRequiredFieldIsRequired(testCase)
params = validParameters();
fields = parameterFields();
for index = 1:numel(fields)
    invalid = rmfield(params, fields{index});
    testCase.verifyError(@() avlc.model.validateTheoryModelParameters(invalid), ...
        'AVLC:InvalidTheoryModelParameters');
end
end

function testScalarStructRequired(testCase)
params = validParameters();
for value = {[], struct([]), [params, params], 1, {params}}
    testCase.verifyError(@() avlc.model.validateTheoryModelParameters(value{1}), ...
        'AVLC:InvalidTheoryModelParameters');
end
end

function testAdditionalFieldAllowedWithoutMutation(testCase)
params = validParameters();
params.notes = "future metadata";
before = params;
avlc.model.validateTheoryModelParameters(params);
testCase.verifyEqual(params, before);
end

function verifyInvalidAt(testCase, index, value)
inputs = {1, 2, 3, 0, 0, 0, 0};
inputs{index} = value;
testCase.verifyError(@() avlc.model.createTheoryModelParameters(inputs{:}), ...
    'AVLC:InvalidTheoryModelParameters');
params = validParameters();
fields = parameterFields();
params.(fields{index}) = value;
testCase.verifyError(@() avlc.model.validateTheoryModelParameters(params), ...
    'AVLC:InvalidTheoryModelParameters');
end

function fields = parameterFields()
fields = {'maxLateralAccel'; 'maxLongitudinalBrake'; 'maxLongitudinalAccel'; ...
    'targetOvershoot'; 'frontSafetyClearance'; 'rearOriginalSafetyClearance'; ...
    'rearTargetSafetyClearance'};
end

function params = validParameters()
params = avlc.model.createTheoryModelParameters(1, 2, 3, 0, 0, 0, 0);
end
