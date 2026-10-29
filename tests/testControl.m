function tests = testControl
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testValidControlAndExactFields(testCase)
control = avlc.model.createControl(1.5, -2);
testCase.verifyEqual(fieldnames(control), {'ax'; 'ay'});
testCase.verifyEqual([control.ax, control.ay], [1.5, -2], 'AbsTol', 1e-10);
avlc.model.validateControl(control);
end

function testNumericInputsNormalizeToDouble(testCase)
control = avlc.model.createControl(int8(-2), uint16(3));
testCase.verifyClass(control.ax, 'double');
testCase.verifyClass(control.ay, 'double');
testCase.verifyEqual([control.ax, control.ay], [-2, 3], 'AbsTol', 1e-10);
end

function testBothSignsAndZeroAccepted(testCase)
for ax = [-1e6, 0, 1e6]
    for ay = [-1e6, 0, 1e6]
        control = avlc.model.createControl(ax, ay);
        avlc.model.validateControl(control);
    end
end
end

function testInvalidNumbersAndTypesRejected(testCase)
for value = {NaN, Inf, -Inf, 1+1i, [1, 2], [], '1', "1", true, {1}}
    testCase.verifyError(@() avlc.model.createControl(value{1}, 0), 'AVLC:InvalidControl');
    testCase.verifyError(@() avlc.model.createControl(0, value{1}), 'AVLC:InvalidControl');
    for field = {'ax', 'ay'}
        control = avlc.model.createControl(0, 0);
        control.(field{1}) = value{1};
        testCase.verifyError(@() avlc.model.validateControl(control), 'AVLC:InvalidControl');
    end
end
end

function testBothFieldsRequired(testCase)
control = avlc.model.createControl(0, 0);
for field = {'ax', 'ay'}
    invalid = rmfield(control, field{1});
    testCase.verifyError(@() avlc.model.validateControl(invalid), 'AVLC:InvalidControl');
end
end

function testScalarStructRequired(testCase)
control = avlc.model.createControl(0, 0);
for value = {[], struct([]), [control, control], 1, {control}}
    testCase.verifyError(@() avlc.model.validateControl(value{1}), 'AVLC:InvalidControl');
end
end

function testAdditionalFieldsAllowedWithoutMutation(testCase)
control = avlc.model.createControl(1, -1);
control.notes = "metadata";
before = control;
avlc.model.validateControl(control);
testCase.verifyEqual(control, before);
end
