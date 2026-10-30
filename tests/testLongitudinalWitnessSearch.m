function tests = testLongitudinalWitnessSearch
tests = functiontests(localfunctions);
end

function setupOnce(t)
t.TestData.oldPath=path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'src'));
t.TestData.hasSolver=~isempty(which('linprog')) && license('test','Optimization_Toolbox');
end

function teardownOnce(t)
path(t.TestData.oldPath);
end

function testRealEasySuccessAndDefaultOptions(t)
requireSolver(t);[s,z]=fixture();
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2);
verifyFound(t,r,s,z,2);
t.verifyEqual(r.search.numSegments,8);t.verifyEqual(r.search.maxCutIterations,20);
t.verifyEqual(r.search.partitionTimes,linspace(0,2,9).');
t.verifyEqual(r.search.solveCount,1);t.verifyEqual(r.search.verifiedCandidateCount,1);
t.verifyEqual(r.search.cutIterationCount,0);
t.verifyEqual(fieldnames(r),{'status';'witnessFound';'reason';'witness';'certificate';'search';'attempts'});
end

function testRealRearFalsePositiveRegression(t)
requireSolver(t);[s,z]=collisionFixture('R');
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
verifyUnresolved(t,r);t.assertGreaterThanOrEqual(r.search.verifiedCandidateCount,1);
t.verifyGreaterThanOrEqual(r.search.cutIterationCount,1);
a=r.attempts(1);t.verifyTrue(a.verifierCalled);t.verifyFalse(a.validWitness);
t.verifyGreaterThanOrEqual(a.certificate.segments.start.gR,0);
t.verifyGreaterThanOrEqual(a.certificate.terminal.gR,0);
t.verifyLessThan(a.minimumR,0);t.verifyTrue(any(a.newCutTimes>0 & a.newCutTimes<2));
t.verifyGreaterThanOrEqual(a.ax,1.75);t.verifyLessThanOrEqual(a.ax,2);
t.verifyEqual(a.certificateStatus,"UNSAFE_WITNESS");
end

function testRealFrontFalsePositiveRegression(t)
requireSolver(t);[s,z]=collisionFixture('F');
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
verifyUnresolved(t,r);t.assertGreaterThanOrEqual(r.search.verifiedCandidateCount,1);
t.verifyGreaterThanOrEqual(r.search.cutIterationCount,1);
a=r.attempts(1);t.verifyGreaterThanOrEqual(a.certificate.segments.start.gF,0);
t.verifyGreaterThanOrEqual(a.certificate.terminal.gF,0);
t.verifyLessThan(a.minimumF,0);t.verifyTrue(any(a.newCutTimes>0 & a.newCutTimes<2));
end

function testRealDifferentFiniteClassesAndStableOutcomes(t)
requireSolver(t);[s,z]=fixture();
for N=[1,2,8]
    options=struct('numSegments',N);
    r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,options);
    verifyFound(t,r,s,z,2);t.verifyEqual(numel(r.witness.ax),N);
end
r1=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',2));
r2=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',2));
t.verifyEqual(r1.status,r2.status);t.verifyEqual(r1.reason,r2.reason);
t.verifyEqual([r1.search.solveCount,r1.search.cutIterationCount], ...
    [r2.search.solveCount,r2.search.cutIterationCount]);
% Bitwise-identical LP points are deliberately not a theorem requirement.
verifyFound(t,r1,s,z,2);verifyFound(t,r2,s,z,2);
end

function testTZeroNeverCallsSolverOrOptions(t)
[s,z]=fixture();guard=mockSolver({step(0,1)},'MATLAB:UndefinedFunction'); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0);
verifyFound(t,r,s,z,0);t.verifyEqual(r.search.solveCount,0);
t.verifyEqual(r.search.verifiedCandidateCount,1);
t.verifyEqual(r.attempts.source,"ZERO_HORIZON");
t.verifyEqual(r.witness.times,0);t.verifyEmpty(r.witness.ax);
z.gT=-1;r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0);
verifyUnresolved(t,r);t.verifyEqual(r.reason,"INITIAL_STATE_UNSAFE");
t.verifyEqual(r.search.solveCount,0);t.verifyFalse(r.attempts.validWitness);
data=mockData();t.verifyEqual([data.calls,data.optionCalls],[0,0]);
end

function testZeroCutBudgetStillVerifiesInitialCandidate(t)
requireSolver(t);[s,z]=collisionFixture('R');
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2, ...
    struct('numSegments',1,'maxCutIterations',0));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"CUT_LIMIT_REACHED");
t.verifyEqual(r.search.solveCount,1);t.verifyEqual(r.search.verifiedCandidateCount,1);
t.verifyEqual(r.search.cutIterationCount,0);t.verifyEmpty(r.attempts.newCutTimes);
t.verifyTrue(all(r.attempts.violationTimes>0 & r.attempts.violationTimes<2));
end

function testExactNodeLPMatrixAndFeasibilityObjective(t)
requireSolver(t);[s,z]=fixture();z.gR=10;z.cR=1;z.gF=20;z.cF=-1;z.gT=15;z.cT=2;
guard=mockSolver({step([0;0],4)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',2));
verifyFound(t,r,s,z,2);data=mockData();args=data.arguments{1};
t.verifyEqual(args{1},zeros(2,1));
t.verifyEqual(args{2},[0,0;0,0;0,0;-0.5,0;-0.5,0;0.5,0; ...
    -1.5,-0.5;-1.5,-0.5;1.5,0.5]);
t.verifyEqual(args{3},[10;15;20;9;13;21;8;11;22]);
t.verifyEmpty(args{4});t.verifyEmpty(args{5});
t.verifyEqual(args{6},[-2;-2]);t.verifyEqual(args{7},[2;2]);
t.verifyEqual(args{8},struct('Display','none'));t.verifyEqual(r.search.finalExitflag,4);
end

function testExactPartialSegmentCutCoefficient(t)
requireSolver(t);[s,z]=collisionFixture('R');
guard=mockSolver({step(2,1),step([],0)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
verifyUnresolved(t,r);t.verifyEqual(r.search.cutTimes,[0;1;2]);
t.verifyEqual(r.attempts(1).newCutTimes,1);t.verifyEqual(r.search.cutIterationCount,1);
data=mockData();args=data.arguments{2};
t.verifyEqual(args{2},[0;0;0;-0.5;-0.5;0.5;-2;-2;2]);
t.verifyEqual(args{3}(4:6),[-1.5;100;100]);
end

function testExactCutDeduplicationAndNoToleranceMerging(t)
requireSolver(t);[s,z]=collisionFixture('R');z.gT=0.5;z.cT=2;
for delta=[0,2^-35]
    changed=z;changed.cT=2+2*delta;
    guard=mockSolver({step(2,1),step([],0)}); %#ok<NASGU>
    r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,changed,2,struct('numSegments',1));
    verifyUnresolved(t,r);
    if delta==0
        t.verifyEqual(r.attempts(1).newCutTimes,1);
    else
        t.verifyEqual(r.attempts(1).newCutTimes,[1;1+delta]);
    end
    t.verifyEqual(r.search.cutIterationCount,1);
    clear guard;
end
end

function testRepeatedCutViolationIsSolverMismatch(t)
requireSolver(t);[s,z]=collisionFixture('R');
guard=mockSolver({step(0,1)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"SOLVER_CONSTRAINT_MISMATCH");
t.verifyEqual(r.attempts.violationTimes,2);t.verifyEmpty(r.attempts.newCutTimes);
t.verifyEqual(r.search.cutIterationCount,0);t.verifyEqual(r.search.solveCount,1);
end

function testOutOfBoundsSolverPointIsNotRepaired(t)
requireSolver(t);[s,z]=fixture();a=2+2^-40;
guard=mockSolver({step(a,1)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"SOLVER_CANDIDATE_OUT_OF_BOUNDS");
t.verifyEqual(r.attempts.ax,a);t.verifyEqual(r.attempts.certificate.control.ax,a);
t.verifyEqual(r.attempts.certificate.reason,"CONTROL_OUT_OF_BOUNDS");
t.verifyEmpty(r.attempts.newCutTimes);t.verifyEqual(r.search.cutIterationCount,0);
end

function testNonpositiveFlagsAndMalformedPointsNeverVerified(t)
requireSolver(t);[s,z]=fixture();
steps={step(0,0),step([],-37,'Mock LP infeasible'),step(NaN,1), ...
    step([0;0],1),step(1i,1)};
for k=1:numel(steps)
    guard=mockSolver(steps(k)); %#ok<NASGU>
    r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2,struct('numSegments',1));
    verifyUnresolved(t,r);t.verifyEqual(r.reason,"SOLVER_NO_USABLE_CANDIDATE");
    t.verifyFalse(r.attempts.verifierCalled);t.verifyEqual(r.search.verifiedCandidateCount,0);
    t.verifyEqual(r.search.finalExitflag,steps{k}.exitflag);
    t.verifyEqual(r.search.finalSolverOutput,steps{k}.output);
    if k==2,t.verifyTrue(r.search.solverReportedSearchClassInfeasible);end
    clear guard;
end
end

function testSolverUnavailableIsUnresolved(t)
[s,z]=fixture();guard=mockSolver({step(0,1)},'MATLAB:UndefinedFunction'); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2);
verifyUnresolved(t,r);t.verifyEqual(r.reason,"SOLVER_UNAVAILABLE");
t.verifyFalse(r.search.solverAvailable);t.verifyEqual(r.search.solveCount,0);
t.verifyEmpty(r.attempts);
end

function testSolverExceptionPreservesDiagnostics(t)
requireSolver(t);[s,z]=fixture();s0=step(0,1);s0.exceptionIdentifier="AVLC:MockLPNumericalFailure";
guard=mockSolver({s0}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,2);
verifyUnresolved(t,r);t.verifyEqual(r.reason,"SOLVER_FAILURE");
t.verifyEqual(r.search.exceptionIdentifier,"AVLC:MockLPNumericalFailure");
t.verifyEqual(r.search.solveCount,1);t.verifyFalse(r.attempts.verifierCalled);
end

function testFrozenVerifierArithmeticFailureIsNotUnsafeOrACut(t)
requireSolver(t);[s,z]=fixture();
s.theoryParameters.maxLongitudinalBrake=realmax;
s.theoryParameters.maxLongitudinalAccel=realmax;
guard=mockSolver({step(0,1)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,1e-155,struct('numSegments',1));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"VERIFIER_ARITHMETIC_FAILURE");
t.verifyTrue(r.attempts.verifierCalled);t.verifyEmpty(fieldnames(r.attempts.certificate));
t.verifyEqual(r.attempts.exceptionIdentifier,"AVLC:LongitudinalWitnessArithmeticFailure");
t.verifyEqual(r.search.cutIterationCount,0);t.verifyEqual(r.search.verifiedCandidateCount,0);
end

function testTZeroVerifierArithmeticFailureIsUnresolved(t)
[s,z]=fixture();s.theoryParameters.maxLongitudinalBrake=realmax;
s.theoryParameters.maxLongitudinalAccel=realmax;
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0);
verifyUnresolved(t,r);t.verifyEqual(r.reason,"VERIFIER_ARITHMETIC_FAILURE");
t.verifyEqual(r.search.solveCount,0);
end

function testLPArithmeticFailureAndUnrepresentablePartition(t)
requireSolver(t);[s,z]=fixture();guard=mockSolver({step(0,1)}); %#ok<NASGU>
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,realmax,struct('numSegments',1));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"SEARCH_ARITHMETIC_FAILURE");
t.verifyEqual(r.search.solveCount,0);
r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,realmin*eps,struct('numSegments',2));
verifyUnresolved(t,r);t.verifyEqual(r.reason,"PARTITION_NOT_REPRESENTABLE");
t.verifyEqual(r.search.solveCount,0);
end

function testOnlyConsumedFieldsAndInputsRemainUnchanged(t)
[s,z]=fixture();s.road=NaN;s.theoryParameters.maxLateralAccel=NaN;
s.theoryParameters.targetOvershoot=NaN;z.y=NaN;z.w='unused';
options=struct('numSegments',uint8(2),'maxCutIterations',0);T=single(0);
before={s,z,T,options};r=avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,T,options);
verifyFound(t,r,s,z,T);t.verifyEqual({s,z,T,options},before);
t.verifyClass(r.search.numSegments,'double');
end

function testMalformedOptions(t)
[s,z]=fixture();id='AVLC:InvalidLongitudinalWitnessSearchInput';
bad={0,-1,0.5,NaN,Inf,1i,true,'2',[],[1,2],struct()};
for k=1:numel(bad)
    t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0, ...
        struct('numSegments',bad{k})),id);
end
bad={-1,0.5,NaN,Inf,1i,true,'2',[],[1,2],struct()};
for k=1:numel(bad)
    t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0, ...
        struct('maxCutIterations',bad{k})),id);
end
t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0,[]),id);
t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0,struct('randomSeed',1)),id);
t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,0,repmat(struct(),1,2)),id);
end

function testMalformedLongitudinalInputs(t)
[s,z]=fixture();id='AVLC:InvalidLongitudinalWitnessSearchInput';
bad={NaN,Inf,1i,true,'1',[],[1,2],struct()};
for field={'gR','cR','gF','cF','gT','cT'}
    for k=1:numel(bad)
        changed=z;changed.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,changed,0),id);
    end
end
for field={'maxLongitudinalBrake','maxLongitudinalAccel'}
    for k=1:numel(bad)
        changed=s;changed.theoryParameters.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(changed,z,0),id);
    end
    changed=s;changed.theoryParameters.(field{1})=0;
    t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(changed,z,0),id);
end
for value={-1,NaN,Inf,1i,true,[1,2]}
    t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,z,value{1}),id);
end
t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(struct(),z,0),id);
t.verifyError(@() avlc.analysis.searchLongitudinalSafePrefixWitness(s,rmfield(z,'cT'),0),id);
end

function verifyFound(t,r,s,z,T)
t.assertEqual(r.status,"WITNESS_FOUND");t.verifyTrue(r.witnessFound);
t.verifyTrue(r.certificate.validWitness);t.verifyEqual(r.certificate.status,"SAFE_WITNESS");
t.verifyEqual(r.witness.times,r.certificate.control.times);
t.verifyEqual(r.witness.ax,r.certificate.control.ax);
fresh=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,r.witness);
t.verifyTrue(fresh.validWitness);t.verifyEqual(fresh,r.certificate);
end

function verifyUnresolved(t,r)
t.verifyEqual(r.status,"UNRESOLVED");t.verifyFalse(r.witnessFound);
t.verifyEmpty(fieldnames(r.witness));t.verifyEmpty(fieldnames(r.certificate));
end

function requireSolver(t)
t.assumeTrue(t.TestData.hasSolver,'Optimization Toolbox/linprog unavailable; real LP and injected-solver tests require its license.');
end

function s=step(ax,exitflag,message)
if nargin<3,message='Mock numerical result';end
s=struct('ax',ax,'exitflag',exitflag,'output',struct('message',message),'exceptionIdentifier',"");
end

function data=mockData()
data=getappdata(groot,'AVLC_P032B_MOCK_SOLVER');
end

function cleanup=mockSolver(steps,optionsError)
% Test-only canned solver responses; never a fallback optimizer or fake verifier.
if nargin<2,optionsError="";end
key='AVLC_P032B_MOCK_SOLVER';hadPrevious=isappdata(groot,key);previous=[];
if hadPrevious,previous=getappdata(groot,key);end
data=struct('steps',{steps},'calls',0,'optionCalls',0,'arguments',{{}}, ...
    'optionsError',string(optionsError));
oldPath=path;folder=tempname;mkdir(folder);
cleanup=onCleanup(@() removeMock(folder,oldPath,key,hadPrevious,previous));
setappdata(groot,key,data);
lp=["function [x,fval,exitflag,output]=linprog(varargin)"; ...
    "key='AVLC_P032B_MOCK_SOLVER';data=getappdata(groot,key);"; ...
    "data.calls=data.calls+1;data.arguments{end+1}=varargin;"; ...
    "s=data.steps{min(data.calls,numel(data.steps))};setappdata(groot,key,data);"; ...
    "if strlength(s.exceptionIdentifier)>0,error(char(s.exceptionIdentifier),'Mock numerical solver failure');end"; ...
    "x=s.ax;fval=0;exitflag=s.exitflag;output=s.output;";"end"];
opts=["function options=optimoptions(varargin)"; ...
    "key='AVLC_P032B_MOCK_SOLVER';data=getappdata(groot,key);"; ...
    "data.optionCalls=data.optionCalls+1;setappdata(groot,key,data);"; ...
    "if strlength(data.optionsError)>0,error(char(data.optionsError),'Mock unavailable solver');end"; ...
    "options=struct('Display','none');";"end"];
writeMock(fullfile(folder,'linprog.m'),lp);
writeMock(fullfile(folder,'optimoptions.m'),opts);
addpath(folder,'-begin');clear linprog optimoptions;
end

function writeMock(file,lines)
fid=fopen(file,'w');assert(fid>=0);guard=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(lines,newline));
end

function removeMock(folder,oldPath,key,hadPrevious,previous)
path(oldPath);clear linprog optimoptions;
if hadPrevious,setappdata(groot,key,previous);
elseif isappdata(groot,key),rmappdata(groot,key);end
% Delete only the two known files, then remove the empty test-owned directory.
for name={'linprog.m','optimoptions.m'}
    file=fullfile(folder,name{1});if isfile(file),delete(file);end
end
if isfolder(folder),rmdir(folder);end
end

function [s,z]=collisionFixture(role)
[s,z]=fixture();z.(['g' role])=0.5;z.(['c' role])=2;
end

function [s,z]=fixture()
s=struct('theoryParameters',struct('maxLongitudinalBrake',2,'maxLongitudinalAccel',2));
z=struct('gR',100,'cR',0,'gF',100,'cF',0,'gT',100,'cT',0);
end
