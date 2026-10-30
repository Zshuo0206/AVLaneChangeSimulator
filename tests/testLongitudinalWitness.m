function tests = testLongitudinalWitness
tests = functiontests(localfunctions);
end

function setupOnce(t)
t.TestData.oldPath=path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'src'));
end

function teardownOnce(t)
path(t.TestData.oldPath);
end

function testSchemaAndNormalizedColumnRepresentation(t)
[s,z]=fixture();w=struct('times',[0,0.5,1.5,2],'ax',[-2,2,0]);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyEqual(fieldnames(c),{'status';'validWitness';'reason';'control'; ...
    'moments';'gaps';'terminal';'segments';'arithmeticAudit'});
t.verifyEqual(c.control.times,w.times(:));t.verifyEqual(c.control.ax,w.ax(:));
t.verifyEqual(c.control.mu,[0;1;0.5]);t.verifySize(c.segments,[3,1]);
t.verifyEqual(c.status,"SAFE_WITNESS");t.verifyClass(c.validWitness,'logical');
t.verifyEqual(fieldnames(c.terminal),{'gR';'cR';'gF';'cF';'gT';'cT'});
t.verifyEqual(c.arithmeticAudit.coordinateOrder,{'gR','cR','gF','cF','gT','cT'});
t.verifyTrue(c.arithmeticAudit.terminalConsistent);
end

function testTZeroSafeSingleton(t)
[s,z]=fixture();w=struct('times',0,'ax',[]);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,0,w);
t.verifyEqual(c.status,"SAFE_WITNESS");t.verifyTrue(c.validWitness);
t.verifyEqual([c.moments.eta,c.moments.zeta],[0,0]);
t.verifyEqual(c.terminal,z);t.verifySize(c.segments,[0,1]);
t.verifyEqual([c.gaps.minimumR,c.gaps.minimumF,c.gaps.minimumT],[10,20,15]);
t.verifyEqual([c.gaps.timeOfMinimumR,c.gaps.timeOfMinimumF,c.gaps.timeOfMinimumT],[0,0,0]);
t.verifySize(c.control.ax,[0,1]);t.verifySize(c.control.mu,[0,1]);
t.verifyTrue(c.moments.rawMomentLensSatisfied);
end

function testInitialUnsafeGapsAreNotMalformed(t)
[s,z]=fixture();w=struct('times',0,'ax',[]);
for field={'gR','gF','gT'}
    changed=z;changed.(field{1})=-2^-40;
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,changed,0,w);
    t.verifyEqual(c.status,"UNSAFE_WITNESS");t.verifyFalse(c.validWitness);
    t.verifyEqual(c.reason,"INITIAL_GAP_UNSAFE");t.verifyEqual(c.terminal,changed);
end
z.gR=-0.25;z.cR=-2;w=struct('times',[0,2],'ax',2);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyGreaterThan(c.terminal.gR,0);t.verifyFalse(c.validWitness);
t.verifyEqual(c.gaps.minimumR,-0.25);t.verifyEqual(c.gaps.timeOfMinimumR,0);
t.verifyEqual(c.reason,"INITIAL_GAP_UNSAFE");
end

function testConstantControlHandAnchors(t)
[s,z]=fixture();
% ax,eta,zeta,gR,cR,gF,cF,gT,cT,minR,minF,minT.
cases=[-2,0,0,4,5,26,-5,7,6,4,20,7; ...
        0,1,1,8,1,22,-1,11,2,8,20,11; ...
        2,2,2,12,-3,18,3,15,-2,9.75,18,14];
for k=1:size(cases,1)
    w=struct('times',[0,2],'ax',cases(k,1));
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
    t.verifyTrue(c.validWitness);t.verifyTrue(c.control.withinBounds);
    t.verifyEqual([c.moments.eta,c.moments.zeta],cases(k,2:3));
    t.verifyEqual(stateNumbers(c.terminal),cases(k,4:9));
    t.verifyEqual([c.gaps.minimumR,c.gaps.minimumF,c.gaps.minimumT],cases(k,10:12));
    t.verifyTrue(c.moments.rawMomentLensSatisfied);
end
end

function testThreeSegmentNonuniformHandAnchor(t)
[s,z]=fixture();w=struct('times',[0,0.5,1.5,2],'ax',[-2,2,0]);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyTrue(c.validWitness);
t.verifyEqual([c.moments.eta,c.moments.zeta],[1.25,1.0625]);
t.verifyEqual(stateNumbers(c.terminal),[8.25,0,21.75,0,11.25,1]);
t.verifyEqual(stateNumbers(c.segments(1).terminal),[9.25,2,20.75,-2,13.75,3]);
t.verifyEqual(stateNumbers(c.segments(2).terminal),[8.25,0,21.75,0,11.75,1]);
t.verifyEqual(c.segments(2).start,c.segments(1).terminal);
t.verifyEqual(c.segments(3).start,c.segments(2).terminal);
t.verifyEqual([c.gaps.minimumR,c.gaps.minimumF,c.gaps.minimumT],[8.25,20,11.25]);
t.verifyEqual([c.gaps.timeOfMinimumR,c.gaps.timeOfMinimumF,c.gaps.timeOfMinimumT],[1.5,0,2]);
t.verifyEqual(c.arithmeticAudit.terminalResidual,zeros(6,1));
t.verifyEqual(c.arithmeticAudit.terminalFromMoments,c.terminal);
end

function testRearInteriorCollisionWithSafeEndpoints(t)
[s,z,w]=collisionFixture('R',-0.5);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyGreaterThan(z.gR,0);t.verifyGreaterThan(c.terminal.gR,0);
t.verifyEqual(c.gaps.minimumR,-0.5);t.verifyEqual(c.gaps.timeOfMinimumR,1);
t.verifyEqual(c.segments.localMinTauR,1);t.verifyFalse(c.gaps.safeR);
t.verifyTrue(c.gaps.safeF && c.gaps.safeT);t.verifyFalse(c.validWitness);
t.verifyEqual(c.reason,"PREFIX_GAP_UNSAFE");
end

function testTargetInteriorCollisionWithSafeEndpoints(t)
[s,z,w]=collisionFixture('T',-0.5);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyGreaterThan(z.gT,0);t.verifyGreaterThan(c.terminal.gT,0);
t.verifyEqual(c.gaps.minimumT,-0.5);t.verifyEqual(c.gaps.timeOfMinimumT,1);
t.verifyEqual(c.segments.localMinTauT,1);t.verifyFalse(c.gaps.safeT);
t.verifyTrue(c.gaps.safeR && c.gaps.safeF);t.verifyFalse(c.validWitness);
end

function testFrontInteriorCollisionWithSafeEndpoints(t)
[s,z,w]=collisionFixture('F',-0.5);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
t.verifyGreaterThan(z.gF,0);t.verifyGreaterThan(c.terminal.gF,0);
t.verifyEqual(c.gaps.minimumF,-0.5);t.verifyEqual(c.gaps.timeOfMinimumF,1);
t.verifyEqual(c.segments.localMinTauF,1);t.verifyFalse(c.gaps.safeF);
t.verifyTrue(c.gaps.safeR && c.gaps.safeT);t.verifyFalse(c.validWitness);
end

function testExactTouchAndBinarySafetySides(t)
for role={'R','F','T'}
    for offset=[-2^-40,0,2^-40]
        [s,z,w]=collisionFixture(role{1},offset);
        c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
        t.verifyEqual(c.gaps.(['minimum' role{1}]),offset);
        t.verifyEqual(c.gaps.(['safe' role{1}]),offset>=0);
        t.verifyEqual(c.validWitness,offset>=0);
        if offset<0
            % Even a negative gap smaller than audit tolerance stays unsafe.
            t.verifyGreaterThan(max(c.arithmeticAudit.terminalTolerance),abs(offset));
        end
    end
end
end

function testStationaryEndpointsAndConcaveArcs(t)
[s,z]=fixture();z.gR=4;z.cR=4;
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct('times',[0,2],'ax',2));
t.verifyEqual(c.gaps.minimumR,0);t.verifyEqual(c.gaps.timeOfMinimumR,2);
z.gR=1;z.cR=0;
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct('times',[0,2],'ax',2));
t.verifyEqual(c.gaps.minimumR,1);t.verifyEqual(c.gaps.timeOfMinimumR,0);
% Rear concave quadratic has a maximum at 1, so its endpoints give the minimum.
z.gR=1;z.cR=-2;
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct('times',[0,2],'ax',-2));
t.verifyEqual(c.gaps.minimumR,1);t.verifyEqual(c.gaps.timeOfMinimumR,0);
z.gF=1;z.cF=-2;
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct('times',[0,2],'ax',2));
t.verifyEqual(c.gaps.minimumF,1);t.verifyEqual(c.gaps.timeOfMinimumF,0);
end

function testEarliestTimeOfTiedMinimum(t)
[s,z]=fixture();z.gR=1;z.cR=2;
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2, ...
    struct('times',[0,1,2],'ax',[2,0]));
t.verifyEqual(c.gaps.minimumR,0);t.verifyEqual(c.gaps.timeOfMinimumR,1);
t.verifyEqual(c.segments(2).localMinTauR,0);
z=struct('gR',10,'cR',0,'gF',20,'cF',0,'gT',15,'cT',0);
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2, ...
    struct('times',[0,0.25,1,2],'ax',[0,0,0]));
t.verifyEqual([c.gaps.timeOfMinimumR,c.gaps.timeOfMinimumF,c.gaps.timeOfMinimumT],[0,0,0]);
end

function testPartitionInvarianceOneTwoTenSegments(t)
[s,z]=fixture();
for N=[1,2,10]
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2, ...
        struct('times',linspace(0,2,N+1),'ax',ones(1,N)));
    t.verifyTrue(c.validWitness);
    t.verifyEqual([c.moments.eta,c.moments.zeta],[1.5,1.5],'AbsTol',1e-12);
    t.verifyEqual(stateNumbers(c.terminal),[10,-1,20,1,13,0],'AbsTol',1e-12);
    t.verifyEqual([c.gaps.minimumR,c.gaps.minimumF,c.gaps.minimumT], ...
        [9.5,20,13],'AbsTol',1e-12);
end
end

function testRawLensLowerAndUpperEndpoints(t)
[s,z]=fixture();
cases={[-2,2],0.5;[2,-2],1.5};
for k=1:2
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2, ...
        struct('times',[0,1,2],'ax',cases{k,1}));
    t.verifyEqual([c.moments.eta,c.moments.zeta],[1,cases{k,2}]);
    t.verifyTrue(c.moments.rawMomentLensSatisfied);
    t.verifyTrue(c.arithmeticAudit.rawMomentLensExact);
end
end

function testControlBoundaryAndExactOutsideSides(t)
[s,z]=fixture();z.gR=100;z.gF=100;z.gT=100;
for a=[-2-2^-40,-2,2,2+2^-40]
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,1,struct('times',[0,1],'ax',a));
    expected=a>=-2 && a<=2;
    t.verifyEqual(c.control.withinBounds,expected);t.verifyEqual(c.validWitness,expected);
    t.verifyEqual(c.control.ax,a);t.verifyEqual(c.control.mu,(a+2)/4);
    if ~expected,t.verifyEqual(c.reason,"CONTROL_OUT_OF_BOUNDS");end
end
end

function testSubstantiallyOutOfBoundsIsWitnessFailureNotArithmeticError(t)
[s,z]=fixture();
for a=[-3,3]
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct('times',[0,2],'ax',a));
    t.verifyFalse(c.validWitness);t.verifyFalse(c.control.withinBounds);
    t.verifyFalse(c.moments.rawMomentLensSatisfied);
    t.verifyEqual(c.status,"UNSAFE_WITNESS");t.verifyEqual(c.reason,"CONTROL_OUT_OF_BOUNDS");
end
end

function testTerminalAuditNonbinaryAndLargeScales(t)
[s,z]=fixture();s.theoryParameters.maxLongitudinalAccel=3;
w=struct('times',[0,0.1,0.3,0.9,1.7],'ax',[-2,0.1,3,-0.7]);
for scale=[1,1e10]
    changed=z;changed.gR=100*scale;changed.gF=120*scale;changed.gT=110*scale;
    c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,changed,1.7,w);
    t.verifyTrue(c.validWitness);t.verifyTrue(c.arithmeticAudit.terminalConsistent);
    t.verifyTrue(all(abs(c.arithmeticAudit.terminalResidual)<=c.arithmeticAudit.terminalTolerance));
    t.verifyTrue(c.moments.rawMomentLensSatisfied);
    t.verifyEqual(c.arithmeticAudit.consistencyFactor,64*5);
end
end

function testOnlyLongitudinalConsumedInputsAreRequired(t)
[s,z]=fixture();w=struct('times',[0,2],'ax',0);
expected=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w);
s.road=NaN;s.vehicles=NaN;s.theoryParameters.maxLateralAccel=NaN;
s.theoryParameters.targetOvershoot=NaN;z.y=NaN;z.w='unused';
w.eta=999;w.zeta=NaN;
t.verifyEqual(avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w),expected);
end

function testRowColumnInputsNumericTypesAndImmutability(t)
[s,z]=fixture();T=single(2);
w=struct('times',single([0,0.5,1.5,2]),'ax',int8([-2,2,0]));
before={s,z,T,w};c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,w);
t.verifyEqual({s,z,T,w},before);t.verifyClass(c.control.times,'double');
t.verifyClass(c.control.ax,'double');t.verifyClass(c.moments.eta,'double');
w.times=w.times(:);w.ax=w.ax(:);
t.verifyEqual(avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,w),c);
t.verifyEqual(avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,w),c);
end

function testMalformedPartitions(t)
[s,z]=fixture();id='AVLC:InvalidLongitudinalWitnessInput';
cases={{[0.1,1,2],[0,0]}, {[0,1,1.9],[0,0]}, ...
    {[0,1.5,1,2],[0,0,0]}, {[0,1,1,2],[0,0,0]}, ...
    {[0,1,2],0}, {[0,2],[]}, {[0,1;1,2],[0,0,0]}, ...
    {[0,0.5,1,1.5,2],[0,0;0,0]}};
for k=1:numel(cases)
    w=struct('times',cases{k}{1},'ax',cases{k}{2});
    t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w),id);
end
for w={struct('times',[0,0],'ax',0),struct('times',0,'ax',0), ...
        struct('times',1,'ax',[])}
    t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,0,w{1}),id);
end
end

function testMalformedNumericValuesAndMissingFields(t)
[s,z]=fixture();w=struct('times',[0,1,2],'ax',[0,0]);
id='AVLC:InvalidLongitudinalWitnessInput';bad={NaN,Inf,1+1i,true,'1',[],[1,2],struct()};
for field={'gR','cR','gF','cF','gT','cT'}
    for k=1:numel(bad)
        changed=z;changed.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,changed,2,w),id);
    end
end
for field={'maxLongitudinalBrake','maxLongitudinalAccel'}
    for k=1:numel(bad)
        changed=s;changed.theoryParameters.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(changed,z,2,w),id);
    end
    changed=s;changed.theoryParameters.(field{1})=0;
    t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(changed,z,2,w),id);
end
for field={'times','ax'}
    for value={NaN,Inf,1+1i,true,'1',struct()}
        changed=w;changed.(field{1})=value{1};
        t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,changed),id);
    end
end
for value={NaN,Inf,1+1i,true,-1,[1,2]}
    t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,value{1},w),id);
end
t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(struct(),z,2,w),id);
t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,rmfield(z,'cT'),2,w),id);
t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,struct()),id);
t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,[w,w]),id);
end

function testArithmeticOverflowFailsExplicitly(t)
[s,z]=fixture();z.cR=-realmax;w=struct('times',[0,2],'ax',0);
t.verifyError(@() avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,2,w), ...
    'AVLC:LongitudinalWitnessArithmeticFailure');
end

function values=stateNumbers(z)
values=[z.gR,z.cR,z.gF,z.cF,z.gT,z.cT];
end

function [s,z,w]=collisionFixture(role,minimum)
[s,z]=fixture();z.gR=100;z.gF=100;z.gT=100;z.cR=0;z.cF=0;z.cT=0;
z.(['g' role])=1+minimum;z.(['c' role])=2;
a=2;if strcmp(role,'F'),a=-2;end
w=struct('times',[0,2],'ax',a);
end

function [s,z]=fixture()
s=struct('theoryParameters',struct('maxLongitudinalBrake',2,'maxLongitudinalAccel',2));
z=struct('gR',10,'cR',1,'gF',20,'cF',-1,'gT',15,'cT',2);
end
