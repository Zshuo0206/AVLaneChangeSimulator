function tests = testGlobalFoundations
tests = functiontests(localfunctions);
end

function setupOnce(t)
t.TestData.oldPath = path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'src'));
end

function teardownOnce(t)
path(t.TestData.oldPath);
end

function testGlobalSchemaAndFiniteInterior(t)
[s,z] = fixture();
m = avlc.analysis.evaluateGlobalPreservationMembership(s,z);
t.verifyEqual(fieldnames(m), {'base';'abort';'completion';'preservation'});
t.verifyEqual(fieldnames(m.base), {'yNonnegative';'wNonnegative'; ...
    'gRNonnegative';'gFNonnegative';'gTNonnegative';'inSet'});
t.verifyEqual(fieldnames(m.abort), {'domainDefined';'thetaO';'psiR'; ...
    'psiF';'FRO';'FFO';'FGO';'inSet'});
t.verifyEqual(fieldnames(m.completion), {'physicalRecoverable';'DT';'D0'; ...
    'Dphys';'thetaT';'timeBranch';'psiT';'FTC';'inSet'});
t.verifyEqual(fieldnames(m.preservation), {'inSet'});
t.verifyTrue(m.preservation.inSet);
t.verifyEqual([m.abort.thetaO,m.abort.psiR,m.abort.psiF,m.abort.FGO], [3,1,0.5,4]);
t.verifyClass(m.base.inSet,'logical'); t.verifyClass(m.abort.inSet,'logical');
t.verifyClass(m.completion.inSet,'logical'); t.verifyClass(m.preservation.inSet,'logical');
cap = avlc.analysis.evaluateCapability(s,z);
t.verifyTrue(isfinite(cap.abort.HO) && isfinite(cap.completion.HT));
verifyEquivalent(t,s,z);
end

function testAllPsiBranchesIncludingEqualities(t)
[s,z] = fixture(); z.y=0.5; z.w=0; % thetaO=1 exactly, A=2.
rear = [-1,0;0,0;1,0.25;2,1;3,2];
front = [-1,0;0,0;2,0.5;4,2;6,4];
for k=1:size(rear,1)
    z.cR=rear(k,1); z.cF=front(k,1);
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.abort.thetaO,1);
    t.verifyEqual(m.abort.psiR,rear(k,2));
    t.verifyEqual(m.abort.psiF,front(k,2));
end
z.y=3; z.w=2; % DT=D0=1, thetaT=1 exactly.
for k=1:size(rear,1)
    z.cT=rear(k,1);
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.completion.thetaT,1);
    t.verifyEqual(m.completion.psiT,rear(k,2));
end
end

function testZeroExecutionTimeWithClosingOriginalVehicles(t)
[s,z]=fixture();z.y=0;z.w=0;z.gR=0;z.gF=0;z.cR=6;z.cF=6;z.cT=0;
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
t.verifyEqual([m.abort.thetaO,m.abort.psiR,m.abort.psiF,m.abort.FGO],[0,0,0,0]);
t.verifyTrue(m.abort.inSet);t.verifyTrue(m.preservation.inSet);
verifyEquivalent(t,s,z);
end

function testInfiniteOpportunitiesDoNotRequireFiniteFrontiers(t)
[s,z]=fixture();z.cR=-1;z.cF=-2;z.cT=0;
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
cap=avlc.analysis.evaluateCapability(s,z);
t.verifyEqual([cap.abort.HO,cap.completion.HT],[Inf,Inf]);
t.verifyTrue(isnan(cap.abort.YO) && isnan(cap.completion.YTin));
t.verifyTrue(m.preservation.inSet);
t.verifyEqual([m.abort.psiR,m.abort.psiF,m.completion.psiT],[0,0,0]);
verifyEquivalent(t,s,z);
% Strictly positive closing speed at exact stopping-distance equality.
z.cR=4;z.gR=4;z.cF=-4;z.gF=2;z.cT=4;z.gT=4;
cap=avlc.analysis.evaluateCapability(s,z);
t.verifyEqual([cap.abort.HO,cap.completion.HT],[Inf,Inf]);
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
t.verifyEqual(m.abort.FRO,0);t.verifyTrue(m.preservation.inSet);
verifyEquivalent(t,s,z);
end

function testClosedBaseBoundariesAndOutsideValues(t)
[s,z]=fixture();z.w=0;z.cR=0;z.cF=0;z.cT=0;
fields={'y','w','gR','gF','gT'};
flags={'yNonnegative','wNonnegative','gRNonnegative','gFNonnegative','gTNonnegative'};
for k=1:numel(fields)
    edge=z;edge.(fields{k})=0;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,edge);
    t.verifyTrue(m.base.(flags{k}));t.verifyTrue(m.base.inSet);
    t.verifyTrue(m.preservation.inSet);
    edge.(fields{k})=-2^-40;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,edge);
    t.verifyFalse(m.base.(flags{k}));t.verifyFalse(m.base.inSet);
    t.verifyFalse(m.preservation.inSet);
end
end

function testOutsideAbortDomainAndCompletionProgressiveDomain(t)
[s,z]=fixture();
for field={'y','w'}
    changed=z;changed.(field{1})=-1;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,changed);
    t.verifyFalse(m.abort.domainDefined);t.verifyFalse(m.abort.inSet);
    t.verifyTrue(all(isnan([m.abort.thetaO,m.abort.psiR,m.abort.psiF, ...
        m.abort.FRO,m.abort.FFO,m.abort.FGO])));
    t.verifyFalse(m.preservation.inSet);
    if strcmp(field{1},'w')
        t.verifyFalse(m.completion.physicalRecoverable);
        t.verifyEqual(m.completion.timeBranch,"OUTSIDE_PROGRESSIVE_DOMAIN");
        t.verifyTrue(all(isnan([m.completion.thetaT,m.completion.psiT,m.completion.FTC])));
    end
end
end

function testNegativeGapsDisableTheirOwnMembership(t)
[s,z]=fixture();
for field={'gR','gF','gT'}
    changed=z;changed.(field{1})=-1;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,changed);
    if strcmp(field{1},'gT')
        t.verifyTrue(m.abort.inSet);t.verifyFalse(m.completion.inSet);
    else
        t.verifyFalse(m.abort.inSet);t.verifyTrue(m.completion.inSet);
    end
    verifyEquivalent(t,s,changed);
end
end

function testExactClosedMarginsAndBothSides(t)
[s,z]=fixture();z.y=0.5;z.w=0;z.cR=2;z.cF=-10;
z.gR=1; % Psi_C(2,1)=1, FRO=0.
for offset=[-2^-40,0,2^-40]
    changed=z;changed.gR=1+offset;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,changed);
    t.verifyEqual(m.abort.FRO,offset);t.verifyEqual(m.abort.inSet,offset>=0);
    verifyEquivalent(t,s,changed);
end
z.cR=-10;z.cF=4;z.gR=8;
for offset=[-2^-40,0,2^-40]
    z.gF=2+offset; % Psi_B(4,1)=2.
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.abort.FFO,offset);t.verifyEqual(m.abort.inSet,offset>=0);
    verifyEquivalent(t,s,z);
end
z.cR=2;z.cF=2;z.gR=2;
for offset=[-2^-40,0,2^-40]
    z.gF=2+offset; % FGO=gR+gF-(cR+cF)*1=offset.
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.abort.FGO,offset);t.verifyEqual(m.abort.inSet,offset>=0);
    verifyEquivalent(t,s,z);
end
z.y=3;z.w=2;z.cT=2;
for offset=[-2^-40,0,2^-40]
    z.gT=1+offset; % thetaT=1, Psi_C(2,1)=1.
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.completion.FTC,offset);t.verifyEqual(m.completion.inSet,offset>=0);
    verifyEquivalent(t,s,z);
end
end

function testCompletionPhysicalBoundaryAndInfiniteOpportunity(t)
[s,z]=fixture();z.w=2;z.cT=0; % D0=1, delta=1 => Dphys=0, boundary y=4.
for offset=[-2^-35,0,2^-35]
    z.y=4+offset;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    t.verifyEqual(m.completion.physicalRecoverable,offset<=0);
    t.verifyEqual(m.completion.inSet,offset<=0);
    if offset>0
        t.verifyEqual(m.completion.timeBranch,"PHYSICALLY_UNRECOVERABLE");
        t.verifyTrue(all(isnan([m.completion.thetaT,m.completion.psiT,m.completion.FTC])));
    else
        t.verifyEqual(m.completion.timeBranch,"OVERSHOOT_RETURN");
    end
    verifyEquivalent(t,s,z);
end
end

function testCompletionTimeSwitchIsClosedAndExact(t)
[s,z]=fixture();z.w=2;
for offset=[-2^-35,0,2^-35]
    z.y=3+offset;
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    if offset<=0
        t.verifyEqual(m.completion.timeBranch,"NON_OVERSHOOT");
    else
        t.verifyEqual(m.completion.timeBranch,"OVERSHOOT_RETURN");
    end
    if offset==0
        t.verifyEqual(m.completion.thetaT,1);
        % Both independently substituted frozen branches give w/A=1.
        t.verifyEqual((sqrt(8+8)-2)/2,1);
        t.verifyEqual((2+sqrt(8-8))/2,1);
    end
    verifyEquivalent(t,s,z);
end
end

function testGlobalWZeroHasNoLocalGF2Restriction(t)
[s,z]=fixture();z.y=3;z.w=0;
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
t.verifyTrue(m.abort.domainDefined);t.verifyTrue(m.completion.physicalRecoverable);
t.verifyTrue(m.preservation.inSet);verifyEquivalent(t,s,z);
end

function testOnlyConsumedGlobalFieldsAreValidated(t)
[s,z]=fixture();expected=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
minimal=struct('theoryParameters',s.theoryParameters,'road',s.road);
minimal.theoryParameters=rmfield(minimal.theoryParameters, ...
    {'frontSafetyClearance','rearOriginalSafetyClearance','rearTargetSafetyClearance'});
minimal.road=struct('originalLaneCenterY',0,'targetLaneCenterY',4);
t.verifyEqual(avlc.analysis.evaluateGlobalPreservationMembership(minimal,z),expected);
minimal.vehicles=NaN;minimal.theoryParameters.frontSafetyClearance=NaN;
minimal.road.displayXRange='unused';z.unused=Inf;
t.verifyEqual(avlc.analysis.evaluateGlobalPreservationMembership(minimal,z),expected);
minimal.road.originalLaneCenterY=5;minimal.road.targetLaneCenterY=9;
t.verifyEqual(avlc.analysis.evaluateGlobalPreservationMembership(minimal,z),expected);
end

function testGlobalMalformedConsumedInputs(t)
[s,z]=fixture();id='AVLC:InvalidGlobalPreservationInput';
bad={[],[1,2],1+1i,NaN,Inf,true,'1',struct()};
for field={'y','w','gR','cR','gF','cF','gT','cT'}
    for k=1:numel(bad)
        changed=z;changed.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(s,changed),id);
    end
end
for field={'maxLateralAccel','maxLongitudinalBrake','maxLongitudinalAccel','targetOvershoot'}
    for k=1:numel(bad)
        changed=s;changed.theoryParameters.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(changed,z),id);
    end
end
for field={'originalLaneCenterY','targetLaneCenterY'}
    for k=1:numel(bad)
        changed=s;changed.road.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(changed,z),id);
    end
end
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(struct(),z),id);
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(s,rmfield(z,'gT')),id);
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership([s,s],z),id);
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(s,[z,z]),id);
for field={'maxLateralAccel','maxLongitudinalBrake','maxLongitudinalAccel'}
    changed=s;changed.theoryParameters.(field{1})=0;
    t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(changed,z),id);
end
changed=s;changed.theoryParameters.targetOvershoot=-1;
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(changed,z),id);
changed=s;changed.road.targetLaneCenterY=0;
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(changed,z),id);
end

function testNumericInputTypesAreConvertedToDouble(t)
[s,z]=fixture();z.y=single(1);z.w=uint8(2);
s.theoryParameters.maxLateralAccel=uint8(2);
s.theoryParameters.maxLongitudinalBrake=int16(4);
s.theoryParameters.maxLongitudinalAccel=single(2);
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
t.verifyClass(m.abort.thetaO,'double');t.verifyClass(m.completion.D0,'double');
r=avlc.analysis.evaluateLateralEndpointReachability(s,z,single(1),single(1.5));
t.verifyClass(r.yMinus,'double');t.verifyClass(r.T,'double');
t.verifyEqual(lateralNumbers(r),[0,2,0.5,3,3.875,0.75,3.125,3.875]);
end

function testSeededTenThousandCrossEquivalenceCases(t)
[s,~]=fixture();n=10000;
stream=RandStream('mt19937ar','Seed',31001);draws=rand(stream,n,9);
actual=false(n,2);expected=false(n,2);finiteCounts=zeros(1,2);
branches=zeros(1,3);wZeroCount=0;
for k=1:n
    d=draws(k,:);
    z=struct('y',5*d(1),'w',3*d(2),'gR',20*d(3),'cR',20*d(4)-5, ...
        'gF',20*d(5),'cF',20*d(6)-5,'gT',20*d(7),'cT',20*d(8)-5);
    switch mod(k-1,8)
        case 0 % All opportunities infinite through nonclosing.
            z.cR=-5*d(4);z.cF=-5*d(6);z.cT=-5*d(8);
        case 1 % Strictly finite quadratic opportunities.
            z.cR=5+15*d(4);z.cF=5+15*d(6);z.cT=5+15*d(8);
            z.gR=z.cR^2/4*d(3)*0.95;
            z.gF=z.cF^2/8*d(5)*0.95;
            z.gT=z.cT^2/4*d(7)*0.95;
        case 3
            z.w=0;
        case 4 % Both sides of physical recovery, away from rounded equality.
            z.y=5-z.w^2/4+(2*(d(9)>0.5)-1)*(0.01+0.2*d(1));
        case 5 % Recoverable overshoot-return.
            z.y=4-z.w^2/4+0.99*d(1);
        case 6 % Non-overshoot.
            z.y=(4-z.w^2/4)*d(1);
        case 7 % Exact stopping-gap equality gives infinite quadratic clocks.
            z.cR=4;z.gR=4;z.cF=4;z.gF=2;z.cT=4;z.gT=4;
    end
    m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
    cap=avlc.analysis.evaluateCapability(s,z);
    actual(k,:)=[m.abort.inSet,m.completion.inSet];
    expected(k,:)=[cap.abort.feasible,cap.completion.feasible];
    finiteCounts=finiteCounts+[isfinite(cap.abort.HO),isfinite(cap.completion.HT)];
    branches=branches+[m.completion.timeBranch=="NON_OVERSHOOT", ...
        m.completion.timeBranch=="OVERSHOOT_RETURN", ...
        m.completion.timeBranch=="PHYSICALLY_UNRECOVERABLE"];
    wZeroCount=wZeroCount+(z.w==0);
end
mismatch=find(any(actual~=expected,2));
t.verifyEmpty(mismatch,'Closed membership disagrees with frozen execution capability.');
t.verifyTrue(all(finiteCounts>0 & finiteCounts<n));
t.verifyTrue(all(branches>0));t.verifyGreaterThan(wZeroCount,0);
fprintf(['P031_RANDOM seed=31001 n=%d mismatches=%d finiteHO=%d finiteHT=%d ' ...
    'nonOvershoot=%d overshootReturn=%d unrecoverable=%d wZero=%d\n'], ...
    n,numel(mismatch),finiteCounts,branches,wZeroCount);
end

function testLateralSchemaAndThreeHandCalculatedAnchors(t)
[s,z]=fixture();
% y0,w0,T,rho,rhoLower,rhoUpper,sigma,wT,yBarT,betaMax,yMinus,yPlus.
cases=[1,2,1,1.5,0,2,0.5,3,3.875,0.75,3.125,3.875; ...
       1,0,2,2,2,4,2,0,3,2,1,3; ...
       1,1,2,2,1.5,4,2,1,5,3.5,1.5,5];
for k=1:size(cases,1)
    z.y=cases(k,1);z.w=cases(k,2);
    r=avlc.analysis.evaluateLateralEndpointReachability(s,z,cases(k,3),cases(k,4));
    t.verifyTrue(r.defined);t.verifyTrue(r.rhoAdmissible);
    t.verifyEqual(lateralNumbers(r),cases(k,5:12));
end
t.verifyEqual(fieldnames(r), {'defined';'reason';'T';'rhoLower';'rhoUpper'; ...
    'rhoAdmissible';'sigma';'wT';'yBarT';'betaMax';'yMinus';'yPlus'});
t.verifyEqual(r.reason,"EXACT_REACHABLE_INTERVAL");
end

function testLateralTZeroIsSingleton(t)
[s,z]=fixture();z.y=-2;z.w=3;
r=avlc.analysis.evaluateLateralEndpointReachability(s,z,0,0);
t.verifyTrue(r.defined);t.verifyTrue(r.rhoAdmissible);
t.verifyEqual(lateralNumbers(r),[0,0,0,3,-2,0,-2,-2]);
for rho=[-1,2^-40,1]
    r=avlc.analysis.evaluateLateralEndpointReachability(s,z,0,rho);
    t.verifyTrue(r.defined);t.verifyFalse(r.rhoAdmissible);
    t.verifyEqual([r.rhoLower,r.rhoUpper],[0,0]);verifyUndefinedEndpoints(t,r);
end
end

function testLateralAllocationEndpointsAndExactSides(t)
[s,z]=fixture();z.w=1;
for rho=[1.5-2^-35,1.5,1.5+2^-35,4-2^-35,4,4+2^-35]
    r=avlc.analysis.evaluateLateralEndpointReachability(s,z,2,rho);
    t.verifyTrue(r.defined);t.verifyEqual([r.rhoLower,r.rhoUpper],[1.5,4]);
    t.verifyEqual(r.rhoAdmissible,rho>=1.5 && rho<=4);
    if ~r.rhoAdmissible,verifyUndefinedEndpoints(t,r);end
    if rho==1.5,t.verifyEqual(r.wT,0);end
    if rho==4
        t.verifyEqual([r.sigma,r.betaMax,r.yMinus,r.yPlus],[0,0,7,7]);
    end
end
end

function testLateralBetaStructuralCasesAndSwitch(t)
[s,z]=fixture();z.w=1;
% A=2,T=1: sigma<w0, sigma=w0, sigma>w0, all valid allocations.
cases=[1.5,0.75;1,1;0.75,0.90625];
for k=1:size(cases,1)
    r=avlc.analysis.evaluateLateralEndpointReachability(s,z,1,cases(k,1));
    t.verifyTrue(r.rhoAdmissible);t.verifyEqual(r.betaMax,cases(k,2));
end
end

function testLateralWholeIntervalOrderingAcrossParameterGrid(t)
[s,z]=fixture();
for w=[0,1,4]
    z.w=w;
    for T=[0,0.5,1,2]
        lower=max(0,(2*T-w)/2);
        for rho=linspace(lower,2*T,5)
            r=avlc.analysis.evaluateLateralEndpointReachability(s,z,T,rho);
            t.verifyTrue(r.defined && r.rhoAdmissible);
            t.verifyGreaterThanOrEqual(r.wT,0);
            t.verifyGreaterThanOrEqual(r.betaMax,0);
            t.verifyLessThanOrEqual(r.yMinus,r.yPlus);
            t.verifyEqual(r.yPlus-r.yMinus,r.betaMax);
        end
    end
end
end

function testLateralNegativeWIsUndefinedNotInputError(t)
[s,z]=fixture();z.w=-1;
r=avlc.analysis.evaluateLateralEndpointReachability(s,z,1,1);
t.verifyFalse(r.defined);t.verifyFalse(r.rhoAdmissible);
t.verifyEqual(r.reason,"OUTSIDE_PROGRESSIVE_DOMAIN");
t.verifyTrue(all(isnan([r.rhoLower,r.rhoUpper])));verifyUndefinedEndpoints(t,r);
end

function testLateralConsumedFieldsAndYTranslation(t)
s=struct('theoryParameters',struct('maxLateralAccel',2));
z=struct('y',1,'w',1);
r=avlc.analysis.evaluateLateralEndpointReachability(s,z,2,2);
s.road=NaN;s.theoryParameters.maxLongitudinalAccel=NaN;
z.gR=NaN;z.cR='unused';
t.verifyEqual(avlc.analysis.evaluateLateralEndpointReachability(s,z,2,2),r);
z.y=-1;
translated=avlc.analysis.evaluateLateralEndpointReachability(s,z,2,2);
t.verifyTrue(translated.defined);
t.verifyEqual([translated.yBarT,translated.yMinus,translated.yPlus], ...
    [r.yBarT,r.yMinus,r.yPlus]-2);
end

function testLateralMalformedConsumedInputs(t)
[s,z]=fixture();id='AVLC:InvalidLateralReachabilityInput';
bad={[],[1,2],1+1i,NaN,Inf,true,'1',struct()};
for k=1:numel(bad)
    t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,z,bad{k},1),id);
    t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,z,1,bad{k}),id);
    for field={'y','w'}
        changed=z;changed.(field{1})=bad{k};
        t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,changed,1,1),id);
    end
    changed=s;changed.theoryParameters.maxLateralAccel=bad{k};
    t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(changed,z,1,1),id);
end
t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,z,-1,1),id);
t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(struct(),z,1,1),id);
t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,rmfield(z,'w'),1,1),id);
changed=s;changed.theoryParameters.maxLateralAccel=0;
t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(changed,z,1,1),id);
end

function testInputsAreUnchangedAndCallsAreDeterministic(t)
[s,z]=fixture();before={s,z};
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
r=avlc.analysis.evaluateLateralEndpointReachability(s,z,1,1.5);
t.verifyEqual({s,z},before);
t.verifyEqual(avlc.analysis.evaluateGlobalPreservationMembership(s,z),m);
t.verifyEqual(avlc.analysis.evaluateLateralEndpointReachability(s,z,1,1.5),r);
t.verifyEqual({s,z},before);
end

function testArithmeticOverflowIsNotSilentNonmembership(t)
[s,z]=fixture();z.w=realmax;
t.verifyError(@() avlc.analysis.evaluateGlobalPreservationMembership(s,z), ...
    'AVLC:GlobalPreservationArithmeticFailure');
s.theoryParameters.maxLateralAccel=realmax;z.w=1;
t.verifyError(@() avlc.analysis.evaluateLateralEndpointReachability(s,z,2,1), ...
    'AVLC:LateralReachabilityArithmeticFailure');
end

function verifyUndefinedEndpoints(t,r)
t.verifyTrue(all(isnan([r.sigma,r.wT,r.yBarT,r.betaMax,r.yMinus,r.yPlus])));
end

function values=lateralNumbers(r)
values=[r.rhoLower,r.rhoUpper,r.sigma,r.wT,r.yBarT,r.betaMax,r.yMinus,r.yPlus];
end

function verifyEquivalent(t,s,z)
m=avlc.analysis.evaluateGlobalPreservationMembership(s,z);
cap=avlc.analysis.evaluateCapability(s,z);
t.verifyEqual([m.abort.inSet,m.completion.inSet], ...
    [cap.abort.feasible,cap.completion.feasible]);
end

function [s,z]=fixture()
roles={'EGO','FO','RO','RT'};v=cell(1,4);
for k=1:4
    v{k}=avlc.model.createVehicle(string(k),roles{k},4,2, ...
        avlc.model.createVehicleState(0,0,0,0,0,0));
end
s=avlc.scenario.createScenario("global foundations test","",avlc.model.createRoad(4,[-100,100]), ...
    avlc.model.createTheoryModelParameters(2,4,2,1,0,0,0),v{:});
z=struct('y',1,'w',2,'gR',8,'cR',2,'gF',8,'cF',2,'gT',6,'cT',5);
end
