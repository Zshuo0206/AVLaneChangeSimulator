function tests = testFixedHorizonFeasibility
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

function testZeroHorizonNeedsOnlyGlobalMembership(t)
[s,z]=canonical();z.cR=0;z.cF=0;z.cT=0;
guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0);
verifyCertified(t,d,s,z,0);
t.verifyEqual(d.reason,"ZERO_DURATION_GLOBAL_MEMBERSHIP");
t.verifyEmpty(fieldnames(d.longitudinalSearch));t.verifyEqual(d.rhoSearch.numCandidates,0);
t.verifyEmpty(fieldnames(d.domain.p5bLift));t.verifyEmpty(fieldnames(d.initial.capability));
t.verifyEqual(mockData().calls,0);
end

function testZeroHorizonOutsideGlobalSet(t)
[s,z]=canonical();z.gR=-1;guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0);
verifyNegative(t,d,"INITIAL_GLOBAL_NONMEMBERSHIP");
t.verifyEqual(d.reason,"INITIAL_STATE_OUTSIDE_GLOBAL_PRESERVATION_SET");
t.verifyEqual(d.rhoSearch.numCandidates,0);t.verifyEqual(mockData().calls,0);
end

function testPositiveInitialGlobalNonmembershipNeverSearches(t)
[s,z]=canonical();guard=mockSearch(struct(),true); %#ok<NASGU>
for field={'y','w','gR','gF','gT'}
    changed=z;changed.(field{1})=-1;
    d=avlc.analysis.evaluateFixedHorizonFeasibility(s,changed,1/32);
    verifyNegative(t,d,"INITIAL_GLOBAL_NONMEMBERSHIP");
    t.verifyEmpty(fieldnames(d.longitudinalSearch));
end
z.y=3; % Positive base state, but outside abort capability.
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
verifyNegative(t,d,"INITIAL_GLOBAL_NONMEMBERSHIP");t.verifyEqual(mockData().calls,0);
end

function testInitialTypeLIsExactNegativeBeforeLiftOrSearch(t)
s=scenario(2,4,2,4.5);z=struct('y',1,'w',2,'gR',6,'cR',2,'gF',6,'cF',2,'gT',6,'cT',5);
guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0.1);
verifyNegative(t,d,"INITIAL_LOCAL_INCOMPATIBILITY");
t.verifyTrue(d.initial.membership.preservation.inSet);
t.verifyTrue(d.initial.localCompatibility.defined);
t.verifyTrue(d.initial.localCompatibility.classification.typeL);
t.verifyEqual(d.reason,"INITIAL_LOCAL_SHARED_CONTROL_INCOMPATIBLE");
t.verifyEmpty(fieldnames(d.domain.p5bLift));t.verifyEqual(mockData().calls,0);
end

function testInitialTypeXIsExactNegative(t)
s=scenario(2,4,2,4);z=struct('y',3.5,'w',2,'gR',100,'cR',-21,'gF',48,'cF',20,'gT',6,'cT',5);
guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0.1);
verifyNegative(t,d,"INITIAL_LOCAL_INCOMPATIBILITY");
t.verifyTrue(d.initial.localCompatibility.classification.typeX);
t.verifyEqual(mockData().calls,0);
end

function testInfiniteRelevantClockIsOutsideDomain(t)
[s,z]=canonical();z.cR=0;guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
verifyOutside(t,d);t.verifyFalse(d.domain.p5bLift.finiteTR);
t.verifyTrue(d.domain.p5bLift.cOPositive);
t.verifyEqual(d.reason,"INFINITE_INITIAL_OPPORTUNITY");t.verifyEqual(mockData().calls,0);
end

function testStoppingDiscriminantEqualityIsOutsideDomain(t)
[s,z]=canonical();z.gR=4; % cR^2=2*C*gR=16, so frozen TRsafe=Inf.
guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
verifyOutside(t,d);t.verifyEqual(d.initial.capability.abort.TRsafe,Inf);
t.verifyFalse(d.domain.p5bLift.strictDiscriminants);
t.verifyTrue(isnan(d.domain.p5bLift.deltaR));t.verifyEqual(mockData().calls,0);
end

function testNonpositiveCOIsOutsideDomain(t)
[s,z]=canonical();z.cR=-4;guard=mockSearch(struct(),true); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
verifyOutside(t,d);t.verifyEqual(d.domain.p5bLift.cO0,0);
t.verifyFalse(d.domain.p5bLift.cOPositive);
t.verifyEqual(d.reason,"NONPOSITIVE_INITIAL_CO");t.verifyEqual(mockData().calls,0);
end

function testUndefinedGF2DoesNotExcludeLift(t)
requireSolver(t);[s,z]=canonical();z.y=0;
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
t.verifyEqual(d.initial.localCompatibility.status,"UNDEFINED");
t.verifyTrue(d.domain.p5bLift.applicable);t.verifyFalse(d.infeasibleCertified);
t.verifyEqual(d.longitudinalSearch.status,"WITNESS_FOUND");
t.verifyGreaterThan(d.rhoSearch.numCandidates,0);
end

function testRealCanonicalPositiveCertificate(t)
requireSolver(t);[s,z]=canonical();T=1/32;
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T);
verifyCertified(t,d,s,z,T);c=d.feasibilityCertificate;p=d.domain.p5bLift;
t.verifyEqual([d.initial.capability.abort.TRsafe,d.initial.capability.abort.TFsafe, ...
    d.initial.capability.abort.TG,d.initial.capability.completion.HT],[1,1,0.75,1]);
t.verifyEqual([p.deltaF,p.deltaR,p.deltaT,p.cO0],[2,2,2,8]);
t.verifyTrue(p.applicable && p.controlBoundsFrozen && p.baseDefinitionFrozen);
t.verifyEqual(c.longitudinalWitness.ax,-2*ones(8,1));
t.verifyEqual([c.longitudinalCertificate.moments.eta,c.longitudinalCertificate.moments.zeta],[0,0]);
t.verifyEqual([c.rho,c.reachability.wT,c.reachability.yMinus,c.reachability.yPlus], ...
    [0.25,0,1,1+1/256]);
t.verifyEqual([c.YO,c.YTout,c.Yfwd],[529/256,5,529/256]);
t.verifyEqual(c.yTerminal,1);t.verifyEqual(c.betaTerminal,1/256);
t.verifyEqual(d.rhoSearch.rhoValues,linspace(0.25,0.5,33).');
t.verifyEqual(d.rhoSearch.numPlannedCandidates,33);t.verifyEqual(d.rhoSearch.numCandidates,1);
end

function testLongitudinalFailureRemainsUnresolved(t)
requireSolver(t);[s,z]=canonical();
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1,struct('numSegments',1));
verifyUnresolved(t,d);t.verifyEqual(d.reason,"LONGITUDINAL_SEARCH_UNRESOLVED");
t.verifyEqual(d.longitudinalSearch.status,"UNRESOLVED");
t.verifyEqual(d.rhoSearch.numCandidates,0);
end

function testRealRhoGridMissIsNotAnExactNegative(t)
requireSolver(t);[s,z]=canonical();
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/8);
verifyUnresolved(t,d);t.verifyEqual(d.longitudinalSearch.status,"WITNESS_FOUND");
t.verifyEqual(d.reason,"NO_CERTIFIED_JOINT_CANDIDATE");
t.verifyEqual(d.rhoSearch.numCandidates,33);
t.verifyFalse(any([d.rhoSearch.candidates.overlap]));verifyMargins(t,d);
end

function testMissingTerminalGeometryKeepsNaNSentinels(t)
requireSolver(t);[s,z]=canonical();
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/2);
verifyUnresolved(t,d);t.verifyEqual(d.longitudinalSearch.status,"WITNESS_FOUND");
bad=d.rhoSearch.candidates(~[d.rhoSearch.candidates.geometryAvailable]);
t.assertNotEmpty(bad);
for k=1:numel(bad)
    t.verifyEqual(bad(k).reason,"FINITE_TERMINAL_GEOMETRY_UNAVAILABLE");
    t.verifyFalse(bad(k).overlap);
    t.verifyTrue(all(isnan([bad(k).M_corr,bad(k).M_dead,bad(k).M_fwd,bad(k).phiDual])));
end
end

function testDeadlineContactAndBinarySides(t)
for offset=[-2^-35,0,2^-35]
    [s,z,T,w]=endpointFixture('deadline',offset);
    guard=mockSearch(certifiedLongitudinal(s,z,T,w),false); %#ok<NASGU>
    d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T);
    if offset<0
        verifyUnresolved(t,d);t.verifyEqual(d.reason,"NO_CERTIFIED_JOINT_CANDIDATE");
    else
        verifyCertified(t,d,s,z,T);
        t.verifyEqual(d.feasibilityCertificate.M_dead,offset);
        if offset==0,t.verifyEqual(d.feasibilityCertificate.phiDual,0);end
    end
    verifyMargins(t,d);clear guard;
end
end

function testForwardContactAndBinarySides(t)
for offset=[-2^-35,0,2^-35]
    [s,z,T,w]=endpointFixture('forward',offset);
    guard=mockSearch(certifiedLongitudinal(s,z,T,w),false); %#ok<NASGU>
    d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T);
    if offset<0
        verifyUnresolved(t,d);t.verifyEqual(d.reason,"NO_CERTIFIED_JOINT_CANDIDATE");
    else
        verifyCertified(t,d,s,z,T);t.verifyEqual(d.feasibilityCertificate.M_fwd,offset);
        if offset==0,t.verifyEqual(d.feasibilityCertificate.phiDual,0);end
    end
    verifyMargins(t,d);clear guard;
end
end

function testCorridorCollapseAndBinarySides(t)
for offset=[-2^-35,0,2^-35]
    [s,z,T,w]=endpointFixture('collapse',offset);
    guard=mockSearch(certifiedLongitudinal(s,z,T,w),false); %#ok<NASGU>
    d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T);
    if offset<0
        verifyUnresolved(t,d);t.verifyEqual(d.reason,"NO_CERTIFIED_JOINT_CANDIDATE");
    else
        verifyCertified(t,d,s,z,T);t.verifyEqual(d.feasibilityCertificate.M_corr,offset);
        if offset==0,t.verifyEqual(d.feasibilityCertificate.YTin,d.feasibilityCertificate.Yfwd);end
    end
    verifyMargins(t,d);clear guard;
end
end

function testProbeCapabilityBooleanDoesNotDecideTerminalPoint(t)
[s,z,T,w]=endpointFixture('probe',0);
guard=mockSearch(certifiedLongitudinal(s,z,T,w),false); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T);verifyCertified(t,d,s,z,T);
a=d.rhoSearch.candidates(end);
t.verifyLessThan(a.probeState.y,a.terminalState.y);
t.verifyFalse(a.probeCapability.joint.dualCapable);
t.verifyTrue(a.terminalCapability.joint.dualCapable);
t.verifyEqual([a.probeCapability.abort.YO,a.probeCapability.completion.YTin, ...
    a.probeCapability.completion.YTout,a.probeCapability.joint.Yfwd], ...
    [a.terminalCapability.abort.YO,a.terminalCapability.completion.YTin, ...
    a.terminalCapability.completion.YTout,a.terminalCapability.joint.Yfwd]);
end

function testInconsistentLongitudinalSuccessNeverCertifies(t)
[s,z]=canonical();fake=struct('status',"WITNESS_FOUND", ...
    'certificate',struct('validWitness',false,'status',"UNSAFE_WITNESS"));
guard=mockSearch(fake,false); %#ok<NASGU>
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32);
verifyUnresolved(t,d);t.verifyEqual(d.reason,"LONGITUDINAL_CERTIFICATE_INCONSISTENT");
t.verifyEqual(d.rhoSearch.numCandidates,0);t.verifyEqual(mockData().calls,1);
end

function testOptionsForwardedOnceAndInputsUnchanged(t)
[s,z]=canonical();T=single(1/32);options=struct('numSegments',uint8(3), ...
    'maxCutIterations',uint8(7),'numRhoSamples',uint8(9));
w=struct('times',linspace(0,double(T),4),'ax',[0,0,0]);
guard=mockSearch(certifiedLongitudinal(s,z,T,w),false); %#ok<NASGU>
before={s,z,T,options};d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,T,options);
verifyCertified(t,d,s,z,T);t.verifyEqual({s,z,T,options},before);
data=mockData();t.verifyEqual(data.calls,1);
t.verifyEqual(data.options,rmfield(options,'numRhoSamples'));
t.verifyEqual(d.rhoSearch.numPlannedCandidates,9);
end

function testMalformedRhoOptionsAndRequest(t)
[s,z]=canonical();id='AVLC:InvalidFixedHorizonFeasibilityInput';
for value={1,0,-1,2.5,NaN,Inf,1i,true,'3',[],[2,3],struct()}
    t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0, ...
        struct('numRhoSamples',value{1})),id);
end
for value={-1,NaN,Inf,1i,true,'1',[],[1,2]}
    t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,z,value{1}),id);
end
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0,[]),id);
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0,struct('rho',1)),id);
end

function testForwardedValidationUsesFrozenSearcher(t)
[s,z]=canonical();id='AVLC:InvalidFixedHorizonFeasibilityInput';
for options={struct('numSegments',0),struct('maxCutIterations',-1)}
    t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,z,1/32,options{1}),id);
end
end

function testMalformedConsumedScenarioAndState(t)
[s,z]=canonical();id='AVLC:InvalidFixedHorizonFeasibilityInput';
for field={'y','w','gR','cR','gF','cF','gT','cT'}
    for value={NaN,Inf,1i,true,'1',[],[1,2]}
        changed=z;changed.(field{1})=value{1};
        t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,changed,0),id);
    end
end
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(struct(),z,0),id);
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(s,rmfield(z,'gT'),0),id);
changed=s;changed.theoryParameters.maxLateralAccel=0;
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(changed,z,0),id);
changed=s;changed.vehicles=NaN;
t.verifyError(@() avlc.analysis.evaluateFixedHorizonFeasibility(changed,z,1/32),id);
end

function testArithmeticFailureIsNotAnExactNegative(t)
[s,z]=canonical();z.w=realmax;
d=avlc.analysis.evaluateFixedHorizonFeasibility(s,z,0);
verifyUnresolved(t,d);
t.verifyEqual(d.reason,"INITIAL_GLOBAL_MEMBERSHIP_ARITHMETIC_FAILURE");
t.verifyEqual(d.numerical.exceptionIdentifier,"AVLC:GlobalPreservationArithmeticFailure");
end

function verifyCertified(t,d,s,z,T)
t.assertEqual(d.status,"CERTIFIED_FEASIBLE");t.verifyTrue(d.feasibleCertified);
t.verifyFalse(d.infeasibleCertified);t.verifyEmpty(fieldnames(d.infeasibilityCertificate));
c=d.feasibilityCertificate;
freshMembership=avlc.analysis.evaluateGlobalPreservationMembership(s,c.terminalState);
freshCap=avlc.analysis.evaluateCapability(s,c.terminalState);
t.verifyTrue(freshMembership.preservation.inSet);t.verifyTrue(freshCap.joint.dualCapable);
t.verifyEqual(freshMembership,c.terminalMembership);
if T==0
    w=struct('times',0,'ax',[]);
    freshLong=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,w);
    freshReach=avlc.analysis.evaluateLateralEndpointReachability(s,z,T,0);
    t.verifyTrue(freshLong.validWitness);t.verifyTrue(freshReach.rhoAdmissible);
else
    freshLong=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,c.longitudinalWitness);
    freshReach=avlc.analysis.evaluateLateralEndpointReachability(s,z,T,c.rho);
    t.verifyEqual(freshLong,c.longitudinalCertificate);t.verifyTrue(freshLong.validWitness);
    t.verifyEqual(freshReach,c.reachability);t.verifyEqual(freshCap,c.terminalCapability);
    t.verifyTrue(all(cell2mat(struct2cell(c.theorem))));
    t.verifyGreaterThanOrEqual(c.yTerminal,freshReach.yMinus);
    t.verifyLessThanOrEqual(c.yTerminal,freshReach.yPlus);
    t.verifyEqual(c.betaTerminal,freshReach.yBarT-c.yTerminal);
    verifyMargins(t,d);
end
end

function verifyNegative(t,d,type)
t.verifyEqual(d.status,"INFEASIBILITY_CERTIFIED");t.verifyTrue(d.infeasibleCertified);
t.verifyFalse(d.feasibleCertified);t.verifyEqual(d.infeasibilityCertificate.type,type);
t.verifyEmpty(fieldnames(d.feasibilityCertificate));
end

function verifyOutside(t,d)
t.verifyEqual(d.status,"OUTSIDE_PROVED_DOMAIN");t.verifyFalse(d.feasibleCertified);
t.verifyFalse(d.infeasibleCertified);t.verifyFalse(d.domain.p5bLift.applicable);
t.verifyEmpty(fieldnames(d.longitudinalSearch));
t.verifyEmpty(fieldnames(d.feasibilityCertificate));t.verifyEmpty(fieldnames(d.infeasibilityCertificate));
end

function verifyUnresolved(t,d)
t.verifyEqual(d.status,"UNRESOLVED");t.verifyFalse(d.feasibleCertified);
t.verifyFalse(d.infeasibleCertified);t.verifyEmpty(fieldnames(d.feasibilityCertificate));
t.verifyEmpty(fieldnames(d.infeasibilityCertificate));
end

function verifyMargins(t,d)
for k=1:numel(d.rhoSearch.candidates)
    a=d.rhoSearch.candidates(k);
    if a.geometryAvailable
        literal=a.M_corr>=0 && a.M_dead>=0 && a.M_fwd>=0;
        t.verifyEqual(a.overlap,literal);t.verifyEqual(a.overlap,a.phiDual>=0);
        t.verifyEqual(a.minMargin,min([a.M_corr,a.M_dead,a.M_fwd]));
    end
end
end

function requireSolver(t)
t.assumeTrue(t.TestData.hasSolver,'Real LP integration requires Optimization Toolbox.');
end

function r=certifiedLongitudinal(s,z,T,w)
c=avlc.analysis.verifyLongitudinalSafePrefixWitness(s,z,T,w);assert(c.validWitness);
r=struct('status',"WITNESS_FOUND",'witnessFound',true,'reason',"CONTROLLED_TEST_WITNESS", ...
    'witness',struct('times',c.control.times,'ax',c.control.ax),'certificate',c);
end

function data=mockData()
data=getappdata(groot,'AVLC_P033_MOCK_SEARCH');
end

function cleanup=mockSearch(result,poison)
% Controlled outputs isolate endpoint certificates; the real verifier is never mocked.
key='AVLC_P033_MOCK_SEARCH';had=isappdata(groot,key);previous=[];
if had,previous=getappdata(groot,key);end
oldPath=path;folder=tempname;package=fullfile(folder,'+avlc','+analysis');mkdir(package);
cleanup=onCleanup(@() removeMock(folder,oldPath,key,had,previous));
setappdata(groot,key,struct('result',result,'poison',poison,'calls',0,'options',struct()));
file=fullfile(package,'searchLongitudinalSafePrefixWitness.m');
lines=["function r=searchLongitudinalSafePrefixWitness(s,z,T,options)"; ...
    "key='AVLC_P033_MOCK_SEARCH';data=getappdata(groot,key);"; ...
    "data.calls=data.calls+1;data.options=options;setappdata(groot,key,data);"; ...
    "if data.poison,error('AVLC:UnexpectedSearch','This path must not call the searcher');end"; ...
    "r=data.result;";"end"];
fid=fopen(file,'w');assert(fid>=0);fcloseGuard=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(lines,newline));clear fcloseGuard;
addpath(folder,'-begin');clear('avlc.analysis.searchLongitudinalSafePrefixWitness');
assert(strcmp(which('avlc.analysis.searchLongitudinalSafePrefixWitness'),file));
end

function removeMock(folder,oldPath,key,had,previous)
path(oldPath);clear('avlc.analysis.searchLongitudinalSafePrefixWitness');
if had,setappdata(groot,key,previous);elseif isappdata(groot,key),rmappdata(groot,key);end
file=fullfile(folder,'+avlc','+analysis','searchLongitudinalSafePrefixWitness.m');
if isfile(file),delete(file);end
for directory={fullfile(folder,'+avlc','+analysis'),fullfile(folder,'+avlc'),folder}
    if isfolder(directory{1}),rmdir(directory{1});end
end
end

function [s,z,T,w]=endpointFixture(kind,offset)
T=1/32;A=2;W=4;w0=0;cO=20;gO=96;cT=5;gT=6;
switch kind
    case 'deadline',y0=3967/2048+offset;
    case 'forward',w0=1/32;y0=2-1/4096-offset;cO=10;gO=20;cT=8;gT=15;
    case 'collapse',y0=2-1/4096;cO=10;gO=20;W=4-offset;
    case 'probe',y0=2-1/4096;
end
s=scenario(A,2,2,W);
z=struct('y',y0,'w',w0,'gR',gO+cO*T,'cR',cO, ...
    'gF',gO+cO*T,'cF',cO,'gT',gT+cT*T,'cT',cT);
w=struct('times',[0,T],'ax',0);
end

function [s,z]=canonical()
s=scenario(16,2,2,4);
z=struct('y',1,'w',0,'gR',3,'cR',4,'gF',3,'cF',4,'gT',3,'cT',4);
end

function s=scenario(A,B,C,W)
roles={'EGO','FO','RO','RT'};v=cell(1,4);
for k=1:4
    v{k}=avlc.model.createVehicle(string(k),roles{k},4,2,avlc.model.createVehicleState(0,0,0,0,0,0));
end
s=avlc.scenario.createScenario("P033 test","",avlc.model.createRoad(W,[-100,100]), ...
    avlc.model.createTheoryModelParameters(A,B,C,1,0,0,0),v{:});
end
