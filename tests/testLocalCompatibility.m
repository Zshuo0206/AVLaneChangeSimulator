function tests = testLocalCompatibility
tests = functiontests(localfunctions);
end
function setupOnce(t)
t.TestData.oldPath=path;
addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'src'));
end
function teardownOnce(t)
path(t.TestData.oldPath);
end

function testRegularInterior(t)
[s,z,cap,d]=fixture();
c=avlc.analysis.evaluateLocalCompatibility(s,z,cap,d);
t.verifyEqual(c.status,"PRESERVABLE");
t.verifyEqual(c.domainCase,"REGULAR_W_POSITIVE");
t.verifyEqual(bounds(c),[-4,2,-2,2]);
t.verifyFalse(any(struct2arrayLocal(c.activeBoundaries)));
end

function testPrimitiveAbortAndSwitchSignatures(t)
for branch=["F","G","R","FG","RG"]
 [s,z,~,~]=fixture();
 s.road=avlc.model.createRoad(5,[-100,100]); z.y=3.5;
 z=setAbort(z,branch);
 [c,~,~]=evaluate(s,z);
 expected=[-4,2,-2,-2];
 if any(branch==["F","FG"]), expected(2)=-4; end
 if any(branch==["R","RG"]), expected(1)=2; end
 t.verifyEqual(c.status,"PRESERVABLE"); t.verifyEqual(bounds(c),expected);
 names=["F_STRICT","G_STRICT","R_STRICT","FG_SWITCH","RG_SWITCH"];
 t.verifyEqual(c.boundaryCases.abortCase,names(find(["F","G","R","FG","RG"]==branch)));
end
end

function testCompletionSingleBoundaries(t)
[s,z]=fixture(); z.y=0.5;
[c,~,~]=evaluate(s,z); t.verifyEqual(bounds(c),[2,2,2,2]);
for delta=[1,0.5,0.25]
 [s,z]=fixture(); z.gR=10;z.gF=10;
 s.theoryParameters.targetOvershoot=delta;
 z.y=3.5; if delta==0.25, z.y=3.25; end
 [c,~,~]=evaluate(s,z);
 expected=[2,2,-2,-2]; if delta==0.25, expected(1)=-4; end
 t.verifyEqual(c.status,"PRESERVABLE");t.verifyEqual(bounds(c),expected);
 if delta==0.5,t.verifyEqual(c.boundaryCases.upperCase,"I_II_SWITCH");end
end
end

function testTypeLAnchor(t)
[s,z]=fixture();s.road=avlc.model.createRoad(4.5,[-100,100]);
z.y=1;z.gR=6;z.gF=6;
[c,cap,d]=evaluate(s,z);
t.verifyTrue(cap.joint.dualCapable);t.verifyEqual([d.reserves.rO,d.reserves.lT,d.reserves.rT],[0,0,3]);
t.verifyEqual(c.status,"NOT_PRESERVABLE");t.verifyEqual(bounds(c),[2,2,2,-2]);
t.verifyTrue(c.classification.typeL);t.verifyFalse(c.classification.typeX);
end

function testTypeXAnchor(t)
[s,z]=fixture();z.y=3.5;z=setAbort(z,"F");
[c,cap,d]=evaluate(s,z);
t.verifyTrue(cap.joint.dualCapable);t.verifyEqual([d.reserves.rO,d.reserves.lT,d.reserves.rT],[0,3,0]);
t.verifyEqual(c.status,"NOT_PRESERVABLE");t.verifyEqual(bounds(c),[2,-4,-2,-2]);
t.verifyTrue(c.classification.typeX);t.verifyFalse(c.classification.typeL);
end

function testCompatibleSharedSignaturesAndSwitches(t)
for branch=["G","R","RG","F","FG"]
 for delta=[1,0.25,0.5]
  [s,z]=fixture();z.y=3.5;z=setAbort(z,branch);
  s.theoryParameters.targetOvershoot=delta;
  if delta==0.25,s.road=avlc.model.createRoad(4.25,[-100,100]);end
  [c,cap,~]=evaluate(s,z);
  t.verifyTrue(cap.joint.dualCapable);t.verifyTrue(c.defined);
  conflict=any(branch==["F","FG"]) && delta~=0.25;
  t.verifyEqual(c.localPreservable,~conflict);
  if ~conflict
   expected=[-4,2,-2,-2];
   if any(branch==["F","FG"]),expected(2)=-4;
   elseif any(branch==["R","RG"]) || delta~=0.25,expected(1)=2;end
   t.verifyEqual(bounds(c),expected);
  end
 end
end
end

function testDegenerateLowerUpperAndTripleAreUndefined(t)
for W=[4,4.5]
 [s,z]=fixture();s.road=avlc.model.createRoad(W,[-100,100]);z.gT=4;z.y=W-1;
 [c,cap,d]=evaluate(s,z);
 t.verifyTrue(cap.joint.dualCapable);t.verifyEqual([d.reserves.lT,d.reserves.rT],[0,0]);
 t.verifyEqual(c.status,"UNDEFINED");t.verifyFalse(c.domainChecks.strictPQ);
 t.verifyTrue(isnan(c.localPreservable));t.verifyFalse(c.defined);
 t.verifyFalse(c.classification.typeL || c.classification.typeX);
end
end

function testWZeroClassificationsAndSimultaneousRequirements(t)
% W, y, overshoot, expected bounds; NaN denotes the empty intersection.
cases=[4,3,1,-4,2,0,2;9,8,1,NaN,NaN,NaN,NaN;4,2,1,2,2,2,2; ...
 4,6,3,NaN,NaN,NaN,NaN;4,6,2,NaN,NaN,NaN,NaN;4,5,1,-4,2,0,0; ...
 7,8,1,NaN,NaN,NaN,NaN];
for k=1:size(cases,1)
 [s,z]=fixture();s.road=avlc.model.createRoad(cases(k,1),[-100,100]);
 z.w=0;z.y=cases(k,2);s.theoryParameters.targetOvershoot=cases(k,3);
 [c,cap,~]=evaluate(s,z);
 t.verifyTrue(cap.joint.dualCapable);t.verifyTrue(c.defined);
 t.verifyEqual(c.domainCase,"PROGRESSIVE_W_ZERO");
 if isnan(cases(k,4)),t.verifyEqual(c.status,"NOT_PRESERVABLE");t.verifyFalse(c.intersection.nonempty);
 else,t.verifyEqual(c.status,"PRESERVABLE");t.verifyEqual(bounds(c),cases(k,4:7));end
end
end

function testUndefinedCasesAreNotProvedIncompatible(t)
[s,z]=fixture();
states={z,z,z,z};
states{1}.cT=0; % Infinite opportunity, undefined finite target frontier.
states{2}.gT=4; % HT=w/A, pT=0.
states{3}.gR=0;states{3}.gF=0;states{3}.w=0;states{3}.y=0; % Zero-time triple.
states{4}.gT=6.25; % Target discriminant collapse / infinite branch.
for k=1:numel(states)
 [c,~,~]=evaluate(s,states{k});
 t.verifyEqual(c.status,"UNDEFINED");t.verifyFalse(c.defined);t.verifyTrue(isnan(c.localPreservable));
end
end

function testInconsistentFrontRearTopologyIsNotCertified(t)
[s,z,cap,d]=fixture();
d.abort.activeR=true;d.abort.activeF=true;d.abort.activeG=false;
c=avlc.analysis.evaluateLocalCompatibility(s,z,cap,d);
t.verifyEqual(c.status,"UNDEFINED");t.verifyFalse(c.domainChecks.branchConfigurationSupported);
end

function testExactBoundaryAndTwoSides(t)
[s,z]=fixture();
for offset=[-2^-35,0,2^-35]
 z.y=0.5+offset;
 [c,~,~]=evaluate(s,z);
 t.verifyEqual(c.activeBoundaries.lower,offset==0);
 if offset<0,t.verifyEqual(c.status,"UNDEFINED");
 elseif offset==0,t.verifyEqual(bounds(c),[2,2,2,2]);
 else,t.verifyEqual(bounds(c),[-4,2,-2,2]);end
end
end

function testInputsUnchangedAndMalformedError(t)
[s,z,cap,d]=fixture();before={s,z,cap,d};
avlc.analysis.evaluateLocalCompatibility(s,z,cap,d);t.verifyEqual({s,z,cap,d},before);
t.verifyError(@() avlc.analysis.evaluateLocalCompatibility(s,z,struct(),d),'AVLC:InvalidLocalCompatibilityInput');
end

function b=bounds(c)
b=[c.intersection.axLower,c.intersection.axUpper,c.intersection.ayLower,c.intersection.ayUpper];
end
function z=setAbort(z,branch)
switch branch
 case "F",z.gR=100;z.cR=-21;z.gF=48;z.cF=20;
 case "R",z.gR=24;z.cR=10;z.gF=100;z.cF=-11;
 case "G",z.gR=8;z.cR=2;z.gF=8;z.cF=2;
 case "FG",z.gR=40;z.cR=2;z.gF=48;z.cF=20;
 case "RG",z.gR=24;z.cR=10;z.gF=24;z.cF=2;
end
end
function [c,cap,d]=evaluate(s,z)
cap=avlc.analysis.evaluateCapability(s,z);d=avlc.analysis.evaluateCapabilityDiagnostics(z.y,cap);
c=avlc.analysis.evaluateLocalCompatibility(s,z,cap,d);
end
function v=struct2arrayLocal(s)
v=cell2mat(struct2cell(s));
end
function [s,z,cap,d]=fixture()
roles={'EGO','FO','RO','RT'};v=cell(1,4);
for k=1:4,v{k}=avlc.model.createVehicle(string(k),roles{k},4,2,avlc.model.createVehicleState(0,0,0,0,0,0));end
s=avlc.scenario.createScenario("GF2 test","",avlc.model.createRoad(4,[-100,100]), ...
 avlc.model.createTheoryModelParameters(2,4,2,1,0,0,0),v{:});
z=struct('y',1,'w',2,'gR',8,'cR',2,'gF',8,'cF',2,'gT',6,'cT',5);
cap=avlc.analysis.evaluateCapability(s,z);d=avlc.analysis.evaluateCapabilityDiagnostics(z.y,cap);
end
