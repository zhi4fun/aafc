clear all;
%%
close all;
%%
load('RunoutData.mat');
load('AccExcData.mat');
load('Model.mat');
load('FilterBank.mat');
%%
clear AA;
AA = aafc(10e4);
FB_VCM = {F5{1}};
% FB_VCM = {F3{1}};
% FB_MA = {C{3}};
FB_MA = {F5{3},F5{5}};
% FB_MA = {F3{3}};
AA.initializeRegressor(5,5,6,6,5,5,6,6);
%AA.initializeGD(1e-3,1e-2,1,1,2000);
AA.initialize(VCM,MA,FB_VCM,FB_MA,Acc,Dist,RRO,NRRO,Exc_VCM,8*Exc_MA);
AA.initializeSGD(1.5e-4,2e-3,0.98,0.98,500);
% AA.initializeRLS({1e8},{1e6},{1-1e-5},{1-1e-4},{1e8,1e8,1e8},{1e6,1e6,1e8},{1-1e-5,1-1e-5,1-1e-5},{1-1e-4,1-1e-4,1-1e-4});
% AA.adaptiveControlRLS;
AA.adaptiveControlNLMS;
%%
AA.plotPES;
%%
% AA.plotAcc;
% AA.plotDist;
%%
AA.plotVCM(Bode_Opt);
AA.plotMA(Bode_Opt);
%%
AA.plotTM_VCM;
AA.plotTD_VCM;
%%
AA.plotTM_MA;
AA.plotTD_MA;
autoArrangeFigures;