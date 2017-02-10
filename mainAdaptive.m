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
AA = aafc(4e4);
FB_VCM = {C{1}};
% FB_MA = {C{3}};
FB_MA = {C{3},C{5},C{7}};
AA.initializeRegressor(5,5,6,6,5,5,10,10);
AA.initializeGD(1e-3,1e-2,1,1,2000);
AA.initialize(VCM,MA,FB_VCM,FB_MA,Acc,Dist,RRO,NRRO,Exc_VCM,8*Exc_MA);
% AA.initializeGD(6e-5,8e-5,1,1,2000);
% AA.adaptiveControlNormPhiBA();
% AA.initializeRLS({1e8},{1e6},{1-1e-5},{1-1e-4},{1e8,1e8,1e8},{1e6,1e6,1e8},{1-1e-5,1-1e-5,1-1e-5},{1-1e-4,1-1e-4,1-1e-4});
AA.adaptiveControlNormGD();
%%
AA.plotPES();
%%
AA.plotVCM(Bode_Opt);
AA.plotMA(Bode_Opt);
%%
AA.plotTM_VCM
AA.plotTD_VCM
%%
AA.plotTM_MA
AA.plotTD_MA
autoArrangeFigures();