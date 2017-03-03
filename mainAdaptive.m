clear all; close all;
weight = [1,1,5,10];
weight = weight/sum(weight);
generateSignal([2e3,7e3,13e3,18e3],weight,7);
autoArrangeFigures;
%%
clear all; close all;
load('RunoutData.mat');
load('AccExcData.mat');
load('Model.mat');
load('FilterBank.mat');
%%
clear AA;
AA = aafc(10e4);
FB_VCM = {F7{1}};
FB_MA = {F7{3},F7{5},F7{7}};
AA.initialize(VCM,MA,FB_VCM,FB_MA,Acc,Dist,RRO,NRRO,2*Exc_VCM,10*Exc_MA);
AA.initializeOrder(5,5,6,6,5,5,6,6);
AA.initializeSGD(2e-3,2e-3,0.98,1,500);
% AA.initializeRLS({1e8},{1e6},{1-1e-5},{1-1e-4},{1e8,1e8,1e8},{1e6,1e6,1e8},{1-1e-5,1-1e-5,1-1e-5},{1-1e-4,1-1e-4,1-1e-4});
%%
% AA.filterSignal(1,1);
AA.optimalD_Dual;
AA.ThetaD_Optimal_VCM
AA.ThetaD_Optimal_MA
AA.optimalControl;
AA.plotMinPES;
%%
AA.adaptiveControlNLMS;
AA.ThetaD_VCM
AA.ThetaD_MA
%%
AA.plotPES;
AA.plotVCM(Bode_Opt);
AA.plotMA(Bode_Opt);
%%
% AA.plotAcc;
% AA.plotDist;
%%
% AA.plotVCM(Bode_Opt);
% AA.plotMA(Bode_Opt);
%%
AA.plotTM_VCM;
AA.plotTD_VCM;
%%
AA.plotTM_MA;
AA.plotTD_MA;
autoArrangeFigures;