close all; clear all;
N = 2e6; % simulation step
Fs = 348*120;
Ts = 1/Fs;
% system
load('Model.mat');
% NRRO and RRO signal
load('RunoutData.mat');
%% generate disturbance vibration
rng(0);
v1 = randn(1,N);
% f = [2e3,3e3,4e3]; % low frequency
f = [2e3,7.5e3];
noiseModel = tf(1);
for i = 1:1:length(f)
    Fpeak = f(i);  % Peak Frequency
    Q = 20;        % Quality factor
    BW =  Fpeak/Q;
    Apass = 5;     % Bandwidth Attenuation
    [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
    % fvtool(b, a) % uncomment to see the peak filter response
    f1 = tf(b,a,-1);
    noiseModel = noiseModel*f1;
end
noiseModel.Ts = Ts;
vib1 = lsim(noiseModel,v1);
vib1 = vib1/std(vib1)*0.5;
%
% f = [14e3,15e3]; % high frequency
f = [12.5e3,17e3];
noiseModel = tf(1);
for i = 1:1:length(f)
    % 1st narrow-band content
    Fpeak = f(i);  % Peak Frequency
    Q = 20;        % Quality factor
    BW =  Fpeak/Q;
    Apass = 5;     % Bandwidth Attenuation
    [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
    % fvtool(b, a) % uncomment to see the peak filter response
    f1 = tf(b,a,-1);
    noiseModel = noiseModel*f1;
end
noiseModel.Ts = Ts;
vib2 = lsim(noiseModel,v1);
vib2 = vib2/std(vib2)*0.5;

vib = vib1+5*vib2;
Dist = lsim(TF_vib2pes,vib)';
Acc = lsim(TF_vib2acc,vib)';

figure; fftp(Dist,Fs);
figure; fftp(Acc,Fs);
% generate excitation signal
rng(0); w = 1;
ue = w*randn(N,1);
%% bandpass filter
figure;
H3 = filter_bank(3,4);
% Exc_VCM = filter(H3(:,1),1,ue)';
% Exc_MA = filter(H3(:,3),1,ue)';
A = {H3(:,1)',H3(:,2)',H3(:,3)'};
figure;
H4 = filter_bank(4,4);
B = {H4(:,1)',H4(:,2)',H4(:,3)',H4(:,4)'};
figure;
H7 = filter_bank(7,4);
C = {H7(:,1)',H7(:,2)',H7(:,3)',H7(:,4)',H7(:,5)',H7(:,6)',H7(:,7)'};
% F = filter_bank(7);
% FB_VCM = {F(:,1)',F(:,3)'};
% FB_MA = {F(:,5)',F(:,7)'};
%% filtered excitation
Exc_VCM = filter(H7(:,1),1,ue)';
Exc_MA = filter(H7(:,3),1,ue)'+filter(H7(:,5),1,ue)'+filter(H7(:,7),1,ue)';
figure;fftp(Exc_VCM,Fs);
figure;fftp(Exc_MA,Fs);
%%
save('AccExcData.mat','Dist','Acc','Exc_VCM','Exc_MA');
save('FilterBank.mat','A','B','C');










