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
% low frequency
f = [2e3];
for i = 1:1:length(f)
    Fpeak = f(i);  % Peak Frequency
    Q = 20;        % Quality factor
    BW =  Fpeak/Q;
    Apass = 5;     % Bandwidth Attenuation
    [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
    % fvtool(b, a) % uncomment to see the peak filter response
    tf1 = tf(b,a,-1);
end
tf1.Ts = Ts;
vib1 = lsim(tf1,v1);
vib1 = vib1/std(vib1)*0.5;
% middle frequency
f = [7e3];
for i = 1:1:length(f)
    Fpeak = f(i);  
    Q = 20;       
    BW =  Fpeak/Q;
    Apass = 5;     
    [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
    tf2 = tf(b,a,-1);
end
tf2.Ts = Ts;
vib2 = lsim(tf2,v1);
vib2 = vib2/std(vib2)*0.5;
% high frequency
f = [12e3,17.5e3];
for i = 1:1:length(f)
    Fpeak = f(i);  
    Q = 100;      
    BW =  Fpeak/Q;
    Apass = 5;     
    [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
    tf3 = tf(b,a,-1);
end
tf3.Ts = Ts;
vib3 = lsim(tf3,v1);
vib3 = vib3/std(vib3)*0.5;

vib = vib1+2*vib2;%+10*vib3;
% vib = vib3;
vib = vib/3;
% vib = vib1+3*vib3;
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
F3 = {H3(:,1)',H3(:,2)',H3(:,3)'};
figure;
H4 = filter_bank(4,4);
F4 = {H4(:,1)',H4(:,2)',H4(:,3)',H4(:,4)'};
figure;
H5 = filter_bank(5,4);
F5 = {H5(:,1)',H5(:,2)',H5(:,3)',H5(:,4)',H5(:,5)'};
figure;
H7 = filter_bank(7,4);
F7 = {H7(:,1)',H7(:,2)',H7(:,3)',H7(:,4)',H7(:,5)',H7(:,6)',H7(:,7)'};
% F = filter_bank(7);
% FB_VCM = {F(:,1)',F(:,3)'};
% FB_MA = {F(:,5)',F(:,7)'};
%% filtered excitation
Exc_VCM = filter(H7(:,1),1,ue)';
Exc_MA = filter(H7(:,3),1,ue)';%+filter(H7(:,5),1,ue)'+5*filter(H7(:,7),1,ue)';
% Exc_VCM = filter(H3(:,1),1,ue)';
% Exc_MA = filter(H3(:,3),1,ue)';
figure;fftp(Exc_VCM,Fs);
figure;fftp(Exc_MA,Fs);
%%
save('AccExcData.mat','Dist','Acc','Exc_VCM','Exc_MA');
save('FilterBank.mat','F3','F4','F5','F7');










