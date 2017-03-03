function generateSignal(f,weight,n)
    % F is the peak frequency vector, WEIGHT is the coefficient vector
    % N indicates the filter class
    if length(f) ~= length(weight)
        disp('The number of peaks and weights are not equal.');
        return;
    end
    if ismember(n,[3,4,5,7]) == 0
        disp('No such filter.');
        return;
    end
    N = 1e6; % total steps
    Fs = 348*120;
    Ts = 1/Fs;
    % system model
    load('Model.mat');
    % bandpass filter
    figure; H3 = filter_bank(3,4);
    F3 = {H3(:,1)',H3(:,2)',H3(:,3)'};
    figure; H4 = filter_bank(4,4);
    F4 = {H4(:,1)',H4(:,2)',H4(:,3)',H4(:,4)'};
    figure; H5 = filter_bank(5,4);
    F5 = {H5(:,1)',H5(:,2)',H5(:,3)',H5(:,4)',H5(:,5)'};
    figure; H7 = filter_bank(7,4);
    F7 = {H7(:,1)',H7(:,2)',H7(:,3)',H7(:,4)',H7(:,5)',H7(:,6)',H7(:,7)'};
    % generate disturbance and acceleration signal
    rng(0);
    v = randn(1,N);
    tfs = cell(1,length(f));
    vib = 0;
    for i = 1:length(f)
        Fpeak = f(i);  % Peak Frequency
        Q = 20;        % Quality factor
        BW =  Fpeak/Q;
        Apass = 5;     % Bandwidth Attenuation
        [b, a] = iirpeak(Fpeak/(Fs/2), BW/(Fs/2), Apass);
        % fvtool(b, a) % uncomment to see the peak filter response
        tfs{i} = tf(b,a,-1);
        tfs{i}.Ts = Ts;
        vibx = lsim(tfs{i},v);
        vibx = vibx/std(vibx);
        vib = vib + vibx*weight(i);
    end
    Dist = lsim(TF_vib2pes,vib)';
    Acc = lsim(TF_vib2acc,vib)';
    figure; fftp(Dist,Fs); legend('Distuibance Spectrum');
    figure; fftp(Acc,Fs); legend('Acceleration Spectrum');
    % generate excitation signal
    rng(0);
    ue = randn(N,1); 
    switch n
        case 3
            Exc_VCM = filter(H3(:,1),1,ue)';
            Exc_MA = filter(H3(:,3),1,ue)';
        case 4
            Exc_VCM = filter(H4(:,1),1,ue)';
            Exc_MA = filter(H4(:,3),1,ue)';
        case 5
            Exc_VCM = filter(H5(:,1),1,ue)';
            Exc_MA = filter(H5(:,3),1,ue)'+filter(H5(:,5),1,ue)';
        case 7
            Exc_VCM = filter(H7(:,1),1,ue)';
            Exc_MA = filter(H7(:,3),1,ue)'+filter(H7(:,5),1,ue)'+filter(H7(:,7),1,ue)';
    end
    figure;fftp(Exc_VCM,Fs);
    figure;fftp(Exc_MA,Fs);
    save('AccExcData.mat','Dist','Acc','Exc_VCM','Exc_MA');
    save('FilterBank.mat','F3','F4','F5','F7');
end










