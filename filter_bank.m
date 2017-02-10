function H=filter_bank(N,K)
%% Codes for subband adaptive filtering: adaptive cross filters, only the adjacent filters
% first we assuming that we know the plant, therefore, we will only need to
% partition the x signal
% We use cosine-modulated filter bank

% Design of cosine-modulated filter bank
% N = 12;                                     % Number of subbands, 8
% K = 4;
L = 2*K*N;                                   % Length of lowpass prototype filter
[hopt,passedge] = opt_filter(L-1,N);       % Design prototype filter
[H,F] = make_bank(hopt,N);                     % Generate filter bank
H = H.'; % Analysis Section
F = F.'; % Synthesis Section

% plot frequency response
DFTpoint = 4096;                           % FFT size
[Hz,w] = FreqResp(H,DFTpoint); Hz = Hz.'; 
for k = 1:N
    if mod(k,2)==0
        plot(w/pi,20*log10(abs(Hz(k,:))+eps));hold on;
    else
        plot(w/pi,20*log10(abs(Hz(k,:))+eps),':');hold on;
    end
end
axis([0 1 -120 10]); box on; ylabel('Gain (dB)');
end

