function fftp(y,Fs,NFFT)
if nargin < 2
    Fs = 2*pi;
end
L = numel(y);
if nargin < 3
    NFFT = 2^nextpow2(L); % Next power of 2 from length of y
end
X = fft(y,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.
magabs = 2*abs(X(1:NFFT/2+1));
magdb = 20*log10(magabs);
plot(f,magabs); 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')