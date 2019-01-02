function [response,phase] = fftpoint(timedomainsignal,samplingrate,frequency);
%[RESPONSE,PHASE] = fftpoint(TIMEDOMAINSIGNAL,SAMPLINGRATE,FREQUENCY)
%   Magnitude and phase from FFT of time domain waveform (sinusoid) at
%   particular Frequency. Returns frequency domain
%   content of a sampled TIMEDOMAINSIGNAL at the specified
%   FREQUENCY.  Only positive frequencies are returned. Voltages are
%   preserved. If TIMEDOMAINSIGNAL is a matrix, then each column must
%   represent a different waveform.
%
%   RESPONSE is a vector whose whose values are the zero to peak values
%   that correspond with the frequency. Each column represents a different
%   waveform.
%  
%   PHASE is a vector whose value correspond to phase of the signal in
%   radians for the frequency in FREQUENCY. Each column represents a
%   different waveform.
%  
%freqresponse.m 
%Created by Brian Miller.  Documented 8/8/2003 by Brian Miller
freqdomainsignal = fft(timedomainsignal);
[len, col] = size(freqdomainsignal);
% Multiply by 2 and divide by the number of points in the vector to get the
% magnitude
mag = (2*freqdomainsignal)/len;
mag = mag(1:length(mag)/2,:);
spacing = (samplingrate)./len;  % Frequency bin spacing
index = (frequency./spacing)+1; % Index that corresponds to correct frequency bin.
if index ~= round(index)
%if abs(index - round(index)) > 0.00001
    disp(sprintf('WARNING! Frequency rounded from %f to %f match an FFT bin.',...
        frequency,(round(index)-1)*spacing));
    index = round(index);
end
index = int32(index);
for count = 1:col
    response(count) = abs(mag(index,count));
    phase(count) = angle(mag(index,count));
    %f = daqfft(timedomainsignal(:,count),samplingrate);
    %hold on;
    %plot(frequencies(count),response(count),'ro');
end
%keyboard;
% freqresponse = semilogx(frequencies,response);
% set(freqresponse,'LineWidth',3);
% xlabel('Frequency(Hz)');
% ylabel('Peak Voltage');
% title('Frequency Response');
% grid on;
