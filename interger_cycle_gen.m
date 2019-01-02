function [signal,freq] = interger_cycle_gen(ex_freq, npts, sample_rate, voltspp)
%Boston University  AZosuls 2007
%this program generates stimulus sines for data aquisition.  
%ex_freq is the sine freq you want to generate
%npts is the number of points to generate
%sample rate is in hertz
%volts is the peak to peak value of the sine

%DACbits =(2^16)/2;              %makes volts equal to peak to peak output on dac
%npts = 2^11;                     %length of base signal

sample_per_sec = 1/sample_rate;  %convert to sample period in seconds
T=1/(ex_freq);                  %compute period of freq
points = T/sample_per_sec;      %compute number of points per cycle of sine
periods = npts/points;            %compute number of periods of sine in presentation
points = npts/round(periods);      %divide num of points by integer number of periods 
T=points*sample_per_sec;       %multiply points per period by SR to get adjusted period
freq=1/T                       %actual excitation frequency
timebase =(0:npts-1)*sample_per_sec;
signal = (voltspp/2*sin(2*pi*freq*timebase));  %generates the signal to be send to TDT

%save('c:\temp\tempsig.dat', 'signal','-ascii','-tabs'); %saves to a readable float format
