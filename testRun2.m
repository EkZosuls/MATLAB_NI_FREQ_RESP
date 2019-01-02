%Zosuls NI daq card frequency response test program. This version uses nidaqmx drivers
%instead of the legacy drivers and the c executable.
%Copyright Aleks Zosuls Boston University 2017
clear
PLOTTING = 0; %true, turn on figure generation
%nChanIn = 4;  %number of Input and output analog channels
%nChanOut = 2;
stimLength =200000;%number of points for each channel
%%%%%%%%%%%%%%%%%%%%
%%%%%%input calibrations%%%%%%%%%%%
speakerVolts = 1;  %V
speakerCurrent = 1/200/.05; %V/gain/R  %measured 0.2 ohms with Fluke November 2016
LDVCal = .005; %m/s/V
DacAtten = 20; %attenuator size on DA converter
%there params are for generating two sine waves...
%ex_freq1 =10;%freqeuncy of channel 0
stimVoltage = .02 * DacAtten;%volts p-p of channel 0
%voltspp2 = 0;%frequency of channel 1
%ex_freq2 = 70;%volts p-p of channel 1
sample_rate = 80645.161;%single channel sample rate. In other words each of the 
%countPerVolt = 2^16/10;
%four IO operations happens this times per second.
%navg = 1;%number of averages. the exe does ensemble avg.

%% ni stuff%%%%%%%%%%%%%%%%%%%
v = daq.getVendors();
d = daq.getDevices();
s = daq.createSession('ni');
addAnalogOutputChannel(s,'Dev3', 'ao0', 'Voltage');
% this version has three input channels....
addAnalogInputChannel(s,'Dev3', 0, 'Voltage'); % speaker volts
addAnalogInputChannel(s,'Dev3', 1, 'Voltage'); % speaker amps 
addAnalogInputChannel(s,'Dev3', 2, 'Voltage'); % LDV
addAnalogInputChannel(s,'Dev3', 3, 'Voltage'); % mon
nChannelsAD = 4;
%sample rate should enable an integer number of cycles to be collected
%not sure if s.Rate is scan or sample rate because of mux
s.Rate = sample_rate; %* nChannelsAD;
npts = stimLength * nChannelsAD;
zerosData = zeros(stimLength,1);
queueOutputData(s, zerosData); %this is here beacuse the output queue size
%determins the input number of samples...

%%%% directory and file making stuff %%%%%%%%%%%%%%%%%%%%%%%%%%
parent_dir = pwd
der_base = input('Enter directory name:  ', 's'); %save each experiment run
%in separate directory
%with dir name that is indicative of the experiment
mkdir(der_base);
run_dir = strcat(parent_dir,'\', der_base); %make string of new directory

%%%%% generate the stimulus frequency vector
%[signal2,freq2] = interger_cycle_gen(ex_freq2, npts, sample_rate, voltspp2);
%freqVec = 5* logspace(0,3,40);
%more points to find Cms
freqVec = [5:2:100 (logspace(2,4,150))];
%%%randomize the presentation order of the frequencies
order = randperm(length(freqVec));
freqVec= freqVec(order);
tic;
%frequency loop
for i = 1:length(freqVec)
%generate them sine waves. Integer number of cycles...
[signal1,freq1] = interger_cycle_gen(freqVec(i), stimLength, sample_rate, stimVoltage);
exactPresFreq(i) = freq1;
queueOutputData(s, signal1'); %check to make sure the signal1 is in volts for daqmx

%make an interleaved signal vector for the NI
%signal = zeros(1,length(signal1)+length(signal2));
%index1 = 1:2:length(signal*nChanOut);
%index2 = 2:2:length(signal*nChanOut);
%signal(index1) = signal1;
%signal(index2) = signal2;
%make sure its done correct

%if(PLOTTING)
%    figure(121)
%    plot(signal)
%end

%save the vector to a temporary file on the hard drive of the windows os
%save('c:\temp\tempsig1.dat', 'signal','-ascii','-tabs'); %saves to a readable float format
data = startForeground(s);   %run the stuff
%make a command line command and run it
%myString = ['!ni2chPlay4chRec c:\temp\tempsig1.dat c:\temp\himom1.dat ', num2str(npts), ' ', num2str(sample_rate), ' ', num2str(navg)];
%eval(myString)
%get back them daters
%load('c:\temp\himom1.dat')
%de interlace the ADC and convert to volts
%index1 = 1:4:length(himom1);
%index2 = 2:4:length(himom1);
%index3 = 3:4:length(himom1);
%index4 = 4:4:length(himom1);
spVolts = speakerVolts * data(:,1);
spAmps = speakerCurrent * data(:,2);
LDV = LDVCal * data(:,3);
mon = data(:,4);

outfile = strcat(run_dir,'\freq',num2str(round(freq1)),'.mat');    % make output filename if saving raw data
save(outfile, 'spVolts', 'spAmps', 'LDV', 'mon'); %,'-ascii','-tabs');
        
[RESPONSE,PHASE] = fftpoint(-LDV, sample_rate, freq1);   %function to pick off frequency from fft
%mag_ph(i,j,1)= velocity_cal * (RESPONSE *2)/AM502;  %disp is LDV cal,x2 gives peak to peak values, divide out amp gain
%2012-02-04 removed '2' from mag_ph calibration to make zero to
%peak not peak to peak. More useful for power measurements etc
mag_ph(i,1)= RESPONSE; 
%PHASE = ni_cli_delay_comp_rad(freq1,PHASE, sample_rate);  %compensates for one point delay in PCI6052E
mag_ph(i,2)=PHASE*180/(pi);       %phase degrees
mag_ph(i,3)=PHASE;                %phase radians
mag_ph(i,4) = freq1;

if(PLOTTING)
figure
plot(spVolts, 'y')
hold on
title(freq1)
plot(spAmps,'b')
plot(LDV,'m')
plot(mon,'g')
set(gca,'Color',[.1 .1 .1]);
end


end %end of for loop of frequency
delete(s)
clear('s')
runTime = toc
outfile = strcat(run_dir,'\metadata.mat');
save(outfile)

[f,I] = sort(mag_ph(:,4))
figure
subplot(2,1,1)
loglog(f, mag_ph(I,1))
title(outfile)
xlabel('Frequency')
ylabel('Magnitude')
subplot(2,1,2)
semilogx(f, mag_ph(I,2))