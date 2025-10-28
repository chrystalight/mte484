% This script collects the following data sent from the Arduino Mega:
%     the the Mode of operation, 
%     the Capture/Control Loop Frequency (Freq) in Hz, 
%     the Duration of the Test (Time) in seconds
%     the Controller Output vector (U) in volts or the
%         Reference Input vector (R) in degrees Celsius and
%     the Output vector (Y) in degrees Celsius.

% Setup comport
comport = input('Which COM port is the Electronic Hardware Module attached to? [i.e. COM1, COM2, ... etc.]: ','s');
arduino = serial(comport,'BaudRate',115200); 
fopen(arduino);
 
% This loop empties the serial port buffer
while arduino.BytesAvailable > 0  
    garbage = fscanf(arduino, '%f');
end

disp('Press button to start data capture.')

% This loop waits for the chamber to reach ambient temperature
Input = 0;
i = 0;
while Input == 0
  Input = fscanf(arduino, '%f');
  if i == 0
      disp('Cooling of chamber in progress!');
  else
      disp('Still cooling...');
  end
  i = i + 1;
end

clear Y;
clear T;
clear R;
clear U;
clear Input;

% Read in inital values
Mode = fscanf(arduino, '%f');     % Read in Mode of operation 
Freq = fscanf(arduino,'%f');      % Read in Sample Frequency in Hz
Time = fscanf(arduino,'%f');      % Read in Duration of Test in seconds
H_Gain = fscanf(arduino,'%f');    % Read in Feedback gain

cnt_max = round(Freq*Time);       % Calculate total # of time steps
T = linspace(0,Time,cnt_max);     % Setup Time Vector

disp('Data collection has started!');

for i=1:cnt_max
    
  if Mode == 0
    U(i) = fscanf(arduino, '%f');  % If in Mode 0, read in Controller_Output
  else
    R(i) = fscanf(arduino, '%f');  % If in Mode 1, read in Reference_Input
  end
    
  HO(i)= fscanf(arduino, '%f');     % Read in the Feedback Output
  
  Count = mod(i,100);
  if Count == 0
      disp('Still collecting data...');
  end
end

fclose(arduino)
clear arduino;
clear i;
clear Count;

% Calculate Y 
Y = HO/H_Gain;
clear HO;

disp('Data collection completed.') 

% Plot captured data
figure;
if Mode == 0
  plot(T,U)
  grid on
  title('Controller Output vs. Time');
  ylabel('Controller Output [volts]');
  xlabel('Time [seconds]');    
else        
  plot(T,R)
  grid on
  title('Reference Input vs. Time');
  ylabel('Reference Input [degrees Celsius]');
  xlabel('Time [seconds]');
end

figure;
plot(T,Y)
grid on
title('Output vs. Time');
ylabel('Temperature [degrees Celsius]');
xlabel('Time [seconds]');

disp('Plotting complete.')

