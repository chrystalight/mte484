% Set initial values
Freq = 40;      % Set Sample Frequency in Hz
Time = 10;      % Set Duration of Test in seconds
H_Gain = 4;     % Set Feedback gain

% Setup
comport = input('Which COM port is in use? [i.e. COM1, COM2, ... etc.]: ','s');
arduino = serialport(comport, 'BaudRate', 115200); 
 
% This loop empties the serial port buffer
while arduino.NumBytesAvailable > 0  
    garbage = readline(arduino, '%f');
end


cnt_max = round(Freq*Time);       % Calculate total # of time steps
T = linspace(0,Time,cnt_max);     % Setup Time Vector
U = zeros(1, cnt_max);
R = zeros(1, cnt_max);
HO = zeros(1, cnt_max);
Y = zeros(1, cnt_max);

disp('Data collection has started!');

for i=1:cnt_max
    
  R(i) = readline(arduino, '%f');     % Read in Reference_Input (as a float)
    
  HO(i)= readline(arduino, '%f');     % Read in the Feedback Output

  % Calculate what the arduino should set its output voltage to:
  % Currently a placeholder, should actually use the system equations
  output_voltage = -2 * R(i);
  Y(i) = output_voltage;
  writeline(arduino, float2str(output_voltage))
  
  Count = mod(i,100);
  if Count == 0
      disp('Still collecting data...'); %???
  end
end

clear arduino;
clear i;
clear Count;
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
  ylabel('Reference Input [volts]');
  xlabel('Time [seconds]');
end

figure;
plot(T,Y)
grid on
title('Output vs. Time');
ylabel('Angle [degrees]');
xlabel('Time [seconds]');

disp('Plotting complete.')

