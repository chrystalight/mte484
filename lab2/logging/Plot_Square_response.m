%%%THIS SCRIPT TREATS THE LOG FILE AS ONE CONTINUOUS RESPONSE
%%%IT GETS RID OF ALL OF THE TEXT AND THEN 'UNWRAPS' THE TIME SIGNAL TO BE
%%%ONE LONG TEST

fname = 'RealControllerLogs.txt';  % <-- change to your file name

L = readlines(fname);
isData = ~cellfun(@isempty, regexp(L, '^\s*\d', 'once'));  % rows that begin with a number
dataLines = L(isData);
S = strjoin(dataLines, newline);
C = textscan(S, '%f%f%f%f%f%f%f', 'Delimiter', ',', 'CollectOutput', true);

% Time (ms), Original Ref (rad), Final Ref (rad), Angle (rad), TrialNum, TrialValue, U_Actual (V)
D = C{1};
t_ms     = unwrap(D(:,1),4995);
ref_orig = D(:,2);
ref_fin  = D(:,3);
angle    = D(:,4);
% trialN = D(:,5); trialVal = D(:,6);
U_actual = D(:,7);

% --- angle & final ref vs time ---
figure;
plot(t_ms, angle, 'LineWidth', 1.2); hold on;
plot(t_ms, ref_fin, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Radians');
title('Angle and Final Ref vs Time');
legend('Angle (rad)', 'Final Ref (rad)', 'Location', 'best');

% --- U_actual vs time ---
figure;
plot(t_ms, U_actual, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('U\_actual (V)');
title('U\_actual vs Time');
yline(-6)
yline(6)

% --- Y_ss --- 
for c = 1:4
    segment = angle((2500+5000*c)/5:(4990+5000*c)/5);
    y_ss(c) = mean(segment)
end

