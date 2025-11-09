%%%THIS SCRIPT TREATS THE LOG FILE AS ONE CONTINUOUS RESPONSE
%%%IT GETS RID OF ALL OF THE TEXT AND THEN 'UNWRAPS' THE TIME SIGNAL TO BE
%%%ONE LONG TEST
fname = 'S12_New_BallStep_alpha06.txt';  % 

[t_ms, ref_orig, ref_fin, angle, TRIALNUM, TRIALVALUE, U_actual, ball] = importsinefile(fname);
t_ms = unwrap(t_ms, 4996);


% Time (ms), Original Ref (rad), Final Ref (rad), Angle (rad), TrialNum, TrialValue, U_Actual (V)
% --- angle & final ref vs time ---
figure;
plot(t_ms, angle, 'LineWidth', 1.2); hold on;
plot(t_ms, ref_fin, 'LineWidth', 1.2);
plot(t_ms, ball/100, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Radians');
title('Angle and Final Ref vs Time');
legend('Angle (rad)', 'Final Ref (rad)', 'Location', 'best');
hold off;

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

