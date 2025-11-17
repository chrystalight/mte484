% --- Script to Compare Two Trials ---

% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% --- 1. Define filenames and import data ---

filename1 = 'alpha-005-t-360ms-clean.txt';
filename2 = 'alpha-005-t-360ms.txt';
dataLines = [2, Inf]; % Assuming header on line 1

% Import data from the first file
[Time_1, y_ref_1, y_curr_1, ~, ~, ~, ~, V_motor_1, ~] = importfile(filename1, dataLines);

% Import data from the second file
[Time_2, y_ref_2, y_curr_2, ~, ~, ~, ~, V_motor_2, ~] = importfile(filename2, dataLines);

% User stated Time and y_ref are identical, so we'll use Time_1 and y_ref_1
% as the common reference.
Time_1 = unwrap(Time_1, max(Time_1))
Time_2 = unwrap(Time_2, max(Time_2))
% --- 2. Create Comparison Figure ---

% Create a new figure window, maximize it
figure('WindowState', 'maximized');

% --- Subplot 1: Position Comparison ---
ax1 = subplot(2, 1, 1); % Top plot
hold on;

% Plot common reference position
plot(Time_1, y_ref_1, 'k-', 'LineWidth', 2);

% Plot ball position for alpha = 0.1
plot(Time_1, y_curr_1, 'r-', 'LineWidth', 1.5);

% Plot ball position for alpha = 0.05
plot(Time_2, y_curr_2, 'b:', 'LineWidth', 1.5);

hold off;
grid on;
title('Position Comparison');
xlabel('Time (ms)');
ylabel('Y Position (m)');
legend('Reference Position', 'Ball Pos (cleaned)', 'Ball Pos (raw)');

% --- Subplot 2: Controller Output Comparison ---
ax2 = subplot(2, 1, 2); % Bottom plot
hold on;

% Plot controller output for alpha = 0.1
plot(Time_1, V_motor_1, 'r-', 'LineWidth', 1.5);

% Plot controller output for alpha = 0.05
plot(Time_2, V_motor_2, 'b:', 'LineWidth', 1.5);

hold off;
grid on;
title('Controller Output (V\_motorV) Comparison');
xlabel('Time (ms)');
ylabel('Voltage (V)');
legend('V\_motor (alpha=0.1)', 'V\_motor (alpha=0.05)');

% --- 3. Final Touches ---

% Link the x-axes of the two subplots for synchronized zooming/panning
linkaxes([ax1, ax2], 'x');

% --- End of Script ---