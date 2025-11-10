%%%THIS SCRIPT TREATS THE LOG FILE AS ONE CONTINUOUS RESPONSE
%%%IT GETS RID OF ALL OF THE TEXT AND THEN 'UNWRAPS' THE TIME SIGNAL TO BE
%%%ONE LONG TEST
clear, clc;
close all;

fname = 'S12_Final.txt';  % 

[t_ms, ref_orig, ref_fin, angle, TRIALNUM, TRIALVALUE, U_actual, ball] = importsinefile(fname);
t_ms = unwrap(t_ms, 4996);
angle = angle*0.2353
% Apply a 5-sample median filter to the 'ball' data to remove spikes
ball_filtered = medfilt1(ball, 33); % You can change '5' to any odd number
%ball_max_filtered = movmax(ball, 5);
% Time (ms), Original Ref (rad), Final Ref (rad), Angle (rad), TrialNum, TrialValue, U_Actual (V)
% --- angle & final ref vs time ---

figure;
plot(t_ms, angle, 'LineWidth', 1.2); hold on;
plot(t_ms, ref_fin, 'LineWidth', 1.2);
plot(t_ms, ball/100, 'LineWidth', 1.2);
plot(t_ms, ball_filtered/100, 'LineWidth', 0.8);

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

min_pot = 300;
max_pot = 700; 

ball = (ball_filtered - min_pot) * (41.5 / (max_pot - min_pot));  % linear mapping 0–41.5 cm



%% --- Calculate and Plot Velocity and Acceleration (Corrected Savitzky-Golay Method) ---

% 1. Convert time to seconds
t_s = t_ms / 1000;
% Get ball position in radians
dt_s = mean(diff(t_s));

poly_order = 2;  % Polynomial order. 3 is good. Must be < framelen.
framelen = 301;   % Frame length. MUST BE ODD.
                 % THIS IS YOUR "SMOOTHING" KNOB.
                 % LARGER = MORE SMOOTHING

% 4. Generate the Savitzky-Golay filter coefficients
% g will be a matrix. 
% Column 1 (g(:,1)) is for smoothing (0th derivative)
% Column 2 (g(:,2)) is for 1st derivative (velocity)
% Column 3 (g(:,3)) is for 2nd derivative (acceleration)
[~, g] = sgolay(poly_order, framelen);

% 5. Calculate Velocity (1st derivative)
% We convolve the position signal with the 1st derivative filter
% Using 'same' keeps the signal the same length and centers the filter
ball_velocity_sg = conv(ball, -g(:,2), 'same');

% 6. Calculate Acceleration (2nd derivative)
% We convolve the position signal with the 2nd derivative filter
ball_acceleration_sg = conv(ball, -g(:,3), 'same');

% 7. Scale the derivatives by the sample time
% The 'g' coefficients give derivatives "per sample", not "per second".
% We must divide by (dt) for velocity and (dt^2) for acceleration.

ball_velocity_sg = ball_velocity_sg / dt_s;
ball_acceleration_sg = -ball_acceleration_sg / (dt_s^2);

% 8. Plot the results
figure;

% Subplot 1: Velocity
subplot(2, 1, 1);
plot(t_s, ball_velocity_sg, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title(sprintf('Smoothed Velocity', poly_order, framelen));
hold off;

% Subplot 2: Acceleration
subplot(2, 1, 2);
plot(t_s, ball_acceleration_sg, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title(sprintf('Smoothed Acceleration', poly_order, framelen));
hold off;

figure;
hold on;
plot(angle, 'LineWidth', 1.2); 
plot(ball/10, 'LineWidth', 1.2);
plot(ball_velocity_sg, 'LineWidth', 0.8);
plot(ball_acceleration_sg, 'LineWidth', 0.8);
legend({'angle', 'position', 'velocity', 'acceleration'});

grid on;
xlabel('Time (ms)');
ylabel('Velocity');
hold off;

%% --- PLOT VALID SEGMENTS (1.0 rad) ---
%% --- DEFINE STEP RESPONSE PARAMETERS ---

% 1.2 Radian Data
start_indices_1_2 = [10000, 12000, 14000];
neg_start_indices_1_2 = [11300, 13300];
segment_length_1_2 = 467; 
title_1_2 = '1.2 Radian (Positive) Step Response';
title_1_2_neg = '1.2 Radian (Negative) Step Response';

% 1.0 Radian Data
% (FIXED: Removed 19200, 21200 from this list)
start_indices_1_0 = [18200, 20200, 22200]; 
neg_start_indices_1_0 = [19200, 21200];
segment_length_1_0 = 407; 
title_1_0 = '1.0 Radian (Positive) Step Response';
title_1_0_neg = '1.0 Radian (Negative) Step Response';

% 0.8 Radian Data
start_indices_0_8 = [26300, 28300, 30300];
neg_start_indices_0_8 = [27300, 29290];
segment_length_0_8 = 314;
title_0_8 = '0.8 Radian (Positive) Step Response';
title_0_8_neg = '0.8 Radian (Negative) Step Response';

% 0.6 Radian Data
start_indices_0_6 = [35265, 37259];
neg_start_indices_0_6 = [36280, 38278];
segment_length_0_6 = 400;
title_0_6 = '0.6 Rad (Positive) Step Response';
title_0_6_neg = '0.6 Rad (Negative) Step Response';

%% --- CREATE MASTER FIGURE FOR ALL STEP RESPONSES ---
step_titles = {
    '1.2 Radian (Positive) Step Response', '1.2 Radian (Negative) Step Response', ...
    '1.0 Radian (Positive) Step Response', '1.0 Radian (Negative) Step Response', ...
    '0.8 Radian (Positive) Step Response', '0.8 Radian (Negative) Step Response', ...
    '0.6 Radian (Positive) Step Response', '0.6 Radian (Negative) Step Response'};

num_plots = numel(step_titles);
cols = 2;
rows = ceil(num_plots / cols);

allStepsFig = figure('Name', 'All Step Responses', 'Units', 'normalized', ...
                     'Position', [0.05 0.05 0.9 0.9]);

plot_idx = 1;

% Helper anonymous function to create subplot axes and call plotting
plotStep = @(start_indices, seg_len, title_str, angle, ball, vel, acc) ...
    plotAndExtractSegments(start_indices, seg_len, title_str, angle, ball, vel, acc, ...
                           subplot(rows, cols, plot_idx));

% Positive steps
plotAndExtractSegments(start_indices_1_2, segment_length_1_2, title_1_2, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;
plotAndExtractSegments(neg_start_indices_1_2, segment_length_1_2+50, title_1_2_neg, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;

plotAndExtractSegments(start_indices_1_0, segment_length_1_0, title_1_0, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;
plotAndExtractSegments(neg_start_indices_1_0, segment_length_1_0, title_1_0_neg, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;

plotAndExtractSegments(start_indices_0_8, segment_length_0_8, title_0_8, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;
plotAndExtractSegments(neg_start_indices_0_8, segment_length_0_8, title_0_8_neg, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;

plotAndExtractSegments(start_indices_0_6, segment_length_0_6, title_0_6, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx)); plot_idx=plot_idx+1;
plotAndExtractSegments(neg_start_indices_0_6, segment_length_0_6, title_0_6_neg, angle, ball, ball_velocity_sg, ball_acceleration_sg, subplot(rows, cols, plot_idx));

sgtitle('All Step Response Segments (Positive and Negative)');

%% --- CALCULATE POSITIVE STEADY-STATE ACCELERATION (Last 1/3) ---
% We calculate the mean of the absolute acceleration for the last 1/3
% of each trial set.

% 1.2 Radian Data
start_ss_1_2 = ceil(segment_length_1_2 * (2/3)); % Find start of last 1/3
accel_ss_1_2 = step_1_2_data(:, 4, start_ss_1_2:segment_length_1_2);
ss_accel_1_2 = mean(abs(accel_ss_1_2), 'all'); % 'all' gets single mean

% 1.0 Radian Data
start_ss_1_0 = ceil(segment_length_1_0 * (2/3)); % Use 1_0 length
accel_ss_1_0 = step_1_0_data(:, 4, start_ss_1_0:segment_length_1_0);
ss_accel_1_0 = mean(abs(accel_ss_1_0), 'all');

% 0.8 Radian Data
start_ss_0_8 = ceil(segment_length_0_8 * (2/3)); % Use 0_8 length
accel_ss_0_8 = step_0_8_data(:, 4, start_ss_0_8:segment_length_0_8);
ss_accel_0_8 = mean(abs(accel_ss_0_8), 'all');

% 0.6 Radian Data
start_ss_0_6 = ceil(segment_length_0_6 * (2/3)); % Use 0_6 length
accel_ss_0_6 = step_0_6_data(:, 4, start_ss_0_6:segment_length_0_6);
ss_accel_0_6 = mean(abs(accel_ss_0_6), 'all');

%% --- CALCULATE NEGATIVE STEADY-STATE ACCELERATION (Last 1/3) ---
% (Same calculation, just on the negative step data)

% 1.2 Radian Data
start_ss_1_2_neg = ceil(segment_length_1_2 * (1/5)); % Use 1_2 length
accel_ss_1_2_neg = neg_step_1_2_data(:, 4, start_ss_1_2_neg:segment_length_1_2);
ss_accel_1_2_neg = mean(abs(accel_ss_1_2_neg), 'all'); 

% 1.0 Radian Data
start_ss_1_0_neg = ceil(segment_length_1_0 * (2/3)); % Use 1_0 length
accel_ss_1_0_neg = neg_step_1_0_data(:, 4, start_ss_1_0_neg:segment_length_1_0);
ss_accel_1_0_neg = mean(abs(accel_ss_1_0_neg), 'all');

% 0.8 Radian Data
start_ss_0_8_neg = ceil(segment_length_0_8 * (2/3)); % Use 0_8 length
accel_ss_0_8_neg = neg_step_0_8_data(:, 4, start_ss_0_8_neg:segment_length_0_8);
ss_accel_0_8_neg = mean(abs(accel_ss_0_8_neg), 'all');

% 0.6 Radian Data
start_ss_0_6_neg = ceil(segment_length_0_6 * (2/3)); % Use 0_6 length
accel_ss_0_6_neg = neg_step_0_6_data(:, 4, start_ss_0_6_neg:segment_length_0_6);
ss_accel_0_6_neg = mean(abs(accel_ss_0_6_neg), 'all');

%% --- DISPLAY RESULTS AND CALCULATE GAIN K3 ---

% --- 1. Display SS Accel Results ---
fprintf('--- Steady-State Acceleration Results ---\n');
fprintf('  1.2 Rad (Pos): %.4f  | (Neg): %.4f\n', ss_accel_1_2, ss_accel_1_2_neg);
fprintf('  1.0 Rad (Pos): %.4f  | (Neg): %.4f\n', ss_accel_1_0, ss_accel_1_0_neg);
fprintf('  0.8 Rad (Pos): %.4f  | (Neg): %.4f\n', ss_accel_0_8, ss_accel_0_8_neg);
fprintf('  0.6 Rad (Pos): %.4f  | (Neg): %.4f\n', ss_accel_0_6, ss_accel_0_6_neg);
fprintf('-------------------------------------------\n');

% --- 2. Calculate Final Gain (k3) ---
rad_col = [1.2, 1.0, 0.8, 0.6]*0.2353;
rad_col = rad_col';% Column vector for polyfit
pos_ss_accel = [ss_accel_1_2, ss_accel_1_0, ss_accel_0_8, ss_accel_0_6]';
neg_ss_accel = [ss_accel_1_2_neg, ss_accel_1_0_neg, ss_accel_0_8_neg, ss_accel_0_6_neg]';

% Average the pos/neg results for a more robust gain calculation
avg_ss_accel = (pos_ss_accel + neg_ss_accel) / 2;

% --- 3. Get Best-Fit Gain (k3) ---
p_coeffs = polyfit(rad_col, avg_ss_accel, 1); % [slope, intercept]
k3_gain = p_coeffs(1);
y_offset = p_coeffs(2);

fprintf('--- Final Gain (k3) from Averaged Data ---\n');
fprintf('Best-Fit Gain (k3): %.4f (cm/s^2) / rad\n', k3_gain);
fprintf('Y-Intercept (Offset): %.4f (cm/s^2)\n', y_offset);
fprintf('--------------------------------------------\n');

% --- 4. Plot Final Gain ---
figure;
plot(rad_col, pos_ss_accel, 'bo', 'DisplayName', 'Positive Steps', 'MarkerFaceColor', 'b');
hold on;
plot(rad_col, neg_ss_accel, 'rs', 'DisplayName', 'Negative Steps', 'MarkerFaceColor', 'r');
plot(rad_col, avg_ss_accel, 'k*', 'DisplayName', 'Average', 'MarkerSize', 10);

% Plot the best-fit line
rad_fit_line = linspace(min(rad_col), max(rad_col), 10);
accel_fit_line = (k3_gain * rad_fit_line) + y_offset;
plot(rad_fit_line, accel_fit_line, 'k--', 'LineWidth', 2, ...
     'DisplayName', sprintf('Best-Fit Line (k3 = %.4f)', k3_gain));

grid on;
xlabel('\Phi Step Input Magnitude (rad)');
ylabel('Mean Steady-State Acceleration (cm/s^2)');
title('Steady-State Acceleration vs. \Phi Magnitude');
legend('Location', 'best');
hold off;

function [segments_matrix] = plotAndExtractSegments(start_indices, segment_length, figure_title, ...
    angle, ball, ball_velocity_sg, ball_acceleration_sg, ax)

num_segments = length(start_indices);
x_relative = 1:segment_length;
segments_matrix = zeros(num_segments, 4, segment_length);

axes(ax);  % draw inside passed subplot
hold on;
for i = 1:num_segments
    start_idx = start_indices(i);
    end_idx = start_idx + segment_length - 1;
    if end_idx > length(angle)
        warning('Segment starting at %d goes out of bounds. Skipping.', start_idx);
        continue;
    end
    indices = start_idx:end_idx;
    segments_matrix(i, 1, :) = angle(indices);
    segments_matrix(i, 2, :) = ball(indices);
    segments_matrix(i, 3, :) = ball_velocity_sg(indices);
    segments_matrix(i, 4, :) = ball_acceleration_sg(indices);
    plot(x_relative, squeeze(segments_matrix(i, 1, :)), 'LineWidth', 1.2);
end

title(figure_title, 'Interpreter', 'none');
xlabel('Sample Index');
ylabel('Angle (rad)');
grid on;
hold off;
end

