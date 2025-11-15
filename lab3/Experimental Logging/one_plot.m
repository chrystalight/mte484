% --- Script to Plot Data and Analyze Step Response ---

% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% --- 1. Define filename and import data ---

% Set the filename you want to plot
filename = 'alpha-01-t-360ms.txt';
filename = 'alpha-01-t-360ms-NO4.txt';
% filename = 'alpha-005-t-360ms.txt'; % You can swap filenames to analyze

dataLines = [2, Inf]; % Assuming header on line 1

% Import the data
% Capture the 'Trial' vector (7th return argument)
[Timems, y_refm, y_currm, theta_ref_rawr, ~, ~, Trial, ~, ~] = importfile(filename, dataLines);
Timems = unwrap(Timems, max(Timems));
fprintf('Analyzing file: %s\n', filename);
fprintf('========================================\n');

% --- 2. Analyze Performance Metrics for Each Trial ---

unique_trials = unique(Trial);

% *** NEW: Initialize previous step's final value. Assume start from 0. ***
previous_y_ref_final = 0.1;

for i = 2:8
    current_trial_num = unique_trials(i);
    
    % Get indices for the current trial
    trial_indices = find(Trial == current_trial_num);
    
    % Extract data for this trial
    trial_time_ms = Timems(trial_indices);
    trial_y_ref = y_refm(trial_indices);
    trial_y_curr = y_currm(trial_indices);
    
    % Get trial start time and make time relative
    trial_start_time_ms = trial_time_ms(1);
    relative_time_ms = trial_time_ms - trial_start_time_ms;
    
    % --- Find step values ---
    % *** MODIFIED: Initial value is the final value of the previous step ***
    y_ref_initial = previous_y_ref_final;
    
    % Assume final value is the mean of the ref signal after 10s
    steady_state_ref_indices = find(relative_time_ms >= 10000);
    
    if isempty(steady_state_ref_indices)
        % Handle cases where trial is shorter than 10s
        y_ref_final = trial_y_ref(end);
    else
        y_ref_final = mean(trial_y_ref(steady_state_ref_indices));
    end
    
    % *** MODIFIED: Step amplitude is now calculated correctly ***
    step_amplitude = y_ref_final - y_ref_initial;

    % --- 1. Steady-State Error ---
    % Use user's hint: steady state is reached by 10000 ms
    steady_state_curr_indices = find(relative_time_ms >= 10000);
    
    if isempty(steady_state_curr_indices)
        sse = NaN; % Not enough data
    else
        y_curr_steady_state_avg = mean(trial_y_curr(steady_state_curr_indices));
        sse = y_ref_final - y_curr_steady_state_avg;
    end

    % --- 2. Overshoot ---
    if step_amplitude == 0
        overshoot_percent = 0; % No step, no overshoot
    elseif step_amplitude > 0 % Step Up
        peak_value = max(trial_y_curr);
        overshoot_percent = ((peak_value - y_ref_final) / step_amplitude) * 100;
    else % Step Down
        % Calculate undershoot as "overshoot"
        peak_value = min(trial_y_curr); 
        overshoot_percent = ((y_ref_final - peak_value) / abs(step_amplitude)) * 100;
    end
    % Ensure overshoot is not negative (i.e., it never crossed the final value)
    overshoot_percent = max(0, overshoot_percent);

    % --- 3. Settling Time (2%) ---
    if step_amplitude == 0
        settling_time_ms = 0; % No step, settled immediately
    else
        % Define the 2% error band based on the step amplitude
        error_band = 0.02 * abs(step_amplitude);
        upper_bound = y_ref_final + error_band;
        lower_bound = y_ref_final - error_band;
        
        % Find all indices *outside* the 2% band, *after* the step starts
        % This avoids counting initial state as "outside"
        outside_band_indices = find(trial_y_curr > upper_bound | trial_y_curr < lower_bound);
        
        if isempty(outside_band_indices)
            % Never left the band (or started in it)
            settling_time_ms = 0;
        else
            % Find the *last* time it was outside the band
            last_outside_index = outside_band_indices(end);
            
            % Check if the system *starts* outside the band and never settles
            % If the last_outside_index is the last point, it hasn't settled
            if last_outside_index == length(relative_time_ms)
                 settling_time_ms = Inf; % Or max trial time
            else
                 % The settling time is the relative time of this last-out-of-band point
                 settling_time_ms = relative_time_ms(last_outside_index);
            end
        end
    end
    
    % --- Print Results ---
    fprintf('--- Trial %d --- (Time: %.0f ms to %.0f ms)\n', ...
        current_trial_num, trial_start_time_ms, trial_time_ms(end));
    fprintf('  Step: %.3f to %.3f (Amplitude: %.3f)\n', ...
        y_ref_initial, y_ref_final, step_amplitude);
    fprintf('  Steady-State Error: %.4f\n', sse);
    fprintf('  Overshoot: %.2f %%\n', overshoot_percent);
    fprintf('  Settling Time (2%%): %.0f ms\n', settling_time_ms);
    fprintf('\n');

    % *** NEW: Update previous_y_ref_final for the next loop iteration ***
    previous_y_ref_final = y_ref_final;
end


% --- 3. Create the Plot ---

% Create a new figure window
figure;
hold on; % Hold on to plot multiple lines

% Plot y_refm vs. Timems
plot(Timems, y_refm, 'b-', 'LineWidth', 1.5);

% Plot y_currm vs. Timems
plot(Timems, y_currm, 'r--', 'LineWidth', 1.5);

% Plot theta_ref_rawr vs. Timems
plot(Timems, theta_ref_rawr, 'g:', 'LineWidth', 1.5);

% Release the hold
hold off;

% --- 4. Add Labels and Title ---

% Add a title to the plot
title(['Trial Data: ' strrep(filename, '_', '\_')]);

% Label the x-axis
xlabel('Time (ms)');

% Label the y-axis
ylabel('Value');

% Add a legend to identify the lines
legend('Y Reference (y\_refm)', 'Y Current (y\_currm)', 'Theta Ref Raw (theta\_ref\_rawr)');

% Add a grid for easier reading
grid on;

% --- End of Script ---