%%% THIS SCRIPT PARSES THE LOG FILE AND SPLITS THE DATA BY TRIAL
%%% This allows for individual analysis of each trial.

clear;
clc;
close all;

% --- 1. Read and Parse All Data (Same as your script) ---
fname = 'RealControllerLogs.txt'; % <-- make sure this file is in the same folder

L = readlines(fname);
isData = ~cellfun(@isempty, regexp(L, '^\s*\d', 'once')); % rows that begin with a number
dataLines = L(isData);
S = strjoin(dataLines, newline);
C = textscan(S, '%f%f%f%f%f%f%f', 'Delimiter', ',', 'CollectOutput', true);

% D is one large matrix containing ALL trials
% Columns: 1:Time(ms), 2:OrigRef(rad), 3:FinalRef(rad), 4:Angle(rad), 5:TrialNum, 6:TrialValue, 7:U_Actual(V)
D = C{1};

% --- 2. Split Data by Trial Number ---

trialNumbers = D(:, 5);           % Get the trial number column
uniqueTrials = unique(trialNumbers); % Find all unique trial numbers (e.g., [1, 2, 3, 4, 5])
numTrials = length(uniqueTrials);  % Count how many trials there are

% Create a cell array to store the data for each trial
% A cell array allows each element to be a matrix of a different size
allTrialsData = cell(numTrials, 1); 

fprintf('Found %d trials. Splitting data...\n', numTrials);

for i = 1:numTrials
    currentTrialNum = uniqueTrials(i);
    
    % Find all rows (indices) that match the current trial number
    trialIndices = (trialNumbers == currentTrialNum);
    
    % Extract all data for those rows and store it in our cell array
    allTrialsData{i} = D(trialIndices, :);
    
    fprintf('Trial %d: %d data points\n', currentTrialNum, size(allTrialsData{i}, 1));
end

fprintf('Data splitting complete.\n');
disp('You can now access each trial using allTrialsData{k}, where k is the trial number.');
disp('Example: To get all data for Trial 1, use allTrialsData{1}');


% --- 3. Example: Analyze and Plot a SINGLE Trial (e.g., Trial 3) ---

% Change this to plot a different trial
trialToPlot = 2;

if trialToPlot > numTrials
    fprintf('Error: Trial %d does not exist. Plotting trial 1 instead.\n', trialToPlot);
    trialToPlot = 1;
end

% Get the data matrix for just the trial we want
data_single_trial = allTrialsData{trialToPlot};

% Extract columns for this trial. Note: Time (col 1) does NOT need unwrap()
t_ms     = data_single_trial(:, 1);
ref_orig = data_single_trial(:, 2);
ref_fin  = data_single_trial(:, 3);
angle    = data_single_trial(:, 4);
% trialN = data_single_trial(:, 5);
% trialVal = data_single_trial(:, 6);
U_actual = data_single_trial(:, 7);

% --- Plot: angle & final ref vs time for ONE trial ---
figure;
plot(t_ms, angle, 'LineWidth', 1.2); hold on;
plot(t_ms, ref_fin, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Radians');
title(sprintf('Trial %d: Angle and Final Ref vs Time', trialToPlot));
legend('Angle (rad)', 'Final Ref (rad)', 'Location', 'best');

% --- Plot: U_actual vs time for ONE trial ---
figure;
plot(t_ms, U_actual, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('U\_actual (V)');
title(sprintf('Trial %d: U\\_actual vs Time', trialToPlot));
yline(-6, 'r--', 'LineWidth', 1.5);
yline(6, 'r--', 'LineWidth', 1.5);
legend('U\_actual (V)', 'Voltage Limit', 'Location', 'best');

% --- NEW PLOT: Show raw vs. filtered angle for the single trial ---
windowSize_plot = 10; % Use the same window size as in settling calc
angle_filtered_plot = movmean(angle, windowSize_plot);

figure;
plot(t_ms, angle, 'Color', [0.5 0.5 0.5], 'DisplayName', 'Raw Angle');
hold on;
plot(t_ms, angle_filtered_plot, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Filtered Angle (5-sample avg)');
plot(t_ms, ref_fin, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Final Ref');
grid on;
xlabel('Time (ms)');
ylabel('Radians');
title(sprintf('Trial %d: Raw vs. Filtered Angle (Noise Smoothing)', trialToPlot));
legend('show', 'Location', 'best');



% --- 4. Example: Plot all trials overlaid (with resetting time) ---
figure;
hold on;
for i = 1:numTrials
    % Get data for trial 'i'
    trialData = allTrialsData{i};
    
    % Extract time and angle
    t_trial = trialData(:, 1);
    angle_trial = trialData(:, 4);
    
    % Plot this trial's angle vs. its own time
    plot(t_trial, angle_trial, 'DisplayName', sprintf('Trial %d', uniqueTrials(i)));
end
hold off;
grid on;
xlabel('Time (ms)');
ylabel('Angle (rad)');
title('All Trials Plotted Individually (Time Resets)');
legend('show', 'Location', 'best');

% --- 5. Calculate 2% Settling Time for Each Trial ---

fprintf('\n--- Calculating 2%% Settling Times ---\n');
fprintf('Using a 5-sample moving average filter to reduce noise.\n');

% Create an array to store settling times
settlingTimes = zeros(numTrials, 1);
windowSize = 10; % Moving average window size. Adjust if noise is still an issue.

for i = 1:numTrials
    % Get data for the current trial
    trialData = allTrialsData{i};
    
    t_ms    = trialData(:, 1);
    ref_fin = trialData(:, 3);
    angle   = trialData(:, 4);
    
    angle_filtered = movmean(angle, windowSize);
    
    % Get final (steady-state) value and initial value
    final_value = ref_fin(1);
    initial_value = angle_filtered(1); % Use the first *filtered* angle as initial
    
    % Calculate the total step magnitude
    step_magnitude = abs(final_value - initial_value);
    
    if step_magnitude < 1e-6 % Handle case where step is (basically) zero
        settling_time_ms = 0;
    else
        % Define the 2% settling band
        upper_band = final_value + 0.02 * step_magnitude;
        lower_band = final_value - 0.02 * step_magnitude;
        
        % Find all time points *OUTSIDE* the 2% band
        is_outside_band = (angle_filtered > upper_band) | (angle_filtered < lower_band);
        
        % Find the *last index* where the angle was outside the band
        last_outside_index = find(is_outside_band, 1, 'last');
        
        if isempty(last_outside_index)
            % Never went outside the band (or started inside and stayed)
            settling_time_ms = 0;
        elseif last_outside_index == length(t_ms)
            % The very last point was outside, so it never settled
            settling_time_ms = Inf;
        else
            % The settling time is the time of the *next* sample
            % after the last time it was outside.
            settling_time_ms = t_ms(last_outside_index + 1);
        end
    end
    
    settlingTimes(i) = settling_time_ms;
    fprintf('Trial %d: Settling Time = %.0f ms\n', uniqueTrials(i), settling_time_ms);
end

% Display the results in a table
disp('--- Settling Time Summary (ms) ---');
T = table(uniqueTrials, settlingTimes, 'VariableNames', {'Trial', 'SettlingTime_ms'});
disp(T);