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
trialToPlot = 3;

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