% This script parses manually specified data files to extract step response
% data and generates a separate plot for each corresponding Kp value.
%

%It's mostly vibecoded and may or may not work but doesn't do any filtering

% Ensure the 'parseDataFile.m' helper function is in the same directory.

clear; 
clc; 
close all;

% --- Manual User Inputs ---
% INSTRUCTIONS: Enter your four filenames in the 'fileNames' cell array
% and their corresponding Kp values in the 'kpValues' array.
fileNames = {'kp_neg_19_29.txt', 'kp_neg_3_34.txt', 'kp_neg_4_82.txt', 'kp_neg_9_64.txt'};
kpValues  = [-19.2928, -3.3494, -4.8232, -9.6483];

% --- Data Validation ---
if length(fileNames) ~= length(kpValues)
    error('The number of file names must match the number of Kp values.');
end

% --- Process Data Files ---
% Use a containers.Map to store all parsed data, keyed by the Kp value.
allParsedData = containers.Map('KeyType', 'double', 'ValueType', 'any');

fprintf('--- Starting Data Processing ---\n');
for i = 1:length(fileNames)
    filename = fileNames{i};
    manual_kp = kpValues(i);
    
    % Check if the specified file actually exists
    if ~isfile(filename)
        fprintf('  > Warning: File "%s" not found. Skipping.\n', filename);
        continue;
    end
    
    fprintf('Parsing %s (manually assigned Kp = %.4f)...\n', filename, manual_kp);
    
    % Call the helper function to parse the file content.
    % We ignore the Kp value found inside the file by using '~'.
    [~, testData] = parseDataFile(filename);
    
    if ~isempty(testData) && testData.Count > 0
        % Use the MANUALLY assigned Kp value as the key
        allParsedData(manual_kp) = testData;
        fprintf('  > Found data for %d unique step magnitude(s).\n', testData.Count);
    else
        fprintf('  > Warning: Could not parse valid data from %s. Skipping.\n', filename);
    end
end

if isempty(allParsedData)
    fprintf('\nProcessing complete, but no valid step response data was found.\n');
    return;
end

% --- Generate Plots ---
fprintf('\n--- Generating Plots ---\n');
kp_values_sorted = sort(cell2mat(allParsedData.keys));

for i = 1:length(kp_values_sorted)
    kp = kp_values_sorted(i);
    testData = allParsedData(kp); % This is the map for the current Kp
    
    % Create a new figure for each Kp value.
    figure('Name', sprintf('Step Responses for Kp = %.4f', kp), 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);
    hold on; % Allow multiple lines on the same plot
    
    magnitudes = sort(cell2mat(testData.keys));
    
    % Use a built-in colormap for distinct colors.
    colors = lines(length(magnitudes));
    
    plotHandles = []; % To store handles for the legend
    legendLabels = {}; % To store labels for the legend
    
    % Plot data for each magnitude
    for j = 1:length(magnitudes)
        mag = magnitudes(j);
        replicates = testData(mag); % Cell array of data tables
        
        % Plot each replicate
        for k = 1:length(replicates)
            data_table = replicates{k};
            p = plot(data_table.Time_ms, data_table.Angle_rad, ...
                     'Color', colors(j, :), 'LineWidth', 1.5);
            if k == 1 % Only save the handle of the first replicate for the legend
                plotHandles(end+1) = p;
                legendLabels{end+1} = sprintf('Response (Mag: %.2f rad)', mag);
            end
        end
    end
    
    % Plot a single reference line (they are all the same shape)
    % We take the first replicate of the first magnitude as representative
    firstMag = magnitudes(1);
    firstRepData = testData(firstMag);
    firstRepTable = firstRepData{1};
    ref_p = plot(firstRepTable.Time_ms, firstRepTable.FinalRef_V_or_rad, ...
                 'k--', 'LineWidth', 2);
             
    plotHandles(end+1) = ref_p;
    legendLabels{end+1} = 'Reference Signal';
    
    hold off;
    
    % --- Formatting ---
    title(sprintf('Step Responses for Kp = %.4f', kp), 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('Time (ms)', 'FontSize', 12);
    ylabel('Angle (rad)', 'FontSize', 12);
    grid on;
    box on;
    ax = gca;
    ax.FontSize = 10;
    legend(plotHandles, legendLabels, 'Location', 'best', 'FontSize', 10);
end

fprintf('\nAll plots have been generated.\n');

