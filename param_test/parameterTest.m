%% Setup / Data cleanup
clear
load('ParameterTest_Sept28.mat')

%Set this to the file you want to study
allData = Data_3ms;

% Clean up data
validRows = [];
for row = 1+1:size(allData, 1)
    if abs(allData(row,:).AngleRad - allData(row-1,:).AngleRad) < 1 || ...
            allData(row,:).Reference ~= allData(row-1,:).Reference
        validRows = [validRows;row];
    end
end

allData = allData(validRows,:);
referenceVals = unique(allData.Reference);

% Plot data
figure
title('1ms Data')
hold on;
for ref = -5 : 1 : 5+1
    condition = all([allData.Reference == ref, abs(allData.AngleRad) < pi], 2);
    data = allData(condition,:);
    x = data.Time;
    y = unwrap(data.AngleRad);
    plot(x, y);
end
legend('-5V', '-4V', '-3V', '-2V', '-1V', ...
    '+1V', '+2V', '+3V', '+4V', '+5V')

%% Export to the graphing-484-data script's format
for ref = -5 : 1 : 5
    if ref == 0
        continue
    end
    condition = all([allData.Reference == ref, abs(allData.AngleRad) < pi], 2);
    data = allData(condition,:);
    T = data.Time/1000;
    U = data.Reference;
    Y = data.AngleRad;
    % data = renamevars(data, ['AngleRad', 'Reference', 'Time'], ['Y', 'U', 'T']);
    % x = data.Time;
    % y = unwrap(data.AngleRad);
    % plot(x, y);
    save(sprintf('step_input_%d.mat', ref), 'T', 'U', 'Y', 'data')
end

