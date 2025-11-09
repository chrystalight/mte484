[TIMEMS, ORIGINALREFRAD, FINALREFRAD, anglerad, TRIALNUM, TRIALVALUE, U_ACTUALV, ball] = importsinefile('S12_BallSineTest');
[TIMEMS, ORIGINALREFRAD, FINALREFRAD, anglerad, TRIALNUM, TRIALVALUE, U_ACTUALV, ball] = importsinefile('S12_BallStepTest');

figure;
%timems = unwrap(TIMEMS,14996)
timems = unwrap(TIMEMS,4996)
k = 30; % Window size -- YOU MUST TUNE THIS VALUE
ball_smoothed_mov = movmean(ball, k);
ball_smoothed_med = medfilt1(ball, k);
order = 2;
framelen = 31; % Window size -- MUST BE ODD. TUNE THIS.
ball_smoothed_sg = sgolayfilt(ball, order, framelen);

start = 13000;
% --- Plot on the Left Y-Axis ---
yyaxis left;
plot(timems((start/4):(20000/4)), anglerad((start/4):(20000/4)), 'b-');
ylabel('anglerad'); % Label for the left axis
ylim([-0.8 0.8]); % Optional: set limits for anglerad

% --- Plot on the Right Y-Axis ---
yyaxis right;
plot(timems((start/4):(20000/4)), ball_smoothed_mov((start/4):(20000/4)), 'r-');
hold on; % <-- ADDED: Tells MATLAB to layer the next plots
plot(timems((start/4):(20000/4)), ball_smoothed_med((start/4):(20000/4)), 'g-');

% --- FIXED THE LINE BELOW ---
% Plot the variable 'ball_smoothed_sg', not the function 'sgolayfilt'
plot(timems((start/4):(20000/4)), ball_smoothed_sg((start/4):(20000/4)), 'k-'); % 'k-' is a black line

hold off; % <-- ADDED: Good practice to release the hold

% --- FIXED LABELS ---
ylabel('Ball Position'); % Use one, general label for the axis
ylim([0 1000]); % Optional: set limits for ball (only need this once)

% --- ADDED A LEGEND ---
% This is crucial for telling your lines apart
legend('anglerad', 'Moving Mean', 'Median Filter', 'S-G Filter', 'Location', 'northwest');

% --- Add Common Labels ---
xlabel('Time (ms)');
title('Filter Comparison vs. Time');