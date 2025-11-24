%clear
%clc
%close all

attempt_number = 6; 
% load('new_plant_polesets.mat')
load('working_poleset.mat');
resultsFile = 'integrator_T_360.mat';

% set time step
k_2 = 0.06091;
k_3 = -4.25;
T = 0.360; 

s = tf('s');
P = (k_2*k_3)/(s*s);

%%%%%%%%%%% ---- ADDING IN THE HEAVY EMA FILTERING ----
alpha = 0.05;    % The 'FILTER_ALPHA_BALL' from the C++ code
T_inner = 0.004; % The ISR sample time (T_INNER)
%F(s) = a / (s + a)
%s_p = log(1-alpha) / T_inner;  (where log is natural log)
a = -log(1 - alpha) / T_inner; 
% F_s = a / (s + a);            % Our 1st-order filter model
% P_filt = P * F_s;              % with the filter
%%%%%%%%% End of EMA stuff

G_orig = c2d(P, T, 'zoh');

[num, den] = tfdata(G_orig, 'v');
%[r, p, k] = residuez(num, den);

%Pole Picking Parameters
total = 200;
rmax = 0.965;
center = 0;
% % poleset = generate_poles(total,rmax,center)
% poleset = complex(myPoleSet(:, 2), myPoleSet(:, 3));
poleset = complex(working_poleset(:, 2), working_poleset(:, 3));
%poleset = prune_1.';

%specs
ref_amplitude = 0.15; 
max_U = 0.65;
max_OS = 1.42;
max_ts = 7; 
max_ess = 0; 

% does the plant have a double integrator?
double_integrator_flag = 1;

% should the controller have an integrator?
controller_integrator_flag = 1;

%% Plant Poles and Coefficients in its Partial Fraction Decomposition

stableRealPlantPoles = [];
stableComplexPlantPoles = [];
unstablePlantPoles = [1];

if double_integrator_flag
    if unstablePlantPoles(end) ~= 1
        disp('The final unstable plant pole must be z=1!');
        stop
    elseif length(find(unstablePlantPoles == 1)) > 1
  disp('There should only be one pole at z=1 included in unstablePlantPoles!');
        stop
    end
end

stablePlantPoles = [stableRealPlantPoles stableComplexPlantPoles];
qs = [stablePlantPoles unstablePlantPoles];

% coefficents go in order of the poles
%cs = [r(1)]
cs = [k_2*k_3*0.5*T^2];
if double_integrator_flag
    % coefficients include both c_n for 1/(z-1) and c_(n+1) for 1/(z-1)^2 for
    %       the pole at z=1
    %c_double_integrator = [r(2)];
    c_double_integrator = [k_2*k_3*T^2];
    cs = [cs c_double_integrator];
end     

n = length(qs);
nhat = length(stablePlantPoles);
nreal = length(stableRealPlantPoles);
ncomplex = length(stableComplexPlantPoles);

% verify that your plant is correct!
z = tf('z',T);
G = 0;
for k=1:n
    G = G + cs(k)/(z-qs(k));
end
if double_integrator_flag
    G = G + c_double_integrator/(z-1)^2;
end

%G = G_justin
%G_orig = G_justin
% plant_error = norm(G_orig - G);
% disp(['Norm of (G_orig - G_check) = ', num2str(plant_error)]);
% if plant_error > 1e-10
%     disp('WARNING: Plant mismatch! G and G_check are not the same.');
%     disp('The optimization is based on a different plant than the verification.');
% end

%% Poles Chosen in the Simple Pole Approximation of W[z]

% j = sqrt(-1);
realWPoles = [];
complexWPoles = poleset;
% for checking the integrator in the controller:
%complexWPoles = poles_B';
ps = [realWPoles complexWPoles];

mreal = length(realWPoles);
mcomplex = length(complexWPoles);
m = length(ps);

%% Calculation of alpha, beta, gamma, and gamma hat

alpha = zeros(m);

for i=1:m
    for k=1:n
        alpha(i,i) = alpha(i,i) + cs(k)/(ps(i)-qs(k));
    end
    if double_integrator_flag
        alpha(i,i) = alpha(i,i) + cs(n+1)/((ps(i)-1)^2);
    end
end

beta = zeros(n,m);
if double_integrator_flag
    beta = zeros(n+1,m);
end

for i=1:m
    for k=1:n
        beta(k,i) = cs(k)/(qs(k)-ps(i));
    end
    if double_integrator_flag
        beta(n,i) = beta(n,i) - cs(n+1)/((1-ps(i))^2);
        beta(n+1,i) = cs(n+1)/(1-ps(i));
    end
end

gamma = zeros(n-nhat,m); %#ok<*PREALL>
if double_integrator_flag
    gamma = zeros(n+1-nhat,m);
end

for i=1:m
    for j=(nhat+1):n
        gamma(j-nhat,i) = cs(j)/(qs(j)-ps(i));
    end
    if double_integrator_flag
        gamma(n-nhat,i) = gamma(n-nhat,i) - cs(n+1)/((1-ps(i))^2);
        gamma(n+1-nhat,i) = cs(n+1)/(1-ps(i));
    end
end

gammaHat = zeros(n-nhat,nhat);
if double_integrator_flag
    gammaHat = zeros(n+1-nhat,nhat);
end

for k=1:nhat
    for j=(nhat+1):n
        gammaHat(j-nhat,k) = cs(j)/(qs(j)-qs(k));
    end
    if double_integrator_flag
        gammaHat(n-nhat,k) = gammaHat(n-nhat,k) - cs(n+1)/((1-qs(k))^2);
        gammaHat(n+1-nhat,k) = cs(n+1)/(1-qs(k));
    end
end

% verify on a simple example that alpha, beta, gamma, and gammahat are correct!
%alpha
%beta
%gamma
%gammaHat

%% Determination of A and b matrices for IOP equations

A = [alpha eye(m) zeros(m,nhat);
     beta [zeros(nhat,m) eye(nhat);
           zeros(size(beta,1)-nhat,m+nhat)];
     zeros(size(gamma)) gamma gammaHat];

b = [zeros(m+size(beta,1),1);
     -cs((nhat+1):end)'];

%% Determination of step response matrices

% time horizon
K = 300;

step_ry = zeros(K,m+nhat);

for k=1:K
    for i=1:m
        step_ry(k,i) = -(1-ps(i)^k)/(1-ps(i));
    end
    for j=1:nhat
        step_ry(k,m+j) = -(1-qs(j)^k)/(1-qs(j));
    end
end

step_ru = zeros(K,m);

for k=1:K
    for i=1:m
        step_ru(k,i) = (1-ps(i)^k)/(1-ps(i));
    end
end

% verify on a simple example that step_ry and step_ru are correct!
step_ry = step_ry*0.15;
step_ru = step_ru*0.15;

%% Determination of steady state vector

steadyState = zeros(1,m+nhat);
if controller_integrator_flag
    steadyState = zeros(3,m+nhat);
end

for i=1:m
    if ~controller_integrator_flag    
        steadyState(i) = 1/(1-ps(i));
    else
        steadyState(1,i) = 1/(1-ps(i));
        steadyState(2,i) = 1/(1-ps(i))^2;
        steadyState(3,i) = 1/(1-ps(i))^3;
    end
end

for k=1:nhat
    if ~controller_integrator_flag
        steadyState(m+k) = 1/(1-qs(k));
    else
        steadyState(1,m+k) = 1/(1-qs(k));
        steadyState(2,m+k) = 1/(1-qs(k))^2;
        steadyState(3,m+k) = 1/(1-qs(k))^3;
    end
end

% verify on a simple example that steadyState is correct!
steadyState = steadyState*0.15;

%% Defining the variables for the optimization

wreal = sdpvar(mreal,1,'full');
wcomplex = sdpvar(mcomplex/2,1,'full','complex');
w = wreal;
for i=1:(mcomplex/2)
    w = [w;
         wcomplex(i);
         conj(wcomplex(i))];
end

xreal = sdpvar(mreal,1,'full');
xcomplex = sdpvar(mcomplex/2,1,'full','complex');
x = xreal;
for i=1:(mcomplex/2)
    x = [x;
         xcomplex(i);
         conj(xcomplex(i))];
end

xhatreal = sdpvar(nreal,1,'full');
xhatcomplex = sdpvar(ncomplex/2,1,'full','complex');
xhat = xhatreal;
for i=1:(ncomplex/2)
    xhat = [xhat;
            xhatcomplex(i);
            conj(xhatcomplex(i))];
end


%% Defining the objective function and constraints for the optimization
%Objective = 0;
%Objective = max(step_ru*w);
%Objective = min(max(step_ru*w));
Objective_num = 3;
Objective = norm(step_ru*w, 2); 
%Objective = min(norm(step_ru*w))

%objective_num legend:
%1 --> %Objective = max(step_ru*w);
%2 --> %Objective = norm(step_ru*w, 2); 
%3 --> objective = min(norm(u))




% IOP constraint
Constraints = [A*[w;x;xhat] == b];

% input saturation constraint
Constraints = [Constraints, ...
              max(step_ru*w) <= max_U];
Constraints = [Constraints, ...
              min(step_ru*w) >= -max_U];   

% Constraints = [Constraints, ...
%                norm(step_ru*w, inf) <= 0.7];        

% steady state constraint
if ~controller_integrator_flag
    Constraints = [Constraints, ...
                   steadyState*[x;xhat]+0.15==0];
else
    Constraints = [Constraints, ...
                   steadyState*[x;xhat]+[1;0;0]*0.15==[0;0;0]];
end


% overshoot constraint
Constraints = [Constraints, ...
                max(step_ry*[x;xhat]) <= max_OS*(-steadyState(1,:)*[x;xhat])]; % 
% settling time constraint
jhat = floor(max_ts/T);

Constraints = [Constraints, ...
               max(step_ry(jhat:end,:)*[x;xhat]) <= ...
               1.02*(-steadyState(1,:)*[x;xhat]), ... 
               min(step_ry(jhat:end,:)*[x;xhat]) >= ...
               0.98*(-steadyState(1,:)*[x;xhat])]; 

%%BELOW IS BEFORE I ADDED THE INTEGRATOR CASE
% % overshoot constraint
% Constraints = [Constraints,
%                max(step_ry*[x;xhat]) <= max_OS*(-steadyState*[x;xhat])];
% 
% % settling time constraint
% jhat = floor(max_ts/T);
% Constraints = [Constraints,
%                max(step_ry(jhat:end,:)*[x;xhat]) <= ...
%                1.02*(-steadyState*[x;xhat]),
%                min(step_ry(jhat:end,:)*[x;xhat]) >= ...
%                0.98*(-steadyState*[x;xhat])];

%% Solving the optimization problem

% set some options for YALMIP and solver
options = sdpsettings('verbose',1,'solver','mosek');

% solve the problem
sol = optimize(Constraints,Objective,options);

% obtain the solution
wsol = value(w);
xsol = value(x);
xhatsol = value(xhat);

%% Plotting the solution

figure(1)
plot(T*(1:K),step_ry*[xsol;xhatsol]);
xlabel('Time [s]');
ylabel('y[k]');

figure(2)
plot(T*(1:K),step_ru*wsol);
xlabel('Time [s]');
ylabel('u[k]');

% use log scale for heat map?
log_scale_flag = 1;

% heat map
heatmap = figure(3);
t = linspace(0,2*pi);
plot(cos(t),sin(t),'k--');
hold on;
if log_scale_flag
    scatter(real(ps),imag(ps),50,log(abs(wsol)),'filled');
    scatter(real(qs(1:nhat)),imag(qs(1:nhat)),50,log(abs(xhatsol)),'filled');
else
    scatter(real(ps),imag(ps),50,abs(wsol),'filled');
    scatter(real(qs(1:nhat)),imag(qs(1:nhat)),50,abs(xhatsol),'filled');
end
hold off;
colormap(jet);
colorbar;
%saveas(heatmap, "Controller_Trial_Figures/Integ_attempt_"+attempt_number+"_polemap.png")

%% Recover the transfer functions

z = tf('z',T);

% calculate W
W = 0;
for i=1:m
    W = W + wsol(i)/(z-ps(i));
end

% calculate X
X = 1;
for i=1:m
    X = X + xsol(i)/(z-ps(i));
end
for k=1:nhat
    X = X + xhatsol(k)/(z-qs(k));
end

% remove the imaginary coefficients in W
[num,den] = tfdata(W);
num{1} = real(num{1});
den{1} = real(den{1});
W = tf(num,den,T);

% remove the imaginary coefficients in X
[num,den] = tfdata(X);
num{1} = real(num{1});
den{1} = real(den{1});
X = tf(num,den,T);

% find the poles and zeros of W and X (if desired)
%zpk(W)
%zero(W)
%pole(W)
%zpk(X)
%zero(X)
%pole(X)
%% Verify design in discrete time


% compute D by hand
j = sqrt(-1);
D = recoverD(W, X, T, 0.00001);

% compute T_ry and T_ru by hand
%

% 3. Calculate T_ry and T_ru using the feedback formulas:
T_ry = feedback(G_orig*D, 1);
T_ru = feedback(D, G_orig); %these are not transfer functions they're zpg 

%By default, step applies an input signal that changes from 0 to 1 at t = 0. 
%To customize the amplitude and bias, use RespConfig. 
%For instance, compute the response of a SISO state-space model 
%to a signal that changes from 1 to –1 to at t = 0.
%opt = stepDataOptions('StepAmplitude', 0.15, 'InputOffset', -0.7);
opt = stepDataOptions('StepAmplitude', 0.15);

%opt.Bias = -0.7;
%opt.Amplitude = 1.4;

y_out = figure(1);
yname = "Controller_Trial_Figures/Integ_attempt_"+attempt_number+"_Y.png";
hold on;
step(T_ry, opt, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
%saveas(y_out, yname);
hold off;

u_out = figure(2);
uname = "Controller_Trial_Figures/Integ_attempt_"+attempt_number+"_U.png";
hold on;
step(T_ru, opt, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
%saveas(u_out, uname);
hold off;
%% --- Generate Performance Metrics  ---
disp('Calculating performance metrics...');

% 1. Get step response data and info for T_ry (output Y)
% We use stepinfo to get standard metrics like settling time and peak
S_y = stepinfo(T_ry, 'StepAmplitude', ref_amplitude);
[y_data, ~] = step(T_ry, opt); % Get raw data for final value
Y_final = y_data(end);

% 2. Get step response data for T_ru (input U)
[u_data, ~] = step(T_ru, opt);

Metric_Ess = ref_amplitude - Y_final; %steady state error
Metric_UMax = max(abs(u_data)); %maximum ctrl input
Metric_YMax = S_y.Peak; %max output
Metric_OS = S_y.Overshoot;
Metric_SettlingTime = S_y.SettlingTime;
Metric_UTotalVariation = sum(abs(diff(u_data))); % 5. Oscillation/Variability of U (using Total Variation)
Metric_NumPoles = m; 
Metric_ObjectiveValue = value(Objective);

%% --- Assemble and Save Table ---

% Define metric names (will be column headers)
metricNames = {
    'Constraint_Max_U', ...
    'Constraint_Max_OS', ...
    'Constraint_Max_TS', ...
    'Constraint_ObjectiveValue', ...
    'Constraint_NumPoles', ...
    'RESULT_Ess', ...
    'RESULT_UMax', ...
    'RESULT_OS', ...
    'RESULT_SettlingTime', ...
    'RESULT_UTotalVariation',
    };

% Collect all values into a single column vector
allValues = [
    max_U;   
    max_OS-1; 
    max_ts;
    Objective_num;
    Metric_NumPoles;
    Metric_Ess;
    Metric_UMax;
    Metric_OS;
    Metric_SettlingTime;
    Metric_UTotalVariation;
    ];

% --- NEW LOGIC: Load, Update, Save (Rows = Attempts, Cols = Metrics) ---

% 1. Define the file name and the new ROW name
newRowName = sprintf('Attempt_%d', attempt_number); % Use char ' (not string ")

% 2. Create a new table for THIS attempt's data
% We transpose allValues to be a row and use array2table
newAttemptTable = array2table(allValues', 'VariableNames', metricNames, 'RowNames', {newRowName});

% 3. Check if the master results file already exists
if isfile(resultsFile)
    % 4a. If it exists, load the master table
    load(resultsFile, 'resultsTable');
    
    % --- Check for Mismatched Columns ---
    % Get column names from existing and new tables
    existingCols = resultsTable.Properties.VariableNames;
    newCols = newAttemptTable.Properties.VariableNames;
    
    % Find columns in new that are not in existing
    colsToAdd = setdiff(newCols, existingCols);
    if ~isempty(colsToAdd)
        fprintf('Adding new columns to master table: %s\n', strjoin(colsToAdd, ', '));
        % Add new columns to the old table, fill with NaN
        for i = 1:length(colsToAdd)
            resultsTable.(colsToAdd{i}) = nan(height(resultsTable), 1);
        end
    end
    
    % Find columns in existing that are not in new (e.g., if you removed one)
    colsToFill = setdiff(existingCols, newCols);
    if ~isempty(colsToFill)
         fprintf('Filling missing columns in new attempt: %s\n', strjoin(colsToFill, ', '));
         for i = 1:length(colsToFill)
            newAttemptTable.(colsToFill{i}) = nan;
         end
    end
    
    % Re-order columns in new table to match existing one before appending
    newAttemptTable = newAttemptTable(:, resultsTable.Properties.VariableNames);
    % --- End Mismatch Check ---

    % Check if this row already exists (e.g., if you re-ran the same number)
    if ismember(newRowName, resultsTable.Properties.RowNames)
        % Overwrite the existing row
        fprintf('Warning: Attempt %d already exists. Overwriting row.\n', attempt_number);
        resultsTable(newRowName, :) = newAttemptTable;
    else
        % Add the new data as a new row
        resultsTable = [resultsTable; newAttemptTable];
    end
else
    % 4b. If it doesn't exist, this is the first attempt.
    % The master table is just this one new attempt.
    resultsTable = newAttemptTable;
end

% 5. Display the updated table in the command window
disp('--- Performance Results ---');
disp(resultsTable);

% 6. Save the updated master table back to the .mat file
% 6. Save the updated master table AND the transfer functions
fprintf('Saving results table and transfer functions...\n');

% --- Create a field name based on the attempt number ---
attemptField = sprintf('Attempt_%d', attempt_number);

% --- Check if the results file already has transfer functions ---
if isfile(resultsFile)
    % Load *just* the tf_results variable, if it exists
    vars = who('-file', resultsFile);
    if ismember('tf_results', vars)
        load(resultsFile, 'tf_results');
    else
        % If it doesn't exist, create an empty struct
        tf_results = struct();
    end
else
    % If the file doesn't exist at all, create an empty struct
    tf_results = struct();
end

% --- Add the new TFs and their step options to the struct ---
tf_results.(attemptField).T_ry = T_ry;
tf_results.(attemptField).T_ru = T_ru;
tf_results.(attemptField).opt = opt; % Save the 'opt' used for this step
tf_results.(attemptField).D = D;

% --- Save both variables back to the .mat file ---
save(resultsFile, 'resultsTable', 'tf_results');


% 7. Also update the LaTeX export to use the new table orientation
try
    table2latex(resultsTable);
    fprintf('Results table saved to %s\n', resultsFile);
catch
    fprintf('Results table not saved, skipping. Is table2latex available?\n')
end



function D = recoverD(W, X, T, tol)
%RECOVERD  
[zW,pW,kW] = zpkdata(W,'v');
[zX,pX,kX] = zpkdata(X,'v');

zD = [zW(:); pX(:)];%a/b/c/d = ad/bc
pD = [pW(:); zX(:)]; %a/b/c/d = ad/bc
kD = kW / kX;
cancelled = []; %taylor swift reference
format long
for i = 1:numel(zD)
    [mindiff, j] = min(abs(pD - zD(i)));
    if mindiff < tol
        
        %%uncomment this block to do manual cancellation
%         disp('Looks like you could cancel these poles:')
%         disp(pD(j))
%         disp(' and ')
%         disp(zD(i))
%         decision = input(' cancel them? 0/1')
%         
%         if decision == 1
%             cancelled(end+1,:) = [zD(i), pD(j)];  %#ok<*AGROW>
%             pD(j) = NaN; zD(i) = NaN; %goodbye
%         end
        
        cancelled(end+1,:) = [zD(i), pD(j)];  %#ok<*AGROW>
        pD(j) = NaN; zD(i) = NaN; %goodbye
        
    end
end

% Print cancelled pairs [vibecoded print statements]
disp('Cancelled pole-zero pairs:')
if isempty(cancelled)
    disp('(none)');
else
    for i = 1:size(cancelled,1)
        fprintf('  (z - %.6g)  with  (z - %.6g)\n', cancelled(i,1), cancelled(i,2));
    end
end

zD = zD(~isnan(zD));
pD = pD(~isnan(pD));
D = zpk(zD, pD, kD, T);

% Show result
disp('Final simplified D(z):')
zpk(D)
end

