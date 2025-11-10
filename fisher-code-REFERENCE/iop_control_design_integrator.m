
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This template is for the IOP control design with SPA with a double
% integrator in the plant, and is incomplete.
% You need to complete it by replacing every * with the correct code.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set time step
T = 0.001;
total = 40;
rmax = 0.6;
center = 0;
% does the plant have a double integrator?
double_integrator_flag = 1;

% should the controller have an integrator?
controller_integrator_flag = 0;
% --- choose bogus discrete poles you want in z ---
zb = [0.12];                    % put any |z|<1 values here
a_bogus = -log(zb)/T;           % continuous rates

s = tf('s');
P = 1/s^2;
for ai = a_bogus
    P = P * (1/(s + ai));
end

G = c2d(P, T, 'zoh');

% === Build qs, cs, c_double_integrator from G automatically ===
[num, den] = tfdata(G, 'v');
[r, p, k]  = residuez(num, den);

tol = 0.001;
idx1 = find(abs(p - 1) < tol);
if numel(idx1) ~= 2
    error('Expected a double pole at z=1; found multiplicity %d.', numel(idx1));
end

syms zsym
Gsym = poly2sym(num, zsym) / poly2sym(den, zsym);
c2 = double(limit((zsym-1)^2 * Gsym, zsym, 1));           % (z-1)^(-2) coeff
c1 = double(limit(diff((zsym-1)^2 * Gsym, zsym), zsym, 1));% (z-1)^(-1) coeff

keep = true(size(p));
keep(idx1) = false;                        % drop the two entries at z=1
p_stable = p(keep);
r_stable = r(keep);

stableRealPlantPoles    = p_stable(imag(p_stable)==0).';
stableComplexPlantPoles = p_stable(imag(p_stable)~=0).';
unstablePlantPoles      = 1;

stablePlantPoles = [stableRealPlantPoles stableComplexPlantPoles];
qs = [stablePlantPoles unstablePlantPoles];    % simple-pole list

cs = [r_stable.' c1];                          % simple residues (ends with z=1)
c_double_integrator = c2;                      % double-pole residue

n      = length(qs);
nhat   = length(stablePlantPoles);
nreal  = length(stableRealPlantPoles);
ncomplex = length(stableComplexPlantPoles);

% quick verification
z = tf('z', T);
G_verify = 0;
for k_i = 1:n
    G_verify = G_verify + cs(k_i)/(z - qs(k_i));
end
G_verify = G_verify + c_double_integrator/(z - 1)^2;
disp(minreal(G - G_verify));   % should print 0 (or numerically tiny)

%% Poles Chosen in the Simple Pole Approximation of W[z]

realWPoles = [];
complexWPoles = [generate_poles(total,rmax,center)];
ps = [realWPoles complexWPoles];

mreal = length(realWPoles);
mcomplex = length(complexWPoles);
m = length(ps);

%% Calculation of alpha, beta, gamma, and gammaHat (general nhat)

alpha = zeros(m,m);

% Locate the index of the simple pole at z=1 inside qs (if present)
idx_one = find(abs(qs - 1) < 1e-12, 1);   % [] if not present

% ---------- ALPHA (m x m diagonal with G(p_i)) ----------
for i = 1:m
    % Sum simple-pole terms
    for k = 1:n
        alpha(i,i) = alpha(i,i) + cs(k)/(ps(i) - qs(k));
    end
    % Add the double-integrator term c2/(p_i - 1)^2 if enabled
    if double_integrator_flag
        alpha(i,i) = alpha(i,i) + c_double_integrator/(ps(i) - 1)^2;
    end
end

% Base rows for each simple pole in qs
beta = zeros(n, m);

for i = 1:m
    for j = 1:n
        % Base contribution: c_j / (q_j - p_i)
        beta(j,i) = cs(j)/(qs(j) - ps(i));
    end
    if double_integrator_flag && ~isempty(idx_one)
        % For the row corresponding to q_j = 1, subtract c2/(1 - p_i)^2
        beta(idx_one,i) = beta(idx_one,i) - c_double_integrator/(1 - ps(i))^2;
    end
end

% Extra row for the (z-1)^(-2) residue when double-integrator is present
if double_integrator_flag
    beta = [beta; zeros(1,m)];            % now (n+1) x m
    for i = 1:m
        beta(end,i) = c_double_integrator/(1 - ps(i));
    end
end

% Unstable poles are j = nhat+1 : n  → (n - nhat) rows
gamma = zeros(n - nhat, m);

for i = 1:m
    row = 0;
    for j = (nhat+1):n
        row = row + 1;
        % Base contribution: c_j / (q_j - p_i)
        gamma(row,i) = cs(j)/(qs(j) - ps(i));
        if double_integrator_flag && ~isempty(idx_one) && j == idx_one
            gamma(row,i) = gamma(row,i) - c_double_integrator/(1 - ps(i))^2;
        end
    end
end

% Extra bottom row for (z-1)^(-2) term if enabled
if double_integrator_flag
    gamma = [gamma; zeros(1,m)];          % now (n - nhat + 1) x m
    for i = 1:m
        gamma(end,i) = c_double_integrator/(1 - ps(i));
    end
end

gammaHat = zeros(n - nhat, nhat);

for k = 1:nhat                       % columns over stable poles
    row = 0;
    for j = (nhat+1):n               % rows over unstable poles
        row = row + 1;
        % Base: c_j / (q_j - q_k)
        gammaHat(row,k) = cs(j)/(qs(j) - qs(k));
        % If the unstable pole is at 1, include derivative coupling on that row
        if double_integrator_flag && ~isempty(idx_one) && j == idx_one
            gammaHat(row,k) = gammaHat(row,k) - c_double_integrator/(1 - qs(k))^2;
        end
    end
end

% If double-integrator, add the extra bottom row for (z-1)^(-2)
if double_integrator_flag
    gammaHat = [gammaHat; zeros(1, nhat)];    % now (n - nhat + 1) x nhat
    for k = 1:nhat
        gammaHat(end,k) = c_double_integrator/(1 - qs(k));
    end
end

% Quick prints
alpha
beta
gamma
gammaHat


%% Determination of A and b matrices for IOP equations


A = [alpha, eye(m),               zeros(m, nhat);
     gamma, zeros(size(gamma,1), m), gammaHat];

b = [zeros(m,1);
     zeros(size(gamma,1),1)];   % b_unstable = 0 for W(1)=0, W'(1)=0


%% Determination of step response matrices

% time horizon
K = 100;

step_ry = zeros(K,m+nhat);

for k=1:K
    for i=1:m
        % This is the k-th sample of the step response of -1/(z-p_i)
        % This implies Y(z) = 1 - X(z)
        step_ry(k,i) = -(1-ps(i)^k)/(1-ps(i));
    end
    for j=1:nhat
        % This is the k-th sample of the step response of -1/(z-q_j)
        step_ry(k,m+j) = -(1-qs(j)^k)/(1-qs(j));
    end
end

step_ru = zeros(K,m);

for k=1:K
    for i=1:m
        % This is the k-th sample of the step response of 1/(z-p_i)
        % This implies u[k] is the step response of W(z)
        step_ru(k,i) = (1-ps(i)^k)/(1-ps(i));
    end
end

% verify on a simple example that step_ry and step_ru are correct!
step_ry
step_ru

%% Determination of steady state vector

steadyState = zeros(1,m+nhat);
if controller_integrator_flag
    % We need 3 rows for X(1)=0, X'(1)=0, X''(1)=0
    steadyState = zeros(3,m+nhat);
end

for i=1:m
    if ~controller_integrator_flag
        % Corresponds to X(1)=0
        % steadyState(i) is the coefficient for x_i in this sum
        steadyState(i) = 1/(1-ps(i));
    else
        % Row 1: X(1)=0 constraint
        steadyState(1,i) = 1/(1-ps(i));
        % Row 2: X'(1)=0 constraint
        steadyState(2,i) = -1/(1-ps(i))^2;
        % Row 3: X''(1)=0 constraint
        steadyState(3,i) = 2/(1-ps(i))^3;
    end
end

for k=1:nhat
    if ~controller_integrator_flag
        % Corresponds to X(1)=0
        % steadyState(m+k) is the coefficient for xhat_k in this sum
        steadyState(m+k) = 1/(1-qs(k));
    else
        % Row 1: X(1)=0 constraint
        steadyState(1,m+k) = 1/(1-qs(k));
        % Row 2: X'(1)=0 constraint
        steadyState(2,m+k) = -1/(1-qs(k))^2;
        % Row 3: X''(1)=0 constraint
        steadyState(3,m+k) = 2/(1-qs(k))^3;
    end
end

% verify on a simple example that steadyState is correct!
steadyState

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
Objective = 0;

% IOP constraint
% This comes directly from A*vars = b
% A is (m+n)x(m+m+nhat), [w;x;xhat] is (m+m+nhat)x1, b is (m+n)x1
Constraints = [A*[w;x;xhat] == b];

% --- NOTE: The constraints below assume umax, y_ss, etc. ---
% You may need to change these values for your specific problem.
umax = 1.0;         % Max control input
ymax_overshoot = 1.05; % Max overshoot (e.g., 5% over y_ss)
settling_band = 0.02;  % Settling band (e.g., +/- 2%)
y_ss = 1;           % Desired steady-state output
jhat = 50;          % Settling time (in samples)

% input saturation constraint |u(k)| <= umax
Constraints = [Constraints,
               step_ru*w <= umax,
               step_ru*w >= -umax];

% steady state constraint
if ~controller_integrator_flag
    % For y_ss = 1, and y_ss = -steadyState * [x;xhat]
    % We need -steadyState * [x;xhat] == 1, or steadyState * [x;xhat] == -1
    Constraints = [Constraints,
                   steadyState*[x;xhat] == -y_ss];
else
    % For a controller integrator, we need X(1)=0 for disturbance rejection
    % X(1) = 1 + steadyState * [x;xhat]
    % So the constraint is 1 + steadyState * [x;xhat] == 0
    Constraints = [Constraints,
                   1 + steadyState*[x;xhat] == 0];
    % You may need to add X'(1) constraints here
end

% overshoot constraint
% y(k) <= y_max
Constraints = [Constraints,
               step_ry*[x;xhat] <= ymax_overshoot];

% settling time constraint
% y(k) must be in [y_ss-band, y_ss+band] after k=jhat
Constraints = [Constraints,
               step_ry(jhat:end,:)*[x;xhat] <= y_ss + settling_band,
               step_ry(jhat:end,:)*[x;xhat] >= y_ss - settling_band];
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
%fixPlot(1);

figure(2)
plot(T*(1:K),step_ru*wsol);
xlabel('Time [s]');
ylabel('u[k]');
%fixPlot(2);

% use log scale for heat map?
log_scale_flag = 1;

% heat map
figure(3)
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
%fixPlot(3);

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
zpk(W)
zero(W)
pole(W)
zpk(X)
zero(X)
pole(X)
%% Verify design in discrete time

% compute D by hand
j = sqrt(-1);
D = recoverD(W, X, T, 0.01);

% compute T_ry and T_ru by hand
%

% 3. Calculate T_ry and T_ru using the feedback formulas:
T_ry = feedback(G*D, 1);
T_ru = feedback(D, G); %these are not transfer functions they're zpg 
% 
% 
figure(1)
hold on;
step(T_ry, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
hold off;

figure(2)
hold on;
step(T_ru, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
hold off;



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

