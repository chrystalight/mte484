clear
clc
load 'poles_B.mat'

% set time step
k_2 = 0.06091;
k_3 = -4.0162;
T = 0.25; %%WE HAVE TO PICK A NEW T VALUE

s = tf('s')
P = (k_2*k_3)/(s*s)
G = c2d(P, T, 'zoh')

[num, den] = tfdata(G, 'v');
[r, p, k] = residuez(num, den);

%Pole Picking Parameters
total = 40;
rmax = 0.9;
center = 0;


% does the plant have a double integrator?
double_integrator_flag = 1;

% should the controller have an integrator?
controller_integrator_flag = 0;

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
cs = [r(1)]

if double_integrator_flag
    % coefficients include both c_n for 1/(z-1) and c_(n+1) for 1/(z-1)^2 for
    %       the pole at z=1
    c_double_integrator = [r(2)];
    cs = [cs c_double_integrator];
end     

n = length(qs);
nhat = length(stablePlantPoles);
nreal = length(stableRealPlantPoles);
ncomplex = length(stableComplexPlantPoles);

% verify that your plant is correct!
z = tf('z',T);
G_check = 0;
for k=1:n
    G_check = G_check + cs(k)/(z-qs(k));
end
if double_integrator_flag
    G_check = G_check + c_double_integrator/(z-1)^2;
end
G_check

%% Poles Chosen in the Simple Pole Approximation of W[z]

j = sqrt(-1);
realWPoles = [];
complexWPoles = [generate_poles(total,rmax,center)];
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

gamma = zeros(n-nhat,m);
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
K = 3000;

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

%Objective = max(step_ru*w); %%% ATTEMPT 3
Objective = norm(step_ru*w, 2); %%% ATTEMPT 4

% IOP constraint
Constraints = [A*[w;x;xhat] == b];

% input saturation constraint
Constraints = [Constraints,
              max(step_ru*w) <= 0.65];
Constraints = [Constraints,
              min(step_ru*w) >= -0.65];       
% Constraints = [Constraints, ...
%                norm(step_ru*w, inf) <= 0.7];        

% steady state constraint
if ~controller_integrator_flag
    Constraints = [Constraints,
                   steadyState*[x;xhat]+0.15==0];
else
    Constraints = [Constraints,
                   steadyState*[x;xhat]+[1;0;0]*0.15==[0;0;0]];
end

% overshoot constraint
Constraints = [Constraints,
               max(step_ry*[x;xhat]) <= 1.45*(-steadyState*[x;xhat])];

% settling time constraint
jhat = floor(7/T);
Constraints = [Constraints,
               max(step_ry(jhat:end,:)*[x;xhat]) <= ...
               1.02*(-steadyState*[x;xhat]),
               min(step_ry(jhat:end,:)*[x;xhat]) >= ...
               0.98*(-steadyState*[x;xhat])];

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
D = recoverD(W, X, T, 0.0001);

% compute T_ry and T_ru by hand
%

% 3. Calculate T_ry and T_ru using the feedback formulas:
T_ry = feedback(G*D, 1);
T_ru = feedback(D, G); %these are not transfer functions they're zpg 


%By default, step applies an input signal that changes from 0 to 1 at t = 0. 
%To customize the amplitude and bias, use RespConfig. 
%For instance, compute the response of a SISO state-space model 
%to a signal that changes from 1 to –1 to at t = 0.
%opt = stepDataOptions('StepAmplitude', 0.15, 'InputOffset', -0.7);
opt = stepDataOptions('StepAmplitude', 0.15);

%opt.Bias = -0.7;
%opt.Amplitude = 1.4;

figure(1)
hold on;
step(T_ry, opt, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
hold off;

figure(2)
hold on;
step(T_ru, opt, 'g--'); % <-- Made the line a dashed green 'g--' to see it better
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

