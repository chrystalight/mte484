
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This template is for the IOP control design with SPA, and is incomplete.
% You need to complete it by replacing every * with the correct code.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cla

% set time step
T = 0.02;

%% Plant Poles and Coefficients in its Partial Fraction Decomposition

stableRealPlantPoles = [];
stableComplexPlantPoles = [complex(1/4, 1/4), complex(1/4, -1/4)];
unstablePlantPoles = [-2];

stablePlantPoles = [stableRealPlantPoles stableComplexPlantPoles];
qs = [stablePlantPoles unstablePlantPoles];

% coefficents go in order of the poles
cs = [complex(1, 1), complex(1, -1), 2];

n = length(qs);
nhat = length(stablePlantPoles);
nreal = length(stableRealPlantPoles);
ncomplex = length(stableComplexPlantPoles);

% verify that your plant is correct!
G = 0;
for k=1:n
    G = G + tf([cs(k)], [1, -qs(k)], T);
end
G;

%% Poles Chosen in the Simple Pole Approximation of W[z]

realWPoles = [1/4];
complexWPoles = [complex(1/2, 1/2) complex(1/2, -1/2)];
ps = [realWPoles complexWPoles];

mreal = length(realWPoles);
mcomplex = length(complexWPoles);
m = length(ps);

%% Calculation of alpha, beta, gamma, and gamma hat

alpha = zeros(m);

for i=1:m
    for k=1:n
        alpha(i,i) = alpha(i,i) + (cs(k) / (ps(i) - qs(k)));
    end
end

beta = zeros(n,m);

for i=1:m
    for k=1:n
        beta(k,i) = (cs(k) / (qs(k) - ps(i)));
    end
end

gamma = zeros(n-nhat,m);

for i=1:m
    for j=(nhat+1):n
        gamma(j-nhat,i) = (cs(j) / (qs(j) - ps(i)));
    end
end

gammaHat = zeros(n-nhat,nhat);

for k=1:nhat
    for j=(nhat+1):n
        gammaHat(j-nhat,k) = (cs(j) / (qs(j) - qs(k)));
    end
end

% verify on a simple example that alpha, beta, gamma, and gammahat are correct!
alpha;
beta;
gamma;
gammaHat;

%% Determination of A and b matrices for IOP equations

A = [alpha eye(m) zeros(m,nhat);
     beta [zeros(nhat,m) eye(nhat);
           zeros(size(beta,1)-nhat,m+nhat)];
     zeros(size(gamma)) gamma gammaHat];

b = [zeros(m+size(beta,1),1);
     -cs(nhat+1:n).'];

%% Determination of step response matrices

% time horizon
K = 500;

step_ry = zeros(K,m+nhat);

for k=1:K
    for i=1:m
        step_ry(k,i) = -(1 - ps(i)^k) / (1 - ps(i));
    end
    for j=1:nhat
        step_ry(k,m+j) = -(1 - qs(j)^k) / (1 - qs(j));
    end
end

step_ru = zeros(K,m);

for k=1:K
    for i=1:m
        step_ru(k,i) = (1 - ps(i)^k) / (1 - ps(i));
    end
end

% verify on a simple example that step_ry and step_ru are correct!
step_ry;
step_ru;

%% Determination of steady state vector

steadyState = zeros(1,m+nhat);

for i=1:m
    steadyState(i) = 1 / (1 + ps(i));
end

for k=1:nhat
    steadyState(m+k) = 1 / (1 + qs(k));
end

% verify on a simple example that steadyState is correct!
steadyState;

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

MaxControlEffort = 4.5;
MaxSteadyStateError = 0.01;
MaxOvershoot = 0.5;
MaxSettlingTime = 0.5;

JHat = MaxSettlingTime/T;

y = step_ry * [x ; xhat];
u = step_ru*w;
Objective = (norm(y(1:end), 2)^2) + (norm(u(1:end), 2)^2);
% Objective = min(max(step_ry * [x ; xhat]));
% Objective = norm(step_ry * [x ; xhat], 'inf');

% IOP constraint
Constraints = [A * [w ; x ; xhat] == b];

% input saturation constraint
Constraints = [Constraints, ...
               norm(step_ru*w, inf) <= MaxControlEffort];

% steady state error constraint
Constraints = [Constraints, ...
               abs(1 + (steadyState * [x ; xhat])) <= MaxSteadyStateError];

% overshoot constraint
Constraints = [Constraints,
               norm(step_ry * [x ; xhat], inf) <= ...
                (1 + MaxOvershoot) * (-steadyState * [x ; xhat])];

% settling time constraint
Constraints = [Constraints,
                max(step_ry(JHat:end, :) * [x ; xhat]) ...
                    <= 1.02 * (-steadyState * [x ; xhat]), ...
                min(step_ry(JHat:end, :) * [x ; xhat]) ...
                    >= 0.98 * (-steadyState * [x ; xhat])];

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
    scatter(real(qs(1:nhat)),imag(qs(1:nhat)),50,log(abs(xhatsol)), '*');
else
    scatter(real(ps),imag(ps),50,abs(wsol),'filled');
    scatter(real(qs(1:nhat)),imag(qs(1:nhat)),50,abs(xhatsol),'*');
end
hold off;
colormap(jet);
colorbar;
legend('', 'Selected poles', 'Plant poles')

%% Recover the transfer functions

z = tf('z',T);

% calculate W
W = 0;
for i=1:m
    W = W + (wsol(i) / (z - ps(i)));
end

% calculate X
X = 1;
for i=1:m
    X = X + (xsol(i) / (z - ps(i)));
end
for k=1:nhat
    X = X + (xhatsol(k) / (z - qs(k)));
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

% find the poles and zeros of W and X
zpkW = zpk(W);
zerosW = zero(W);
polesW = pole(W);
zpkX = zpk(X);
zerosX = zero(X);
polesX = pole(X);

% %% Verify design in discrete time
% 
% % compute D by hand
% j = sqrt(-1);
% % D = (z-0.4)*(z-0.4-0.4j)*(z-0.4+0.4)*(z-0.5) / ...
% %     ( (z-0.4467-1.558j)*(z-0.4467+1.558j)*(z-0.406)*(z-0.4-0.4j)*(z-0.4+0.4j) );
% D = (-2.271) * ( (z-0.4)*(z-0.5) ) / ...
%     ( (z-0.4467-1.558j)*(z-0.4467+1.558j)*(z-0.406) );
% 
% % compute T_ry and T_ru by hand
% T_ry = 1-X;
% T_ru = W;
% 
% figure(4)
% hold on;
% step(T_ry);
% hold off;
% 
% figure(5)
% hold on;
% step(T_ru);
% hold off;
% 

% assert(all(sum(cs ./ (ps.' - qs), 2) == diag(alpha), "all"))
% assert(all(cs.' ./ (qs.' - ps) == beta, "all"))
% assert(all(cs(nhat+1:n) ./ (qs(nhat+1:n) - ps) == gamma, "all"))
% assert(all(cs(nhat+1:n) ./ (qs(nhat+1:n) - qs(1:nhat)) == gammaHat, "all"))