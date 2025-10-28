
% define the plant and controller in continuous time using the command tf
P = ;
C = ;

% compute the closed-loop transfer function from r to y
cont_sys = feedback(P*C,1);

% set the duration of the simulation in seconds
tfinal = 10;

% choose the time steps at which to perform the simulation
t_cont = linspace(0,tfinal,1000);

% simulate the step response of y(t)
y_cont = step(cont_sys,t_cont);

% calculate e(t)
e_cont = 1-y_cont;

% simulate to find u(t)
u_cont = lsim(C,e_cont,t_cont);
