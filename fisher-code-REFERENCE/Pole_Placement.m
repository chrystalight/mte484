
%% Pole Placement Control Design

% fill in the *'s to complete this code

T = 0.01;
G = *
G

%% Choose 4 desired poles which will contain the poles of all the closed-loop
%%         transfer functions

j = sqrt(-1);
desired_poles = [* *]

%% Compute the controller D[z]

error_flag = 0;

if length(desired_poles) ~= 4
    disp('The number of poles must be exactly equal to 4!');
    error_flag = 1;
end

real_pole_indeces = [];
complex_pole_indeces = [];

for ii=1:length(desired_poles)
    if imag(desired_poles(ii)) ~= 0
        complex_pole_indeces = [complex_pole_indeces ii];
        found_conjugate = 0;        
        for jj=[1:(ii-1) (ii+1):4]
            if (real(desired_poles(jj)) == real(desired_poles(ii))) && ...
               (imag(desired_poles(jj)) == -imag(desired_poles(ii)))
                found_conjugate = 1;
            end
        end
        if ~found_conjugate
            disp(['The pole ' num2str(desired_poles(ii)) ...
                  'has no complex conjugate!']);
            error_flag = 1;
        end
    else
        real_pole_indeces = [real_pole_indeces ii];
    end
end

if ~error_flag

% compute A matrix based on the plant
A = [1     0      0    0    0    0;
     1.6   1      0    1    0    0;
     0.55  1.6    1   1.5   1    0;
       0   0.55  1.6   0   1.5   1;
       0     0   0.55  0    0   1.5;
       1     1    1    0    0    0];

% compute b matrix based on desired poles
if length(real_pole_indeces) == 0
    p1 = desired_poles(1);
    if (real(desired_poles(2)) == real(desired_poles(1))) && ...
        (imag(desired_poles(2)) == -imag(desired_poles(1)))
        p2 = desired_poles(3);
    else
        p2 = desired_poles(2);
    end

    rp1 = real(p1);
    rp2 = real(p2);
    ap1 = abs(p1)^2;
    ap2 = abs(p2)^2;
        
    b = [1;
         -2*(rp1+rp2);
         4*rp1*rp2 + ap1 + ap2;
         -2*(rp1*ap2 + rp2*ap1);
         ap1*ap2;
         0];
elseif length(real_pole_indeces) == 2
    p1 = desired_poles(complex_pole_indeces(1));
    p2 = desired_poles(real_pole_indeces(1));
    p3 = desired_poles(real_pole_indeces(2));

    rp1 = real(p1);
    ap1 = abs(p1)^2;

    b = [1;
         -(2*rp1+p2+p3);
         2*rp1*(p2+p3) + p2*p3 + ap1;
         -(2*rp1*p2*p3 + ap1*(p2+p3));
         ap1*p2*p3;
         0];
else
    p1 = desired_poles(1);
    p2 = desired_poles(2);
    p3 = desired_poles(3);
    p4 = desired_poles(4);
    
    b = [1;
         -(p1+p2+p3+p4);
         (p1+p2)*(p3+p4)+p1*p2+p3*p4;
         -((p1+p2)*p3*p4 + (p3+p4)*p1*p2);
         p1*p2*p3*p4;
         0];
end

% solve for coefficients in D[z]
x = A\b;
D = tf([x(4) x(5) x(6)],[x(1) x(2) x(3)],T);

%% Evaluate the performance of the control design

Try = feedback(G*D,1);
Tru = feedback(D,G);
Tfinal = 10;
[y,ty] = step(Try,Tfinal);
[u,tu] = step(Tru,Tfinal);
yss = y(end);
ess = 1 - yss;
percent_OS = 100*((max(y)-yss)/(yss-y(1)));
ydeviation = 100*abs((y-yss)/(yss-y(1)));
temp = find(ydeviation >= 2);
settling_index = temp(end)+1;
Ts = ty(settling_index);
umax = max(u);

end
