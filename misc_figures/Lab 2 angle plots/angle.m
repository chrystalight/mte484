clear
close all

f_phi = @(x)(sign(x) .* acosd((2.50*cosd(x)+40.5)/43));

th = linspace(-45, 45, 501);
phi = f_phi(th);
th_long = linspace(-180, 180, 1001);
phi_long = f_phi(th_long);

figure(1)
hold on;
plot(th_long, f_phi(th_long));
plot(th, phi);
xlabel("\theta angle (degrees)")
ylabel("\phi angle (degrees)")
title("\theta-\phi angle plot")
legend("Full range", "Valid range");

f_lerp = @(x)(interp1([min(th) max(th)], [min(phi) max(phi)], x));

figure(2)
hold on;
plot(th, f_lerp(th)-phi);
xlabel("\theta angle (degrees)")
ylabel("Error (degrees)")
title("Error in linearization of \theta")

K_2 = (max(phi)-min(phi)) / (max(th)-min(th))