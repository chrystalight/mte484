
function poles = generate_random_poles(total,rmax,center)
    
    if mod(total,2) ~= 0
        disp(['An odd number of poles was specified.  The number of ' ...
              'poles generated must be even.']);
        stop
    end
    
    rs = zeros(1,total);
    thetas = zeros(1,total);
    
    for k=1:(total/2)
        rk = rand() * rmax; %random radius, [-rmax, +rmax]
        thetak = rand() * (2 * pi) - pi; %random angle, [-pi, +pi]
        % rk = rmax*sqrt(k/((total/2)));
        % thetak = 2*sqrt(pi*k);
        reals(2*(k-1)+1) = rk*cos(thetak);
        imags(2*(k-1)+1) = rk*sin(thetak);
        reals(2*k) = rk*cos(-thetak);
        imags(2*k) = rk*sin(-thetak);
    end
    
    poles = reals + sqrt(-1)*imags + center;

end
