function dx = CartPoleSystem(t, x)

    global m M l I g;
    
    L = (I + m*l^2)/(m*l);
    Mt = M + m;

    s = x(1);
    ds = x(2);
    theta = x(3);
    dtheta = x(4);
    

    %% State Space Nonlinear Equations developed from equations of motion
    dds = (- m*l*g*sin(theta)*cos(theta) + m*l*L*sin(theta)*dtheta.^2)/...
        (Mt*L - m*l*(cos(theta).^2)); % Where is the u contribution???
    
    ddtheta = (-m*l*sin(theta)*cos(theta)*dtheta.^2 + Mt*g*sin(theta))/ ...
        (Mt*L - m*l*(cos(theta).^2)); % Where is the u contribution???
    
    dx = [ds; dds; dtheta; ddtheta];
    
end
