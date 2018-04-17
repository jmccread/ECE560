function dx = CartPoleSystem(t, x)

    global m M l I g x_d K u Tau alpha;
    
    L = (I + m*l^2)/(m*l);
    Mt = M + m;

    s = x(1);
    ds = x(2);
    theta = x(3);
    dtheta = x(4);
    v = x(5);
  
    u = -K*x; 
    w = alpha*(t>=Tau);
    %% State Space Nonlinear Equations developed from equations of motion
    dds = (- m*l*g*sin(theta)*cos(theta) + m*l*L*sin(theta)*dtheta.^2)/...
        (Mt*L - m*l*(cos(theta).^2)) +(L*u)/(L*Mt - l*m*cos(theta)^2);
    ddtheta = (-m*l*sin(theta)*cos(theta)*dtheta.^2 + Mt*g*sin(theta))/ ...
        (Mt*L - m*l*(cos(theta).^2)) - (u*cos(theta) + w)/(L*Mt - l*m*cos(theta)^2);
    dv = theta - x_d(3);
    dx = [ds; dds; dtheta; ddtheta; dv];
    
end
