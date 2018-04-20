% Problem9.m
%
% Created by J. McCready on 2018-04-15
% ECE 560 Winter 18
% University of Michigan - Dearborn
%
% Last updated: 2018-04-08 by J. McCready
%   - Controls the pendulum from down to up position

close all;
clear all;

%% Declare Variables
global m M l I g x_d K A B lin_state u;
%syms k1 k2 k3 k4 gam
% use A, B, K, and lin_state as globals to feed into dx functions

    m = 0.1;
    M = 2;
    l = 0.5;
    I = 0.025;
    g = 9.8;
    L = (I + m*l^2)/(m*l);
    Mt = M + m;

    x_d = [0; 0; 0; 0];
%% Syms
% K = [k1 k2 k3 k4];
% Gamma = [gam 0 0 0;
%          0 gam 0 0;
%          0 0 gam 0;
%          0 0 0 gam];

    A_down =   [0, 1,                    0, 0; ...
                0, 0, (g*l*m)/(l*m - L*Mt), 0; ...
                0, 0,                    0, 1; ...
                0, 0,  (Mt*g)/(l*m - L*Mt), 0];
    B_down =   [0; -L/(l*m - L*Mt); 0; -1/(l*m - L*Mt)]; 

    A_up =     [0, 1,                    0, 0; ...
                0, 0, (g*l*m)/(l*m - L*Mt), 0; ...
                0, 0,                    0, 1; ...
                0, 0, -(Mt*g)/(l*m - L*Mt), 0];

    B_up =     [0; -L/(l*m - L*Mt); 0; 1/(l*m - L*Mt)];


%% Hanging Equilibrium  
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 

    init = [0; 0; pi; 0;];
    x_d = [0; 0; pi/2; -3/2*pi];
    tspan = [0, 20];
    Q = [0.000001 0 0  0; ...
         0 0.000001 0  0; ...
         0 0  100 0; ...
         0 0 0 500];
     R = 1;
    [K, S, e] = lqr(A_down, B_down, Q, R); 

    options = odeset('AbsTol', 1e-6, 'RelTol', 1e-6, 'Events', @upTransitionEvent);
    [t_down, x_down] = ode45(@CartPoleSystem, tspan, init, options);
    formatspec1 = 'K = %G, %G, %G, %G';
    formatspec2 = '\\lambda = %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi';
    formatspec3 = '$$Q = \\begin{tabular}{|l l l l|} %G&0&0&0\\\\ 0&%G&0&0\\\\ 0&0&%G&0\\\\ 0&0&0&%G\\\\ \\end{tabular}$$, R = %G';
    strKdown = sprintf(formatspec1, K(1), K(2), K(3), K(4));
    strEigdown = sprintf(formatspec2, real(e(1)), imag(e(1)), real(e(2)), imag(e(2)), real(e(3)), imag(e(3)), real(e(4)), imag(e(4))); 
    strLQRdown = sprintf(formatspec3, Q(1,1), Q(2,2), Q(3,3), Q(4,4), R); 
    
% Compute u for down position
    q = (x_down-repmat(x_d', [size(x_down,1), 1, 1]));
    for i = 1:size(x_down, 1)
    u_down(i) = -K*q(i,:)';
    end 

% Event should have stopped the simulation start with new controller
    tspan2 = [t_down(end) t_down(end) + 10]; 
    init2 = x_down(end,:);
    Q = [1e-15 0 0  0; ...
         0  15 0  0; ...
         0 0  50 0; ...
         0 0 0 50];
     R = 5;
    x_d = [0; 0; 0; 0];
    [K S e] = lqr(A_up, B_up, Q, R);    
    
    strKup = sprintf(formatspec1, K(1), K(2), K(3), K(4));
    strEigup = sprintf(formatspec2, real(e(1)), imag(e(1)), real(e(2)), imag(e(2)), real(e(3)), imag(e(3)), real(e(4)), imag(e(4))); 
    strLQRup = sprintf(formatspec3, Q(1,1), Q(2,2), Q(3,3), Q(4,4), R); 

    [t_up, x_up] = ode45(@CartPoleSystem, tspan2, init2);
    
    q = (x_up-repmat(x_d', [size(x_up,1), 1, 1]));
    for i = 1:size(x_up, 1)
    u_up(i) = -K*q(i,:)';
    end 
    
    U  = [u_down'; u_up'];
    x  = [x_down; x_up]; 
    t  = [t_down; t_up]; 
    
    animation.Draw(x(:,1), x(:,3), t);
    animation.Close();

 %% Plots 
    f2 = figure('Name',num2str(1,1),'units','normalized','outerposition',[0 0 1 .95]);
    grid on; 
    % Plot Time Data 
    top = subplot(3,1,1);
    title(top, 'State variables, Controlling down to up using LQR method' ,'FontWeight', 'bold', 'FontSize', 14);    

    yyaxis left
    plot(t, x(:,1), 'LineWidth',2);
%    xlabel('time (s)')
    ylabel('s (m)')
    yyaxis right 
    plot(t, x(:,2), 'Linewidth',2);
    ylabel('ds/dt (m/s)')
    top.FontSize = 16;
    top.FontWeight = 'bold';
    top.LineWidth = 1.5;

    middle = subplot(3,1,2);
    yyaxis left
    plot(t, x(:,3),'LineWidth',2);
%    xlabel('time (s)')
    ylabel('\theta (rad)');
    yyaxis right
    plot(t, x(:,4), 'Linewidth',2);
    ylabel('d\theta/dt (rad/s)')
    middle.FontSize = 16;
    middle.FontWeight = 'bold';
    middle.LineWidth = 1.5;
    
    bottom = subplot(3,1,3); 
    plot(t, U, 'r-','LineWidth',2); 
    xlabel('time (s)');
    ylabel('u (m/s^2)')
    bottom.FontSize = 16;
    bottom.FontWeight = 'bold';
    bottom.LineWidth = 1.5;

    
%% Functions     
function [value,isterminal,direction] = upTransitionEvent(t,x)
    if (x(4)<0)
        value = x(3)-pi/2;
    else
        value = x(3) - 3/2*pi; 
    end
    isterminal = 1; 
    % Stop integration and transfer new state two up controller
    if (x(3) > 3/2*pi) % Only allow if angular rate is correct
        direction = 1; 
    else
        direction = -1;
    end
end 


function dx = CartPoleSystem(t, x)

    global m M l I g x_d K u;
    
    L = (I + m*l^2)/(m*l);
    Mt = M + m;

    s = x(1);
    ds = x(2);
    theta = x(3);
    dtheta = x(4);
  
    u = -K*(x-x_d); 

    %% State Space Nonlinear Equations developed from equations of motion
    dds = (- m*l*g*sin(theta)*cos(theta) + m*l*L*sin(theta)*dtheta.^2)/...
        (Mt*L - m*l*(cos(theta).^2)) +(L*u)/(L*Mt - l*m*cos(theta)^2);
    ddtheta = (-m*l*sin(theta)*cos(theta)*dtheta.^2 + Mt*g*sin(theta))/ ...
        (Mt*L - m*l*(cos(theta).^2)) - (u*cos(theta))/(L*Mt - l*m*cos(theta)^2);
    
    dx = [ds; dds; dtheta; ddtheta];
    
end
