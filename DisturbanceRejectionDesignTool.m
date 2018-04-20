% DisturbanceRejectionDesignTool.m
%
% Created by J. McCready on 2018-04-15
% ECE 560 Winter 18
% University of Michigan - Dearborn
%
% Last updated: 2018-04-08 by J. McCready
% Add a disturbance and design an integrator to compensate.

close all;
clear all;

%% Declare Variables
global m M l I g x_d K A B lin_state u alpha Tau KI;
%syms k1 k2 k3 k4 gam
% use A, B, K, and lin_state as globals to feed into dx functions

m = 0.1;
M = 2;
l = 0.5;
I = 0.025;
g = 9.8;
L = (I + m*l^2)/(m*l);
Mt = M + m;
alpha = 1/10;
Tau = 1;


%% State space system manipulation
C = [0 0 1 0]; 
Bd =   [0; 0; 0; alpha*-1/(l*m - L*Mt)]; 
A_down =   [0, 1,                    0, 0; ...
            0, 0, (g*l*m)/(l*m - L*Mt), 0; ...
            0, 0,                    0, 1; ...
            0, 0,  (Mt*g)/(l*m - L*Mt), 0];

B_down = [0; -L/(l*m - L*Mt); 0; -1/(l*m - L*Mt)]; 
Bt_down = [B_down, Bd]; 

Aa_down = [[A_down,[0 0 0 0]'];[C, 0]];
Ba_down = [Bt_down;[0 0]];
Ca_down = [C,0];


A_up =     [0, 1,                    0, 0; ...
            0, 0, (g*l*m)/(l*m - L*Mt), 0; ...
            0, 0,                    0, 1; ...
            0, 0, -(Mt*g)/(l*m - L*Mt), 0];
B_up =     [0; -L/(l*m - L*Mt); 0; 1/(l*m - L*Mt)];
Bt_up = [B_up, Bd]; 

Aa_up = [[A_up,[0 0 0 0]'];[C, 0]];
Ba_up = [Bt_up;[0 0]];
Ca_up = [C,0];

%% Hanging Equilibrium  
disp('Movie Run Non Linear Simulation - down');

simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 
% Compute controller settings from problem 5 
delta = pi/10;
init = [0; 0; pi+delta; 0; 0];
x_d = [0; 0; pi; 0; 0];
tspan = [0, 20];
Q = [10 0 0  0; ...
     0 10 0  0; ...
     0 0  100 0; ...
     0 0 0 100];
 R = 0.5;
[K, S, e] = lqr(A_down, B_down, Q, R); 
%Poles = [e; -30];
K = [K 10];

[t, x] = ode45(@CartPoleSystem, tspan, init);
formatspec1 = 'K = %G, %G, %G, %G, %G';
formatspec2 = '\\lambda = %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi';
str1 = sprintf(formatspec1, K(1), K(2), K(3), K(4), K(5));
str2 = sprintf(formatspec2, real(e(1)), imag(e(1)), real(e(2)), imag(e(2)), real(e(3)), imag(e(3)), real(e(4)), imag(e(4))); 
%controlled_pendulum_plotter(t,x, Q, R, 3, {'Hanging Equilibrium, input disturbance' str1 str2 ' '})
% 
animation.Draw(x(:,1), x(:,3), t);
animation.Close();
controlled_pendulum_plotter(t,x, Q, R, 3, {'Hanging Equilibrium, input disturbance' str1 str2 ' '})

%% Vertical Equilibrium  
disp('Movie Run Non Linear Simulation - up');
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 
% 
% 
delta = pi/10;
init = [0; 0; delta; 0; 0];
theta_fin = 0;
x_d = [0; 0; theta_fin; 0; 0];
tspan = [0, 20];
Q = [1 0 0 0; ...
     0 10 0 0; ...
     0 0 50 0; ....
     0 0 0 50];
 R = .1;
[K, S, e] = lqr(A_up, B_up, Q, R); 
%Poles = [e; -30];
K = [K 10];
[t, x] = ode45(@CartPoleSystem, tspan, init);
formatspec1 = 'K = %G, %G, %G, %G, %G';
formatspec2 = '\\lambda = %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi, %.3g %+.3gi';
str1 = sprintf(formatspec1, K(1), K(2), K(3), K(4), K(5));
str2 = sprintf(formatspec2, real(e(1)), imag(e(1)), real(e(2)), imag(e(2)), real(e(3)), imag(e(3)), real(e(4)), imag(e(4))); 
%controlled_pendulum_plotter(t,x, Q, R, 4, {'Vertical Equilibrium, input disturbance' str1 str2 ' '})

animation.Draw(x(:,1), x(:,3), t);
animation.Close();
controlled_pendulum_plotter(t,x, Q, R, 4, {'Vertical Equilibrium, input disturbance' str1 str2 ' '})

%% State Trajectories Vertical, Initial theta displacement, poles close to origin
function controlled_pendulum_plotter(t,x, Q, R, fignum, figname)
% This is a function that makes this file shorter 
    global K x_d alpha Tau
    q = (x-repmat(x_d', [size(x,1), 1, 1]));
    for i = 1:size(x, 1)
    u(i) = -K*q(i,:)';
    w(i) = alpha*(t(i)>=Tau);
    end 

    %% Plot time series simulation results 
    f2 = figure('Name',num2str(fignum(1),1),'units','normalized','outerposition',[0 0 1 .95]);

    % Plot Time Data 
    left = subplot(2,2,1);
    left.FontSize = 16;
    left.FontWeight = 'bold';
    left.LineWidth = 1.5;
    yyaxis left
    plot(t, x(:,1), 'LineWidth',2);
    xlabel('time (s)')
    ylabel('s (m)')
    yyaxis right 
    hold on;
    plot(t, x(:,2), 'Linewidth',2);
    line = left.YLim(1)+0.1: 0.1: left.YLim(2)-0.1;
    plot(Tau*ones(size(line)), line, 'k--','Linewidth',1);
    plot(Tau*[1, 1], [left.YLim(1) left.YLim(2)], 'k*','Linewidth',1);
    hold off; 
    ylabel('ds/dt (m/s)')
    
    right = subplot(2,2,2);
    right.FontSize = 16;
    right.FontWeight = 'bold';
    right.LineWidth = 1.5;
    yyaxis left
    plot(t, x(:,3),'LineWidth',2);
    xlabel('time (s)')
    ylabel('\theta (rad)');
    yyaxis right
    hold on;
    plot(t, x(:,4), 'Linewidth',2);
    line = right.YLim(1)+0.1: 0.1: right.YLim(2)-0.1;
    plot(Tau*ones(size(line)), line, 'k--','Linewidth',1);
    plot(Tau*[1, 1], [right.YLim(1) right.YLim(2)], 'k*','Linewidth',1);
    hold off; 
    ylabel('d\theta/dt (rad/s)')
        right.FontSize = 16;
    right.FontWeight = 'bold';
    right.LineWidth = 1.5;
      
    input = subplot(2,2,3); 
    yyaxis left
    plot(t, u,'LineWidth',2); 
    xlabel('time (s)');
    ylabel('u (m/s^2)')
    yyaxis right
    plot(t, w,'LineWidth',2); 
    ylabel('w (N-m)')
    input.FontSize = 16;
    input.FontWeight = 'bold';
    input.LineWidth = 1.5;
    
    % Calculate some stats 
    tdat = stepinfo(x(:,3), t, x_d(3)); 
    sdat = stepinfo(x(:,1), t, 0);
    %udat = stepinfo(u(:,1), t);
    tabledata = {' ' 'Settling T (s)' 'Rise T (s)' 'Stlng Min (rad)'; ...
                 'theta:' tdat.SettlingTime tdat.RiseTime tdat.SettlingMin; ...
                 ' ' 'Settling T (s)' 'Peak T (s)' 'Peak S (m)';
                 's:' sdat.SettlingTime sdat.PeakTime sdat.Peak}; 
    % Create the column and row names in cell arrays 
    cnames = {'1','2','3','4'};
    rnames = {'1','2','3','4'};
    % Create the uitable
    table = uitable(f2,'Data',tabledata,...
                'ColumnWidth',{120}, ...
                'FontSize', 12, ...
                'FontWeight', 'bold'); 
    info = subplot(2,2,4);
    pos = get(subplot(2,2,4),'position');
    delete(subplot(2,2,4))
    set(table,'units','normalized')
    set(table,'position',pos)
    
    formatspec = '$$Q = \\begin{tabular}{|l l l l|} %G&0&0&0\\\\ 0&%G&0&0\\\\ 0&0&%G&0\\\\ 0&0&0&%G\\\\ \\end{tabular}$$, R = %G';
    str = sprintf(formatspec, Q(1,1), Q(2,2), Q(3,3), Q(4,4), R); 
    title(right, {str ' '}, 'Interpreter', 'latex', 'FontWeight', 'bold', 'FontSize', 14);
    title(left, figname,'FontWeight', 'bold', 'FontSize', 14);    
end

%% Functions 
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
