close all;
clear all;

% clean up simulation warnings en masse
warning('off','all')
warning
%% Declare Variables
global m M l I g x_d K A B lin_state; 
% use A, B, K, and lin_state as globals to feed into dx functions

m = 0.1;
M = 2;
l = 0.5;
I = 0.025;
g = 9.8;
L = (I + m*l^2)/(m*l);
Mt = M + m;

x_d = [0; 0; 0; 0];
K = [0 0 0 0];

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
    


% %% Movie Run Non Linear Simulation - up
% % Plot Preparation   
disp('Movie Run Non Linear Simulation - up');
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 

delta = pi/1000;
init = [0; 0; delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 5];
[t, x] = ode45(@CartPoleSystem, tspan, init);

animation.Draw(x(:,1), x(:,3), t);
animation.Close();


%% Movie: Run Linearized Simulation - up 
% Plot Preparation          
disp('Movie: Run Linearized Simulation - up');
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 

A = A_up; B = B_up;
lin_state = [0; 0; 0; 0]; % change s0 position if needed 
init = [0; 0; delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 5];
[t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);

animation.Draw(x_lin(:,1), x_lin(:,3), t_lin);
animation.Close();

%% Plot Linear vs Non Linear as fn of time
index_lin = find(t_lin < 3);
index = find(t <3); 
LinNonLin = figure('Name','7','units','normalized','outerposition',[0 0 1 .95]);
subplot(1,2,1);
hold on;
plot(t(index), x(index,1), 'k-','LineWidth',2);
plot(t_lin(index_lin), x_lin(index_lin,1), 'r-.','LineWidth',2);
hold off
left = subplot(1,2,1);
left.XLabel.String='time (s)';
left.YLabel.String='Cart position: x_1 = s (m)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;

subplot(1,2,2);
hold on;
plot(t(index), x(index,3), 'k-','LineWidth',2);
plot(t_lin(index_lin), x_lin(index_lin,3), 'r-.','LineWidth',2);
hold off
right = subplot(1,2,2);
right.XLabel.String='time (s)';
right.YLabel.String='Angular Dispacement from vertical x_3 = \theta (rad)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
right.YLim =[0 6*pi];
right.YTick =[0 0.25*pi 0.5*pi 0.75*pi pi 1.25*pi 1.5*pi 1.75*pi  2*pi];
right.YTickLabel = {'0' '0.25\pi' '0.5\pi' '0.75\pi'  '\pi' '1.25\pi' '1.5\pi' '1.75\pi' '2\pi'};

legend('Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Vertical Equilibrium, Linearized vs Nonlinear Model, x_0 = [ 0 0 \pi/1000 0]'`);
hold off; 


%% State Trajectories Vertical, Initial theta displacement
disp('State Trajectories Vertical, Initial theta displacement');
tspan = [0, 0.5];
delta = (0.01:0.04:0.17)*pi; 
theta_traj = figure('Name','4','units','normalized','outerposition',[0 0 1 .95]);
hold on;
for i = 1:size(delta,2)
    init = [0; 0; delta(i); 0];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
left.XLim = [0,0.5*pi];
left.YLim =[0 1.5*pi];
left.XTick =[0 0.125*pi 0.25*pi .375*pi 0.50*pi];
left.XTickLabel = {'0' '0.125\pi' '0.25\pi' '0.375\pi' '0.5\pi'};
left.YTick =[0 0.5*pi pi 1.5*pi];
left.YTickLabel = {'0' '0.5\pi' '\pi' '1.5\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;

% right.XLim = [-.08 .0];
% right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Vertical Equilibrium, State Trajectories with initial state [ 0 0 \delta 0]');
hold off; 

%% State Trajectories Vertical, Initial angular rate
disp('State Trajectories Vertical, Initial angular rate');
tspan = [0, 0.5];
delta = (0.01:0.1:0.51)*pi; 
theta_dot_traj =  figure('Name','5','units','normalized','outerposition',[0 0 1 .95]);
hold on;
for i = 1:size(delta,2)
    init = [0; 0; 0; delta(i)];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
left.XLim = [0,0.4*pi];
left.YLim =[0 1.25*pi];
left.XTick =[0*pi .1*pi 0.2*pi 0.3*pi .4*pi];
left.XTickLabel = {'0', '0.1\pi','0.2\pi','0.3\pi','0.4\pi'};
left.YTick =[0 0.25*pi 0.5*pi 0.75*pi pi 1.25*pi];
left.YTickLabel = {'0','0.25\pi', '0.5\pi', '0.75\pi', '\pi', '1.25\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;
% right.XLim = [-.08 .0];
% right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Vertical Equilibrium, State Trajectories with initial state [ 0 0 0 d\theta/dt_0]');
hold off; 

%% State Trajectories, Vertical, Initial cart speed 
disp('State Trajectories, Vertical, Initial cart speed');
tspan = [0, 2.5];
delta = (-0.51:0.2:.51); 
s_dot_traj =  figure('Name','6','units','normalized','outerposition',[0 0 1 .95]);
hold on;
for i = 1:size(delta,2)
    init = [0; delta(i); 0; 0];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
%  left.XLim = [0.75*pi, 1.25*pi];
%  left.YLim =[-0.25*pi 0.25*pi];
%  left.XTick =[0.75*pi pi 1.25*pi];
%  left.XTickLabel = {'0.75\pi', '\pi','1.25\pi'};
%  left.YTick =[-0.25*pi  0 0.25*pi];
%  left.YTickLabel = {'-0.25\pi','0', '0.25\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;
% right.XLim = [-.08 .0];
% right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Vertical Equilibrium, State Trajectories with initial state [ 0 ds/dt_0 \pi 0]');
hold off; 

%% Movie Run Non Linear Simulation - down
% Plot Preparation       
disp('Movie Run Non Linear Simulation - down');
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 

delta = pi/5;
init = [0; 0; pi+delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 7.5];
[t, x] = ode45(@CartPoleSystem, tspan, init);

animation.Draw(x(:,1), x(:,3), t);
animation.Close();

%% Movie: Run Linearized Simulation - down 
% Plot Preparation            
disp('Movie: Run Linearized Simulation - down');
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder(); 

A = A_down; B = B_down;
lin_state = [0; 0; pi; 0]; % change s0 position if needed 
init = [0; 0; pi+delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 7.5];
[t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);

animation.Draw(x_lin(:,1), x_lin(:,3), t_lin);
animation.Close();

%% Plot Linear vs Non Linear as fn of time
index_lin = find(t_lin < 5);
index = find(t <5); 
LinNonLin2 = figure('Name','9','units','normalized','outerposition',[0 0 1 .95]);
subplot(1,2,1);
hold on;
plot(t(index), x(index,1), 'k-','LineWidth',2);
plot(t_lin(index_lin), x_lin(index_lin,1), 'r-.','LineWidth',2);
hold off
left = subplot(1,2,1);
left.XLabel.String='time (s)';
left.YLabel.String='Cart position: x_1 = s (m)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;

subplot(1,2,2);
hold on;
plot(t(index), x(index,3), 'k-','LineWidth',2);
plot(t_lin(index_lin), x_lin(index_lin,3), 'r-.','LineWidth',2);
hold off
right = subplot(1,2,2);
right.XLabel.String='time (s)';
right.YLabel.String='Angular Dispacement from vertical x_3 = \theta (rad)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
right.YLim =[.75*pi 1.25*pi];
right.YTick =[0.75*pi 0.875*pi pi 1.125*pi 1.25*pi];
right.YTickLabel = {'0.75\pi' '0.875\pi' '\pi' '1.125\pi' '1.25\pi'};

legend('Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Hanging Equilibrium, Linearized vs Nonlinear Model, x_0 = [ 0 0 1.2\pi 0]');
hold off; 

%% Hanging, Initial theta displacement
disp('Hanging, Initial theta displacement');
tspan = [0, 5];
delta = (0.01:0.1:0.51)*pi; 
theta_traj = figure(1); 
hold on;
for i = 1:size(delta,2)
    init = [0; 0; pi+delta(i); 0];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
left.XLim = [0.4*pi,1.6*pi];
left.YLim =[-2*pi 2*pi];
left.XTick =[0.5*pi .75*pi pi 1.25*pi 1.50*pi];
left.XTickLabel = {'0.5\pi', '0.75\pi','\pi','1.25\pi','1.5\pi'};
left.YTick =[-2*pi -pi 0 pi 2*pi];
left.YTickLabel = {'-2\pi', '-\pi',0 '\pi','2\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;

right.XLim = [-.08 .0];
right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Hanging Equilibrium, State Trajectories with initial state [ 0 0 (\pi+\delta) 0]');
hold off; 

%% Hanging, Initial angular rate
disp('Hanging, Initial angular rate');
tspan = [0, 2.5];
delta = (0.01:0.25:1.26)*pi; 
theta_dot_traj = figure(2); 
hold on;
for i = 1:size(delta,2)
    init = [0; 0; pi; delta(i)];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
left.XLim = [0.5*pi,1.5*pi];
left.YLim =[-1.5*pi 1.5*pi];
left.XTick =[0.5*pi .75*pi pi 1.25*pi 1.50*pi];
left.XTickLabel = {'0.5\pi', '0.75\pi','\pi','1.25\pi','1.5\pi'};
left.YTick =[-1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi];
left.YTickLabel = {'-1.5\pi', '-\pi', '-0.5\pi',0, '0.5\pi', '\pi','1.5\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;

% right.XLim = [-.08 .0];
% right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Hanging Equilibrium, State Trajectories with initial state [ 0 0 \pi d\theta/dt_0]');
hold off; 

%% Hanging, Initial cart speed 
disp('Hanging, Initial cart speed');
tspan = [0, 2.5];
delta = (-0.51:0.1:.51); 
s_dot_traj = figure(3); 
hold on;
for i = 1:size(delta,2)
    init = [0; delta(i); pi; 0];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    left = subplot(1,2,1);
    hold on;
    plot(init(3), init(4), 'b*');
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2);
    plot(x_lin(:,3), x_lin(:,4), 'r-.','LineWidth',2);
    right = subplot(1,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2);
    plot(x_lin(:,1), x_lin(:,2), 'r-.','LineWidth',2);
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
 left.XLim = [0.75*pi, 1.25*pi];
 left.YLim =[-0.25*pi 0.25*pi];
 left.XTick =[0.75*pi pi 1.25*pi];
 left.XTickLabel = {'0.75\pi', '\pi','1.25\pi'};
 left.YTick =[-0.25*pi  0 0.25*pi];
 left.YTickLabel = {'-0.25\pi','0', '0.25\pi'};
left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
left.FontSize = 16;
left.FontWeight = 'bold';
left.LineWidth = 1.5;
% right.XLim = [-.08 .0];
% right.YLim = [-.15 .15];
right.XLabel.String='Cart position: x_1 = s (m)';
right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
right.FontSize = 16;
right.FontWeight = 'bold';
right.LineWidth = 1.5;
legend('Initial state', 'Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
suptitle('Hanging Equilibrium, State Trajectories with initial state [ 0 ds/dt_0 \pi 0]');
hold off; 
% 



        