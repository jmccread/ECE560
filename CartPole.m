close all;

%% Declare Variables
global m M l I g x_d K;

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
B_down =   [1,               0, 0,               0; ...
            0, -L/(l*m - L*Mt), 0,               0; ...
            0,               0, 1,               0; ...
            0,               0, 0, -1/(l*m - L*Mt)]; 
        
A_up =     [0, 1,                    0, 0; ...
            0, 0, (g*l*m)/(l*m - L*Mt), 0; ...
            0, 0,                    0, 1; ...
            0, 0, -(Mt*g)/(l*m - L*Mt), 0];
        
B_up =     [1,               0, 0,              0; ...
            0, -L/(l*m - L*Mt), 0,              0; ...
            0,               0, 1,              0; ...
            0,               0, 0, 1/(l*m - L*Mt)];
    
%% Plot Preparation            
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder();            


%% Run Non Linear Simulation

% delta = pi/20;
% init = [0; 0; pi+delta; 0];
% x_d = [0; 0; 0; 0];
% tspan = [0, 15];
% [t, x] = ode45(@CartPoleSystem, tspan, init);
% 
% animation.Draw(x(:,1), x(:,3), t);
% animation.Close();

%% Run Linearized Simulation

A = A_down; B = B_down; 
delta = pi -0.05;
init = [0; 0; pi+delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 15];
[t, x] = ode45(@CartPoleSystem, tspan, init);

animation.Draw(x(:,1), x(:,3), t);
animation.Close();

%% ADD YOUR PLOTS






        