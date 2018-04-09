close all;
clear all;
%% Declare Variables
global m M l I g x_d K A B lin_state;

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
    
%% Plot Preparation            
% simulationRefreshSpeed = 10;
% animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% animation.EnablePlotRecoder();            


%% Run Non Linear Simulation vs. Linear Simulation, Hanging

delta = pi/20;
init = [0; 0; pi+delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 15];
[t, x] = ode45(@CartPoleSystem, tspan, init);

% animation.Draw(x(:,1), x(:,3), t);
% animation.Close();

% Run Linearized Simulation - down - working
% With this system it was assumed that the intial state was x = [s0; 0; pi
% 0;], consequently this assumed intial state must be subtracted from x.
% Basically the substitution is x_down_lin = x - x_down^*

A = A_down; B = B_down;
lin_state = [0; 0; pi; 0]; % change s0 position if needed 
x_d = [0; 0; 0; 0];
tspan = [0, 5];


delta = [0.01:0.1:0.51 3/4]*pi; 
theta_traj = figure(1); 
hold on;
for i = 1:size(delta,2)
    init = [0; 0; pi+delta(i); 0];
    [t_lin, x_lin] = ode45(@CartPoleLinearSystem, tspan, init);
    [t, x] = ode45(@CartPoleSystem, tspan, init);
    plot(init(3), init(4), '*');
    plot(x(:,3), x(:,4));
    plot(x_lin(:,3), x_lin(:,4), '-');
    
%     animation.Draw(x(:,1), x(:,3), t);
%     animation.Close();
end 
axis([0,2*pi, -2.5*pi 2.5*pi]);
xticks([0 0.25*pi 0.5*pi .75*pi pi 1.25*pi 1.50*pi 1.75*pi 2*pi])
xticklabels({'0', '0.25\pi','0.5\pi', '0.75\pi','\pi','1.25\pi','1.5\pi','1.75\pi', '2\pi'})
hold off; 


%% Run Linearized Simulation - up
% With this system it was assumed that the intial state was x = [s0; 0; pi
% 0;], consequently this assumed intial state must be subtracted from x.
% Basically the substitution is x_down_lin = x - x_down^*

% A = A_up; B = B_up;
% lin_state = [0; 0; 0; 0]; % change s0 position if needed 
% delta = 0.001;
% init = [0; 0; delta; 0];
% x_d = [0; 0; 0; 0];
% tspan = [0, 15];
% [t, x] = ode45(@CartPoleLinearSystem, tspan, init);
% 
% animation.Draw(x(:,1), x(:,3), t);
% animation.Close();

%% ADD YOUR PLOTS



        