close all;

global m M l I g;

m = 0.1;
M = 2;
l = 0.5;
I = 0.025;
g = 9.8;
    
%% Plot Preparation            
simulationRefreshSpeed = 10;
animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
animation.EnablePlotRecoder();            


%% Run Simulation

init = [0; 0; 0.05; 0];
tspan = [0, 15];
[t, x] = ode45(@CartPoleSystem, tspan, init);

animation.Draw(x(:,1), x(:,3), t);
animation.Close();

%% ADD YOUR PLOTS






        