% StateCompensatorDesignTool.m
%
% Created by J. McCready on 2018-04-18
% ECE 560 Winter 18
% University of Michigan - Dearborn
%
% Last updated: 2018-04-08 by J. McCready
%   - Work to easily design state feedback compensators for a given set of
%   poles. 

close all;
clear all;

%% Declare Variables
global m M l I g x_d K A B lin_state u;
syms k1 k2 k3 k4 gam
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
K = [k1 k2 k3 k4];
Gamma = [gam 0 0 0;
         0 gam 0 0;
         0 0 gam 0;
         0 0 0 gam];

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

%% Pole Locations
up_poles = eig(A_up);
down_poles = eig(A_down);
char_eqn_down = det(Gamma - (A_down - B_down*K));
char_eqn_down = simplify(char_eqn_down);
char_eqn_up = det(Gamma - (A_up - B_up*K));
char_eqn_up = simplify(char_eqn_up);

%% Pole Placement with place command
% % place poles at -1, -2 -3, -4 as a silly solution near to the origin
% P_down_close = [-0.5 -1 -1.5 -2]; 
% K_down_close =  place(A_down, B_down, P_down_close);
% 
% % place poles at -20, -21 -22, -22 as a silly solution near far from the origin
% P_down_far = [-8, -8.5 -9, -9.5]; 
% K_down_far =  place(A_down, B_down, P_down_far);
% 
% % Place poles at -1.5 + 1.5j, -1.5 - 1.5j, -3.0 + 1.5j, -3 + 1.5j
% P_down_complex = [-1.5 + 1.5j, -1.5 - 1.5j, -3.0 + 1.5j, -3 - 1.5j]; 
% K_down_complex =  place(A_down, B_down, P_down_complex);
% 
% %% Real axis pole placement close to origin
% % disp('Movie Run Non Linear Simulation - up');
% % simulationRefreshSpeed = 10;
% % animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% % animation.EnablePlotRecoder(); 
% 
% delta = pi/10;
% init = [0; 0; pi+delta; 0];
% x_d = [0; 0; pi; 0];
% tspan = [0, 15];
% K = K_down_close;
% [t, x] = ode45(@CartPoleSystem, tspan, init);
% title = {'Hanging Equilibrium, State Trajectories with initial state [ 0 0 1.1\pi 0]';
%          'Hanging Equilibrium, State variables with time domain characteristics'};
% controlled_pendulum_plotter(t,x, init, P_down_close,[1,2], title);
% 
% 
% % animation.Draw(x(:,1), x(:,3), t);
% % animation.Close();
% % 
% %% Real axis pole placement far from origin 
% % disp('Movie Run Non Linear Simulation - up');
% % simulationRefreshSpeed = 10;
% % animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% % animation.EnablePlotRecoder(); 
% 
% delta = pi/10;
% init = [0; 0; pi+delta; 0];
% x_d = [0; 0; pi; 0];
% tspan = [0, 15];
% K = K_down_far;
% [t, x] = ode45(@CartPoleSystem, tspan, init);
% title = {'Hanging Equilibrium, State Trajectories with initial state [ 0 0 1.1\pi 0]';
%          'Hanging Equilibrium, State variables with time domain characteristics'};
% controlled_pendulum_plotter(t,x, init, P_down_far,[3,4], title);
% 
% % animation.Draw(x(:,1), x(:,3), t);
% % animation.Close();
% % 
% %% Real axis pole placement complex plane
% % disp('Movie Run Non Linear Simulation - up');
% % simulationRefreshSpeed = 10;
% % animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% % animation.EnablePlotRecoder(); 
% 
% delta = pi/10;
% init = [0; 0; pi+delta; 0];
% x_d = [0; 0; pi; 0];
% tspan = [0, 15];
% K = K_down_complex;
% [t, x] = ode45(@CartPoleSystem, tspan, init);
% title = {'Hanging Equilibrium, State Trajectories with initial state [ 0 0 1.1\pi 0]';
%          'Hanging Equilibrium, State variables with time domain characteristics'};
% controlled_pendulum_plotter(t,x, init, P_down_complex,[1,2], title);
% 
% % 
% % animation.Draw(x(:,1), x(:,3), t);
% % animation.Close();


%% Pole Placement with place command
% place poles at -1, -2 -3, -4 as a silly solution near to the origin
P_up_close = [-0.1 -0.2 -1.2 -0.6]; 
K_up_close =  place(A_up, B_up, P_up_close);

% place poles at -20, -21 -22, -22 as a silly solution near far from the origin
P_up_far = [-1, -1.5 -3.6, -1.8]; 
K_up_far =  place(A_up, B_up, P_up_far);

% Place poles at -1.5 + 1.5j, -1.5 - 1.5j, -3.0 + 1.5j, -3 + 1.5j
%P_up_complex = [-3 + .5j, -3 - .5j, -1.5 + 1.5j, -1.5 - 1.5j]; 
P_up_complex = [-1 + 1/6j, -1 - 1/6j, -1/2 + 1/3j, -1/2 - 1/3j];
K_up_complex =  place(A_up, B_up, P_up_complex);

%% Real axis pole placement close to origin
% disp('Movie Run Non Linear Simulation - up');
%simulationRefreshSpeed = 10;
%animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
%animation.EnablePlotRecoder(); 

delta = pi/10;
init = [0; 0; delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 150];
K = K_up_close;
[t, x] = ode45(@CartPoleSystem, tspan, init);
title = {'Vertical Equilibrium, State Trajectories with initial state [ 0 0 0.1\pi 0]';
         'Vertical Equilibrium, State variables with time domain characteristics'};
controlled_pendulum_plotter(t,x, init, P_up_close,[1,2], title);


%animation.Draw(x(:,1), x(:,3), t);
%animation.Close();

%% Real axis pole placement "far" to origin
% disp('Movie Run Non Linear Simulation - up');
% simulationRefreshSpeed = 10;
% animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% animation.EnablePlotRecoder(); 

delta = pi/10;
init = [0; 0; delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 20];
K = K_up_far;
[t, x] = ode45(@CartPoleSystem, tspan, init);
title = {'Vertical Equilibrium, State Trajectories with initial state [ 0 0 0.1\pi 0]';
         'Vertical Equilibrium, State variables with time domain characteristics'};
controlled_pendulum_plotter(t,x, init, P_up_far,[3,4], title);

% animation.Draw(x(:,1), x(:,3), t);
% animation.Close();

%% Real axis pole placement in complex plane
% disp('Movie Run Non Linear Simulation - up');
% simulationRefreshSpeed = 10;
% animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
% animation.EnablePlotRecoder(); 

delta = pi/10;
init = [0; 0; delta; 0];
x_d = [0; 0; 0; 0];
tspan = [0, 25];
K = K_up_complex;
[t, x] = ode45(@CartPoleSystem, tspan, init);
title = {'Vertical Equilibrium, State Trajectories with initial state [ 0 0 0.1\pi 0]';
         'Vertical Equilibrium, State variables with time domain characteristics'};
controlled_pendulum_plotter(t,x, init, P_up_complex,[5,6], title);


% animation.Draw(x(:,1), x(:,3), t);
% animation.Close();


%% State Trajectories Vertical, Initial theta displacement, poles close to origin
function controlled_pendulum_plotter(t,x, init, P, fignum, title)
    % This is a function that maes this file shorter 
    global K x_d 
    q = (x-repmat(x_d', [size(x,1), 1, 1]));
    for i = 1:size(x, 1)
    u(i) = -K*q(i,:)';
    end 
    Climits = max([abs(real(P)), abs(imag(P))]);
    
    % Plot State Trajectories
    stat_traj = figure('Name',num2str(fignum(1),1),'units','normalized','outerposition',[0 0 1 .95]);
    left = subplot(2,2,1); hold on;
    plot(init(3), init(4), 'b*','LineWidth',2);
    plot(x(:,3), x(:,4), 'k-','LineWidth',2); hold off; 
    right = subplot(2,2,2);
    hold on;
    plot(init(1), init(2), 'b*','LineWidth',2);
    plot(x(:,1), x(:,2), 'k-','LineWidth',2); hold off; 
    
    input = subplot(2,2,3); 
    plot(t, u, 'k-','LineWidth',2); hold off; 
    poles = subplot(2,2,4); hold on; 
    %plot(real(P(1,:)), imag(P(1,:)),'k*','LineWidth',2);
    plot(real(P(1,:)), imag(P(1,:)),'b+','LineWidth',2);
    hold off; 
    
    % Set up state trajectory plots
    left.XLabel.String='Angular displacement from vertical: x_3 = \theta (radians)';
    left.YLabel.String='Angular rate x_4 = d\theta/dt (rad/s)';
    left.FontSize = 16;
    left.FontWeight = 'bold';
    left.LineWidth = 1.5;
    right.XLabel.String='Cart position: x_1 = s (m)';
    right.YLabel.String='Cart velocity x_2 = ds/dt (m/s)';
    right.FontSize = 16;
    right.FontWeight = 'bold';
    right.LineWidth = 1.5;
    legend(right,'Initial state', 'Nonlinear Controlled Trajectory','Location', 'best');
    input.FontSize = 16; 
    input.FontWeight = 'bold';
    input.XLabel.String='time (seconds)';
    input.YLabel.String='Input u (m/s^2)'; 
    poles.FontSize = 16; 
    poles.FontWeight = 'bold';
    poles.XLabel.String='Real Pole Part';
    poles.YLabel.String='Imaginary Pole Part (j)'; 
    poles.XLim = [-Climits Climits];
    poles.YLim = [-Climits Climits];
    suptitle(title(1,:));
    grid on
    hold off; 
    
    f2 = figure('Name',num2str(fignum(2),1),'units','normalized','outerposition',[0 0 1 .95]);

    % Plot Time Data 
    subplot(2,2,1);
    hold on;
    plot(t, x(:,1), 'k-','LineWidth',2);
    hold off
    left = subplot(2,2,1);
    left.XLabel.String='time (s)';
    left.YLabel.String='Cart position: x_1 = s (m)';
    left.FontSize = 16;
    left.FontWeight = 'bold';
    left.LineWidth = 1.5;

    subplot(2,2,2);
    hold on;
    plot(t, x(:,3), 'k-','LineWidth',2);
    hold off
    right = subplot(2,2,2);
    right.XLabel.String='time (s)';
    right.YLabel.String='Angle: x_3 = \theta (rad)';
    right.FontSize = 16;
    right.FontWeight = 'bold';
    right.LineWidth = 1.5;
    %right.YLim =[0 6*pi];
    %right.YTick =[0 0.25*pi 0.5*pi 0.75*pi pi 1.25*pi 1.5*pi 1.75*pi  2*pi];
    %right.YTickLabel = {'0' '0.25\pi' '0.5\pi' '0.75\pi'  '\pi' '1.25\pi' '1.5\pi' '1.75\pi' '2\pi'};

    %legend('Nonlinear Trajectory', 'Linearized Trajectory', 'Location', 'best');
    suptitle(title(2,:));
    
    % Calculate some stats 
    tdat = stepinfo(x(:,3), t, x_d(3)); 
    sdat = stepinfo(x(:,1), t, 0);

    tabledata = {' ' 'Settling Time' 'Rise Time' 'Settling Min'; ...
                 'theta:' tdat.SettlingTime tdat.RiseTime tdat.SettlingMin; ...
                 ' ' 'Settling Time' 'Peak Time' 'Peak Input';
                 's:' sdat.SettlingTime sdat.PeakTime sdat.Peak}; 
    % create the data
    % Create the column and row names in cell arrays 
    cnames = {'1','2','3','4'};
    rnames = {'1','2','3','4'};
    % Create the uitable
    table = uitable(f2,'Data',tabledata,...
                'ColumnName',cnames,... 
                'RowName',rnames,...
                'ColumnWidth',{180}, ...
                'FontSize', 16, ...
                'FontWeight', 'bold'); 
    subplot(2,2,[3,4])
    pos = get(subplot(2,2,[3,4]),'position');
    delete(subplot(2,2,[3,4]))
    set(table,'units','normalized')
    set(table,'position',pos)

end
