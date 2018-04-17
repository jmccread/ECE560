% LinearizationWork.m
%
% Created by J. McCready on 2018-01-18
% ECE 560 Winter 18
% University of Michigan - Dearborn
%
% Last updated: 2018-04-04 by J. McCready
%   - Work to linearize system after some initial manipulation of the
%   equations from the prompt

clear all
syms L Mt m l x1 x2 x3 x4 u1 u2 u3 u4 g w1 w2 w3 w4 
% Define G(x) * u + H(x) = f(x, u) = dot(x)
G =  [
     0 0 0 0; ...
     0 L/(Mt*L - m*l*cos(x3).^2) 0 0; ...
     0 0 0 0; ...
     0 0 0 -cos(x3)/(Mt*L - m*l*cos(x3).^2)
     ];
K  =  [
     0 0 0 0; ...
     0 0 0 0; ...
     0 0 0 0; ...
     0 0 0 1/(Mt*L - m*l*cos(x3).^2)
     ];
H = [ 
    x2; ...
    m*l/(Mt*L - m*l*cos(x3).^2)*(-g*sin(x3)*cos(x3) +L*sin(x3)*x4^2); ...
    x4; ...
    1/(Mt*L - m*l*cos(x3).^2)*(-m*l*sin(x3)*cos(x3)*x4^2 +Mt*g*sin(x3))
    ];
u = [u1; u2; u3; u4];
w = [w1; w2; w3; w4];
eq = [0; 0; 0; 0];
ubar = G^-1*(eq - H); 
f = K*w + G*u + H; 
% Determine A and B matricies using diff command 
JacobianA = [jacobian(f,x1), jacobian(f,x2), jacobian(f,x3), jacobian(f,x4)];
JacobianB = [jacobian(f,u1), jacobian(f,u2), jacobian(f,u3), jacobian(f,u4)];
JacobianBd = [jacobian(f,w1), jacobian(f,w2), jacobian(f,w3), jacobian(f,w4)];


syms s0
x_up_star = [s0; 0; 0; 0]; 
x_down_star = [s0; 0; pi; 0];

% % Substitute in real numbers 
% M = 2.0; %kg
% m = 0.1; %kg
% l = 0.5; %m
% J = 0.025; %kg*m^2
% g = 9.8;  %m/s^2
% Mt = M + m; 
% L = (J+m*l^2)/(m*l);

% Linearization for the vertical position
x1 = s0; x2 = 0; x3 = 0; x4 = 0; 
u1 = 0; u2 = 0; u3 = 0; u4 = 0; 

A_up = subs(JacobianA);
B_up = subs(JacobianB); 
Bd_up = subs(JacobianBd); 


% Linearization for the hanging position
x1 = s0; x2 = 0; x3 = pi; x4 = 0; 
u1 = 0; u2 = 0; u3 = 0; u4 = 0;

A_down = subs(JacobianA);
B_down = subs(JacobianB);
Bd_down = subs(JacobianBd); 

A_num_up = eval(A_up); 
B_num_up = eval(B_up)*[1;1;1;1];
Bd_num_up = eval(Bd_up)*[1;1;1;1];

A_num_down = eval(A_down); 
B_num_down = eval(B_down)*[1;1;1;1]; 
Bd_num_down = eval(Bd_down)*[1;1;1;1]; 

%% Inspect the internal stability of the hanging and vertical conditions
% syms t 
% %expA_up = expm(t*A_num_up);
% %expA_up = simplify(expA_up); 
% 
% expA_down = exp(t*A_num_down);
% %expA_down = simplify(expA_down);

%% Compute Controllability and Observability matricies
C_M_up = ctrb(A_num_up, B_num_up); 

C_M_down = ctrb(A_num_down, B_num_down); 

C_M_up_d = ctrb(A_num_up, [B_num_up, Bd_num_up]); 

C_M_down_d = ctrb(A_num_down, [B_num_down, Bd_num_dowm_d]); 

O_M_up = obsv(A_num_up,ones(1,4)); 

O_M_down = obsv(A_num_down, ones(1,4)); 