% LinearizationWork.m
%
% Created by J. McCready on 2018-01-18
% ECE 560 Winter 18
% University of Michigan - Dearborn
%
% Last updated: 2018-04-04 by J. McCready
%   - Work to linearize system after some initial manipulation of the
%   equations from the prompt

syms L Mt m l x1 x2 x3 x4 u1 u2 u3 u4 g

% Define G(x) * u + H(x) = f(x, u) = dot(x)

G =  [
     1 0 0 0; ...
     0 L/(Mt*L - m*l*cos(x3).^2) 0 0; ...
     0 0 1 0; ...
     0 0 0 -cos(x3)/(Mt*L - m*l*cos(x3).^2)
     ];

H = [ 
    x2; ...
    m*l/(Mt*L - m*l*cos(x3).^2)*(-g*sin(x3)*cos(x3) +L*sin(x3)*x4^2); ...
    x4; ...
    1/(Mt*L - m*l*cos(x3).^2)*(-m*l*sin(x3)*cos(x3)*x4^2 +Mt*g*sin(x3))
    ];
    
u = [u1; u2; u3; u4]; 

eq = [0; 0; 0; 0];

ubar = G^-1*(eq - H); 

f = G*u + H; 

% Determine A and B matricies using diff command 
JacobianA = [diff(f,x1), diff(f,x2), diff(f,x3), diff(f,x4)];
JacobianB = [diff(f,u1), diff(f,u2), diff(f,u3), diff(f,u4)];

syms s0
x_up_star = [s0; 0; 0; 0]; 
x_down_star = [s0; 0; pi; 0]; 

% Linearization for the vertical position
x1 = s0; x2 = 0; x3 = 0; x4 = 0; 
u1 = 0; u2 = 0; u3 = 0; u4 = 0; 

A_up = subs(JacobianA);
B_up = subs(JacobianB); 

% Linearization for the hanging position
x1 = s0; x2 = 0; x3 = pi; x4 = 0; 
u1 = 0; u2 = 0; u3 = 0; u4 = 0;

A_down = subs(JacobianA);
B_down = subs(JacobianB); 
