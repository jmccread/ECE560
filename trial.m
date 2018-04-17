%% 
m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

A=[0                 1   0                                              0
  -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)
   b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1
   k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0];
B=[0                 0
   1/m1              (b1*b2)/(m1*m2)
   0                -(b2/m2)
   (1/m1)+(1/m2)    -(k2/m2)];
C=[0 0 1 0];
D=[0 0];
sys=ss(A,B,C,D);

Aa=[0                 1   0                                              0         0
   -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)   0
    b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1         0
    k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0         0
    0                 0   1                                              0         0];
Ba=[0                 0
    1/m1              (b1*b2)/(m1*m2)
    0                -(b2/m2)
    (1/m1)+(1/m2)    -(k2/m2)
    0                 0];
Ca=[0 0 1 0 0];
Da=[0 0];
sys=ss(Aa,Ba,Ca,Da);

Aa = [[A,[0 0 0 0]'];[C, 0]];
Ba = [B;[0 0]];
Ca = [C,0];
Da = D;
sys=ss(Aa,Ba,Ca,Da);

K = [0 2.3e6 5e8 0 8e6]

t = 0:0.01:2;
sys_cl = ss(Aa-Ba(:,1)*K,-0.1*Ba,Ca,Da);
step(sys_cl*[0;1],t)
title('Closed-Loop Response to a 0.1-m Step')

%% Trial 2 
J = 3.2284E-6;
b = 3.5077E-6;
K = 0.0274;
R = 4;
L = 2.75E-6;

A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];
B = [0 ; 0 ; 1/L];
C = [1  0  0];
D = 0;
motor_ss = ss(A,B,C,D);

p1 = -100+100i;
p2 = -100-100i;
p3 = -200;
Kc = place(A,B,[p1, p2, p3])

Aa = [0 1 0 0
      0 -b/J K/J 0
      0 -K/L -R/L 0
      1 0 0 0];
Ba = [0 ; 0 ; 1/L ; 0 ];
Br = [0 ; 0 ; 0; -1];
Ca = [1 0 0 0];
Da = [0];

p4 = -300;
Ka = place(Aa,Ba,[p1,p2,p3,p4]);

t = 0:0.001:.05;
sys_cl = ss(Aa-Ba*Ka,Br,Ca,Da);
step(sys_cl,t)