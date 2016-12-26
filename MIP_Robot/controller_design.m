% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Controller_design.m 							  %
%                                                                         %
% The plant and digital controller for the MIP robot's body angle and     %
% wheel position are designed and tested in MATLAB.                       %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

clear all
close all

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 1: Stablizing Body Angle                                           %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Initialize constants used in equations of motion

s_bar = 0.003;              % stall torque in Nm (Newton-meter)
w_f = 1760;                 % free run speed in rad/s
k = s_bar/w_f;              % torque constant k
G_r = 35.57;                % gearbox ratio 
M_b = 263*10^-3;            % total assembled MIP body mass in Kg
R_w = 34*10^-3;             % radius of wheels in meters 
L = 36*10^-3;               % dist from MIP c.o.m to wheel axis
I_b = 0.0004;               % MIP body inertia about wheel axis in Kg * m^2
g = 9.8;                    % gravity in m/2^2 
M_w = 27*10^-3;             % mass of wheels in Kg
I_m = 3.6*10^-8;            % inertia of motor armature in Kg * m^2

% combined wheel inertia
I_w = 2*((M_w*(R_w^2))/2 + (G_r^2)*I_m);     

% define new constants for ease of calculations and human readability 

c1 = (I_w + (M_b + M_w)*(R_w^2));
c2 = M_b*R_w*L;
c3 = I_b + M_b*L^2;
c4 = M_b*g*L;
c5 = 2*G_r*s_bar;
c6 = 2*(G_r^2)*k;

% Plug in transfer function constant values into open loop plant
% G1(s) = MIP angle/duty cycle 
num = [(c1*c5+c2*c5) 0];
den = [c2^2-c1*c3 (-c1*c6-2*c2*c6-c3*c6) c1*c4 c4*c6];
G1_s = tf(num,den);
roots(den);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 1.1 Plant G(s) Modeling for MIP Body Angle                         %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Plot root locus of G1_s
subplot(2,3,1)
rlocus(G1_s)
title('Root locus of G1')

% Plot bode of G1_s
subplot(2,3,2)
bode(G1_s)
title('Bode of G1')

% plot step response for closed loop of G1_s
subplot(2,3,3)
T1_s = G1_s/(1+G1_s);
step(T1_s)
title('Step Response of CL')

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 1.2 Designing D1(s) Controller for G1(s)                           %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Design D1 to stabilize G1
num = [-0.148 -9.6 -155];
den = [0.00083 1 0];
D1_s = tf(num,den)

H1_s = G1_s*D1_s;

% Plot root locus of G1*D1
subplot(2,3,4)
rlocus(D1_s*G1_s)
title('Root locus of D1*G1')

% Plot step response to closed loop of D1*G1
subplot(2,3,6)
T1_s = (D1_s*G1_s)/(1+D1_s*G1_s);
step(T1_s)
title('Step Response to CL')

% Plot bode of open loop D1*G1
subplot(2,3,5)
bode(D1_s*G1_s)
title('Bode of D1*G1')

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 1.3 Discretize transfer function to difference equation to be      %
% implemented in Beaglebone                                               %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Sampling period 
Ts = 0.005;

% Convert laplace to z domain using Tustin's approximation with prewarping
opt = c2dOptions('Method', 'tustin', 'PrewarpFrequency', 107);
D1_z = c2d(D1_s,Ts,opt)

% Convert z transfer function into difference equation
[num,den] = tfdata(D1_z);
D1_k = idpoly(den,num,'NoiseVariance',0);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 2: Stablizing Wheel Position                                       %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 2.1: Plant G2(s) Modeling for Wheel Position                       %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% G2(s) = MIP wheel position/MIP Body Angle 
num = [-(c2+c3)/(c1+c2) 0 c4/(c1+c2)];
den = [1 0 0];
G2_s = tf(num,den);
roots(num)

figure
% Plot root locus of G2_s
subplot(2,3,1)
rlocus(G2_s)
title('Root locus of G2')

% Plot bode of G2_s
subplot(2,3,2)
bode(G2_s)
title('Bode of G2')

% plot step response for closed loop of G2_s
subplot(2,3,3)
T2_s = G2_s/(1+G2_s);
step(T2_s)
title('Step Response of CL')

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 2.2: Design controller D2(s) for G2(s) with T1(s) = 1 for now      %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Design D2 to stabilize G2
num = [0.125 0.125*0.2];
den = [1 9.3438];
D2_s = tf(num,den)

H2_s = G2_s*D2_s;

% Plot root locus of G2*D2
subplot(2,3,4)
rlocus(D2_s*G2_s)
title('Root locus of D2*G2')

% Plot step response to closed loop of D2*G2
subplot(2,3,6)
T2_s = (D2_s*G2_s)/(1+D2_s*G2_s);
step(T2_s)
title('Step Response to CL')

% Plot bode of open loop D2*G2
subplot(2,3,5)
bode(D2_s*G2_s)
title('Bode of D2*G2')

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 2.3: Include inner loop for modeled G1(s) and D1(s)                %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
k = 1.9;
CL_num = k*D2_s*T1_s*G2_s;
CL_den = 1+CL_num;
CL = CL_num/CL_den;
figure 
step(CL)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Part 2.4 Discretize transfer function D2(s) to difference equation to   %
% be implemented in Beaglebone      .                                     %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Sampling period 
Ts = 0.05;

% Convert laplace to z domain using Tustin's approximation with prewarping
opt = c2dOptions('Method', 'tustin', 'PrewarpFrequency', 1.63);
D2_z = c2d(D2_s,Ts,opt)

% Convert z transfer function into difference equation
[num,den] = tfdata(D2_z);
D2_k = idpoly(den,num,'NoiseVariance',0);









