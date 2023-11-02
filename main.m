clc;clear;close all
%% Draw nyquist plot for main transfer function    
s = tf('s'); 
G = 10/((s+1)*(s+5)*s); % plant transfer function 
H=1; % feedback transfer function
Gcl = G/(1+G*H); % close loop transfer function before adding compensator 
figure("Name","main transfer function")
nyquist(Gcl); % draw nyquist plot
%% The main transfer function and ploting root locus and step response
figure("Name","root locus-The main transfer function")
rlocus(G*H); grid on
hold on; 
plot([-1.4 -1.4], [1.47 -1.47], '*') % root locus with goal points
figure("Name","step response-The main transfer function")
step(Gcl); grid on % step response to main trasnfer function
stepinfo(Gcl)
%% Design lead compensator and ploting step response
Tetac = 80; % the compensator angel
Pc = -4.4; % compensator pole position
Zc = -.96; % compensator zero position

Gc_lead = (s + .96)/(s + 4.4); % lead compensator

S_1 = [-1.4 + 1.47i ;-1.4 - 1.47i]; % the gola points

G_func = @(x)(10/((x+1)*(x+5)*x));
C_func = @(x)((x + .96)/(x + 4.4));

K1 = abs(-1/(G_func(S_1(1))*C_func(S_1(1)))); % k1 is 2.6183 for the lead
% Kp for this system is infinity
Gc1_lead = K1*Gc_lead;
M1 = (G*Gc1_lead)/(1 + G*Gc1_lead);
figure("Name","lead compensator and ploting step response")
step(M1); grid on
stepinfo(M1)
%% Design lead lag compensator and ploting step response
Gc_lag = (s + .1)/s; % lag compensator
M2 = (G*Gc1_lead*Gc_lag)/(1 + G*Gc1_lead*Gc_lag);
figure("Name","lead lag compensator")
step(M2); grid on
stepinfo(M2)
% the results showing that lag compensator has badly effect on this system
%% Design lag compensator and ploting step response
M3 = (G*Gc_lag)/(1 + G*Gc_lag);
figure("Name","lag compensator");
step(M3); grid on
stepinfo(M3)
%% PD-Controller     
G = 10 / (s * (s + 1) * (s + 5)); % desired phase margin -> 60 degree
K = 2.5; %based on desired Kv -> 5
alpha = 0.0931;
T = 0.9185;
Gc = (1 + T * s)/(1 + T * alpha * s); %designated controller
figure("Name" , "Frequency Response")
subplot(1,2,1)
margin(K * G)
text(5 , 100,"Before adding controller",'Color','red')
subplot(1,2,2)
margin(K * Gc * G)
text(50 , 100,"After adding controller",'Color','green')
figure("Name","Time Response")
step(feedback(G,1),feedback(K*Gc*G,1))
legend("before","After")
%% PI-Controller
G = 10 / (s * (s + 1) * (s + 5)); % desired phase margin -> 60 degree
K = 2.5; %based on desired Kv -> 5
beta = 9.6605;
T1 = 21.3675;
Gl = (1 + T1 * s)/(1 + T1 * beta * s); %designated controller
figure("Name","Frequency Response")
subplot(1,2,1)
margin(K * G)
text(5 , 100,"Before adding controller",'Color','red')
subplot(1,2,2)
margin(K * Gl * G)
text(2 , 100,"After adding controller",'Color','green') %better phase margin correction
figure("Name","Time Response")
step(feedback(G,1),feedback(K*Gl*G,1))
legend("before","After")
%% PID-Controller (with 2 extra PD controller for better performance)
G = 10 / (s * (s + 1) * (s + 5)); % desired phase margin -> 60 degree
K = 2.5; %based on desired Kv -> 5
alpha1 = 0.2710;
T2 = 0.6861;
Gc1 = (1 + T2 * s)/(1 + T2 * alpha1 * s); %designated controller
beta2 = 1.5613;
T3 = 4.8077;
Gl2 = (1 + T3 * s)/(1 + T3 * beta2 * s); %designated controller
alpha2 = 0.3207;
T4 = 0.5814;
Gc2 = (1 + T4 * s)/(1 + T4 * alpha2 * s); %designated controller
alpha3 = 0.6104;
T5 = 0.3404;
Gc3 = (1 + T5 * s)/(1 + T5 * alpha3 * s); %designated controller
figure('Name','Stage 1')
subplot(1,2,1)
margin(K * G)
text(5 , 100,"Before adding controller",'Color','red')
subplot(1,2,2)
margin(K * Gc1 * G)
text(10 , 100,"After adding first controller",'Color','green')
figure('Name',"Stage 2")
subplot(1,2,1)
margin(K * Gl2 * Gc1 * G)
text(5 , 100,"After adding Second controller",'Color','green')
subplot(1,2,2)
margin(K * Gc1 * Gl2 * Gc2 * G)
text(2 , 100,"After adding Third controller",'Color','green')
figure('Name',"Stage 3")
margin(K * Gc1 * Gl2 * Gc2 * Gc3 * G)
text(5 , 100,"After adding Last controller",'Color','green')
figure("Name","Time Response")
step(feedback(G,1),feedback(K*Gc1*G,1),feedback(K*Gc1*Gl2*G,1),feedback(K * Gc1 * Gl2 * Gc2 * G,1),feedback(K * Gc1 * Gl2 * Gc2 * Gc3 * G,1))
legend("before","Controller 1" , "Controller 2","Controller 3","Controller 4")
%% Compensator Design with MATLAB    
Gpd_Matlab = (1.36) * (1+1.19*s); % PD Control in Time Domain with Matlab(pidTunner Command) -> settling time is 1.27s
%margin(G*Gpd_Matlab)
Closedloop_Matlab = (G*Gpd_Matlab)/(1 + G*Gpd_Matlab);
%step(M)
figure("Name","Step Response with PD Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
Gpi_Matlab = (0.127) * (1+1/(9.05*s)); % PI Control in Time Domain with Matlab(pidTunner Command) -> Kp = 0.127 , Ti = 9.05
%margin(G*Gpi_Matlab)
Closedloop_Matlab = (G*Gpi_Matlab)/(1 + G*Gpi_Matlab); 
figure("Name","Step Response with PI Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
Gpid_Matlab = (0.064) * (1+1/(450*s)+0.582); % PID Control in Time Domain with Matlab(pidTunner Command) -> Kp=0.064 , Ti = 450 , Td = 0.582
%margin(G*Gpid_Matlab)
Closedloop_Matlab = (G*Gpid_Matlab)/(1 + G*Gpid_Matlab); 
figure("Name","Step Response with PID Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
Gpd_fMatlab = (0.554) * (1+0.444*s); %PD Control in Frequency Domain with Matlab -> phaseMargin = 60 deg
Closedloop_Matlab = (G*Gpd_fMatlab)/(1 + G*Gpd_fMatlab); 
figure("Name","Step Response with PD feq Domain Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
Gpi_fMatlab = (0.142) * (1+1/(18*s)); %PI Control in Frequency Domain with Matlab --> phaseMargin = 60 deg , Kp=0.142 , Ti=18
Closedloop_Matlab = (G*Gpi_fMatlab)/(1 + G*Gpi_fMatlab); 
figure("Name","Step Response with PI feq Domain Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
Gpid_fMatlab = (0.429) * (1+1/(6.22*s)+1.56*s) ;%PID Control in Frequency Domain with Matlab --> phaseMargin = 60 deg , Kp=1.65 , Ti=5 , Td=0.987
Closedloop_Matlab = (G*Gpid_fMatlab)/(1 + G*Gpid_fMatlab); 
figure("Name","Step Response with PID feq Domain Compensator with MATLAB")
step(Closedloop_Matlab);grid on;
figure("Name","Selected Nyquist Diagram Controller")
nyquist((G*Gpd_Matlab)/(1 + G*Gpd_Matlab));grid on;
title("Nyquist Diagram Of System with Compensator");
figure("Name","Selected root locus Diagram Controller")
rlocus(Gpd_Matlab * G); grid on;
%title("Root Locus Of System with Compensator");