%% ME227 Assignment #4

clear all; close all;
setup_niki;

load('project_data.mat')
load('desired.mat')
aX = zeros(1009,1); 
K_T = path.k_1pm
% % Comparison
% 
% clear all; close all;
% load('project_data.mat')
% load('desired.mat')
% load('desired_profiles_1.mat')
% paths =path.s_m;
% plot(paths,ax_des,paths,aX)
% legend('ideal','current')

%% Problem 3 Code Here:
clear all; 
setup_niki;

% path information
init = [0;0;0];
% straight  path
load('desired_profiles_1.mat')
load('project_data.mat')
% load('desired.mat')
path.axDes = aX;
path.UxDes = Udes;

% path.axDes = ax_des;
% path.UxDes = Ux_des;
g = 9.81;                   	% gravity acceleration, meters/sec^2

% vehicle parameters
setup_niki;
m = veh.m;
Caf = tire_f.Ca;
Car = tire_r.Ca;
a = veh.a;
b = veh.b;
Iz = veh.Iz;
L = veh.L;

% simulation time
% t_final = 36.5;
% t_final = 37.5;
t_final = 35.5;
dT =0.005;
t_s = 0:dT:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps         = zeros(N,1);
uy_mps          = zeros(N,1);
ux_mps          = zeros(N,1);
dpsi_rad        = zeros(N,1);
s_m             = zeros(N,1);
e_m             = zeros(N,1);
delta_rad       = zeros(N,1);
dot_dpsi_rad    = zeros(N,1);
dot_e_m         = zeros(N,1);
ax              = zeros(N,1);
ay              = zeros(N,1);
K_lqr           = zeros(N,4);

% set initial conditions

ux_mps(1)      = 1;
e_m(1)         = 0;

for idx = 1:N

    % look up K
    K = interp1(path.s_m, path.k_1pm, s_m(idx));
    
    r = r_radps(idx)+normrnd(0.00,0.01); %0.01*randn;
    uy = uy_mps(idx) + normrnd(0.00,0.01);%0.01*randn;
    ux = ux_mps(idx) +normrnd(0.00,0.01);% 0.01*randn;
    dpsi = dpsi_rad(idx) + normrnd(0.00,0.01);%0.01*randn;
    s = s_m(idx);
    e = e_m(idx) + normrnd(0.00,0.01);%0.01*randn;
%     
    
    [ delta, Fx, Fx1 ,Fx2,K1] = me227_controller( s, e, dpsi, ux, uy, r,1, path);
 
    %Calculate the Dynamics with the Nonlinear Bike Model
    [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r);
    ax(idx) = ux_dot;
    ay(idx) = K*ux^2;
    F1(idx) = Fx1;
    F2(idx) = Fx2;
    F(idx) = Fx;
    gains(idx,:)=K1;
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    if idx < N
        
        % Euler integration
        z = integrate_euler(([r,uy,ux,s,e,dpsi]'),...
            ([ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot]'),dT); 
    r_radps(idx + 1) = z(1);
    uy_mps(idx + 1) = z(2);
    ux_mps(idx + 1) = z(3);
    s_m(idx + 1) = z(4);
    e_m(idx + 1) = z(5);
    dpsi_rad(idx + 1) = z(6);
    end
    
end

figure(7);
subplot(2,3,1); hold on; grid on;
    plot(t_s, r_radps)
    xlabel('Time [s]')
    ylabel('r [radps]')
subplot(2,3,2); hold on; grid on;
    plot(t_s, uy_mps)
    xlabel('Time [s]')
    ylabel('u_y [mps]')
subplot(2,3,3); hold on; grid on;
    plot(t_s, ux_mps)
    xlabel('Time [s]')
    ylabel('u_x [mps]')
subplot(2,3,4); hold on; grid on;
    plot(t_s, dpsi_rad)
    xlabel('Time [s]')
    ylabel('\Delta\psi [rad]')
subplot(2,3,5); hold on; grid on;
    plot(t_s, e_m)
    xlabel('Time [s]')
    ylabel('e [m]')
subplot(2,3,6); hold on; grid on;
    plot(t_s, s_m)
    xlabel('Time [s]')
    ylabel('s [m]')
% animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)

figure(10);
title('Acceleration Profile with noise')
subplot(2,1,1)
plot(t_s,ax);grid on;
xlabel('Time(s)')
ylabel('Acceleration in x m/s^2')
subplot(2,1,2)
plot(t_s,ay);grid on;
xlabel('Time(s)')
ylabel('Acceleration in y m/s^2')
figure(12)
plot(t_s,gains(:,1),t_s,gains(:,2),t_s,gains(:,3),t_s,gains(:,4));
legend('1','2','3','4')
plot(t_s,delta_rad)
figure(11);
plot(t_s,F1);hold on;
plot(t_s,F2);hold on;
plot(t_s,F)
legend(' drag, rolling resistance terms','Ux error term','total')

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%STUDENT FUNCTIONS BELOW
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate Forces with the Fiala Nonlinear Tire Model
function Fy = fiala_model(alpha, tire)
%   Calculate tire forces with the fiala model

%%%%% STUDENT CODE HERE %%%%%
alpha_sl = atan2((3*tire.mu*tire.Fz)/tire.Ca,1);
if abs(alpha) < alpha_sl
Fy = -tire.Ca*tan(alpha) + ((tire.Ca^2)/(3*tire.mu*tire.Fz))*(2-(tire.mu_s/tire.mu))*...
    abs(tan(alpha))*tan(alpha) - ((tire.Ca^3)/(9*tire.mu^2*tire.Fz^2))*tan(alpha)^3*...
    (1-((2*tire.mu_s)/(3*tire.mu))); 
else
Fy = - tire.mu_s*tire.Fz*sign(alpha);
end

%%%%% END STUDENT CODE %%%%%
end

%Calculate the Nonlinear Bicycle Model Dynamics
function [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
    nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r )
%KINEMATIC_MODEL
%   Calculate state derivatives for the kinematic vehicle model

%%%%% STUDENT CODE HERE %%%%%
frr = 0.015; %
C_DA = 0.594; %
rho = 1.225; %
g = 9.81;
% slip angles
[alphaF, alphaR] = slip_angles( r, uy, ux, delta, veh);

% lateral tire forces
fyf = fiala_model(alphaF, tire_f);
fyr = fiala_model(alphaR, tire_r);

%Split longitudinal force based on drive and brakedistribution
if Fx > 0
    fxf = Fx;
    fxr = 0; 
else
    fxf = Fx/2;
    fxr = Fx/2;
end

% dynamics

ux_dot = ((fxr + fxf*cos(delta) - fyf*sin(delta))/veh.m) + r*uy;
ux_dot = ux_dot + ( -frr*veh.m*g - 0.5*rho*C_DA*(ux^2) )/veh.m + 0.001*rand;
uy_dot = ((fyf*cos(delta) + fyr + fxf*sin(delta))/veh.m) - r*ux;
r_dot = (1/veh.Iz)*(veh.a*fyf*cos(delta) + veh.a*fxf*sin(delta) - veh.b*fyr);
s_dot = (1/(1 - e*K))*(ux*cos(dpsi) - uy*sin(dpsi));
e_dot = uy*cos(dpsi) + ux*sin(dpsi);
dpsi_dot = r - K*s_dot; %+ 0.001*rand;

%%%%% END STUDENT CODE %%%%%
end

%Calculate the Slip Angles Here:
function [alphaF, alphaR] = slip_angles( r, uy, ux, delta, veh)
%slip_angles
%   calculate the tire slip angles

%%%%% STUDENT CODE HERE %%%%%
alphaR = atan2(((uy - (veh.b).*r)/(ux)),1);
alphaF = atan2(((uy + (veh.a).*r)/(ux)),1) - delta;
%%%%% END STUDENT CODE %%%%%
end

%Use standard Euler Integration
function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state

%%%%% STUDENT CODE HERE %%%%%
x1 = x0 + x0_dot*dt;
%%%%% END STUDENT CODE %%%%%
end
