%% ME227 Assignment #4
% code template
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
setup_niki;
%% Problem 1 Code Here:
delta = 3 * pi/180;
e = 1;
K_la =  (delta * tire_f.Ca_lin)/e

%% 1.2
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
K_la = 4188.79;
x_la = 0;
Ux   = [5, 10, 15, 20, 25, 30];
for i = 1:6
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux(i)), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-veh.a * tire_f.Ca_lin + veh.b * tire_r.Ca_lin)/(veh.m*Ux(i)) ; ...
         0, 0, 0, 1; ...
         -veh.a * K_la/veh.Iz, (veh.b * tire_r.Ca_lin - veh.a * tire_f.Ca_lin)/(veh.Iz * Ux(i)),...
         ((veh.a * tire_f.Ca_lin - veh.b * tire_r.Ca_lin)/(veh.Iz))-(K_la*veh.a*x_la/veh.Iz),...
         -(veh.a^2 * tire_f.Ca_lin + veh.b^2 * tire_r.Ca_lin)/(veh.Iz*Ux(i))];
    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    figure(1); hold on; %grid on;
    %plot(t,e);
    pzmap(sys);
end
title('Pole Zero Map'); xlabel('Real axis'); ylabel('Imaginary axis');
legend('5 m/s','10 m/s','15 m/s','20 m/s','25 m/s','30 m/s');

%% 1.3
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
K_la = 4188.79;
x_la = 10;
for i = 1000:1000:10000
    K_la = i;
    Ux(i) = 15;
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux(i)), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-veh.a * tire_f.Ca_lin + veh.b * tire_r.Ca_lin)/(veh.m*Ux(i)) ; ...
         0, 0, 0, 1; ...
         -veh.a * K_la/veh.Iz, (veh.b * tire_r.Ca_lin - veh.a * tire_f.Ca_lin)/(veh.Iz * Ux(i)),...
         ((veh.a * tire_f.Ca_lin - veh.b * tire_r.Ca_lin)/(veh.Iz))-(K_la*veh.a*x_la/veh.Iz),...
         -(veh.a^2 * tire_f.Ca_lin + veh.b^2 * tire_r.Ca_lin)/(veh.Iz*Ux(i))];
    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    figure(2); hold on; %grid on;
    %plot(t,e);
    pzmap(sys);
end
title('Pole Zero Map'); xlabel('Real axis'); ylabel('Imaginary axis');
legend('Kla = 1000','Kla = 2000','Kla = 3000','Kla = 4000','Kla = 5000',...
    'Kla = 6000','Kla = 7000','Kla = 8000','Kla = 9000','Kla = 10000' );

%% 1.4 
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
% K_la = 
x_la = 15;
Ux   = 15;
% A    = [, , , ; ...
%         , , , ; ...
%         , , , ; ...
%         , , , ];
K = [1000, 10000];
for i = 1:2
    K_la = K(i);
    Ux(i) = Ux;
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux(i)), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-veh.a * tire_f.Ca_lin + veh.b * tire_r.Ca_lin)/(veh.m*Ux(i)) ; ...
         0, 0, 0, 1; ...
         -veh.a * K_la/veh.Iz, (veh.b * tire_r.Ca_lin - veh.a * tire_f.Ca_lin)/(veh.Iz * Ux(i)),...
         ((veh.a * tire_f.Ca_lin - veh.b * tire_r.Ca_lin)/(veh.Iz))-(K_la*veh.a*x_la/veh.Iz),...
         -(veh.a^2 * tire_f.Ca_lin + veh.b^2 * tire_r.Ca_lin)/(veh.Iz*Ux(i))];

    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    y    = lsim(sys,zeros(size(t)),t,x0)';  % u = 0, autonomous system
    e    = y(1,:);
    dPsi = y(3,:);
    figure(3); hold on; grid on;
    plot(t,e);
end
title('1.4'); xlabel('time [s]'); ylabel('lateral error [m]');
legend('K_la = 1000', 'K_la = 10000');
 
%% 1.5
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
% K_la = 
Ux   = 15;
K_la = 4188.79;
for i = 2:2:20
    x_la = i;
    Ux(i) = 15;
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux(i)), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-veh.a * tire_f.Ca_lin + veh.b * tire_r.Ca_lin)/(veh.m*Ux(i)) ; ...
         0, 0, 0, 1; ...
         -veh.a * K_la/veh.Iz, (veh.b * tire_r.Ca_lin - veh.a * tire_f.Ca_lin)/(veh.Iz * Ux(i)),...
         ((veh.a * tire_f.Ca_lin - veh.b * tire_r.Ca_lin)/(veh.Iz))-(K_la*veh.a*x_la/veh.Iz),...
         -(veh.a^2 * tire_f.Ca_lin + veh.b^2 * tire_r.Ca_lin)/(veh.Iz*Ux(i))];

    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    y    = lsim(sys,zeros(size(t)),t,x0)';  % u = 0, autonomous system
    e    = y(1,:);
    dPsi = y(3,:);
    figure(4); hold on; 
    pzmap(sys);
end
title('1.5'); xlabel('Real axis'); ylabel('Imaginary axis');
legend('x_la = 2 m', 'x_la = 4 m','x_la = 6 m','x_la = 8 m','x_la = 10 m','x_la = 12 m','x_la = 14 m',...
    'x_la = 16 m','x_la = 18 m','x_la = 20 m');

%% 1.6
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
K_la = 4188.79;
x = [2,20];
Ux   = 15;
% A    = [, , , ; ...
%         , , , ; ...
%         , , , ; ...
%         , , , ];
for i = 1:2
    x_la = x(i);
    Ux(i) = 15;
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux(i)), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-veh.a * tire_f.Ca_lin + veh.b * tire_r.Ca_lin)/(veh.m*Ux(i)) ; ...
         0, 0, 0, 1; ...
         -veh.a * K_la/veh.Iz, (veh.b * tire_r.Ca_lin - veh.a * tire_f.Ca_lin)/(veh.Iz * Ux(i)),...
         ((veh.a * tire_f.Ca_lin - veh.b * tire_r.Ca_lin)/(veh.Iz))-(K_la*veh.a*x_la/veh.Iz),...
         -(veh.a^2 * tire_f.Ca_lin + veh.b^2 * tire_r.Ca_lin)/(veh.Iz*Ux(i))];

    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    y    = lsim(sys,zeros(size(t)),t,x0)';  % u = 0, autonomous system
    e    = y(1,:);
    dPsi = y(3,:);
    figure(5); hold on; grid on;
    plot(t,e);
end
title('1.6'); xlabel('time [s]'); ylabel('lateral error [m]');
legend('x_la = 2', 'x_la = 20');


%% Problem 2 Code Here:
wf = 0.3 * veh.m *g;
wr = 0.7 * veh.m *g;
K = ((wf/tire_f.Ca_lin) - (wr/tire_r.Ca_lin))/g
Vcr = sqrt(-veh.L/K)


%% 2.2
t    = 0:.001:10;
x0   = [1;0;0;0];                       % [e,e_dot,dPsi,dPsi_dot]
% K_la = 
a = 0.7 * veh.L;
b = 0.3 * veh.L;
Ux   = 30;
K_la = 3500;
for i = 0:2:30
    x_la = i;
    Ux = 30;
    A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-a * tire_f.Ca_lin + b * tire_r.Ca_lin)/(veh.m*Ux) ; ...
         0, 0, 0, 1; ...
         -a * K_la/veh.Iz, (b * tire_r.Ca_lin - a * tire_f.Ca_lin)/(veh.Iz * Ux),...
         ((a * tire_f.Ca_lin - b * tire_r.Ca_lin)/(veh.Iz))-(K_la*a*x_la/veh.Iz),...
         -(a^2 * tire_f.Ca_lin + b^2 * tire_r.Ca_lin)/(veh.Iz*Ux)];

    B = zeros(4,1);     C = eye(4);     D = B;
    sys  = ss(A,B,C,D);
    y    = lsim(sys,zeros(size(t)),t,x0)';  % u = 0, autonomous system
    e    = y(1,:);
    dPsi = y(3,:);
    figure(6); hold on; 
    pzmap(sys);
end
title('2.2'); xlabel('Real axis'); ylabel('Imaginary axis');
legend('x_la = 2 m', 'x_la = 4 m','x_la = 6 m','x_la = 8 m','x_la = 10 m','x_la = 12 m','x_la = 14 m',...
    'x_la = 16 m','x_la = 18 m','x_la = 20 m','x_la = 22 m','x_la = 24 m','x_la = 26 m','x_la = 28 m','x_la = 30 m');

%% 2.3
t    = 0:.001:10;                       % [e,e_dot,dPsi,dPsi_dot] 
Ux   = 30;
a = 0.7 * veh.L;
b = 0.3 * veh.L;
K_la = 3500;
x_la = 25;
A    = [0, 1, 0, 0; ...
         -K_la/veh.m, -(tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m * Ux), ((tire_f.Ca_lin + tire_r.Ca_lin)/(veh.m)...
         - (K_la * x_la/veh.m)), (-a * tire_f.Ca_lin + b * tire_r.Ca_lin)/(veh.m*Ux) ; ...
         0, 0, 0, 1; ...
         -a * K_la/veh.Iz, (b * tire_r.Ca_lin - a * tire_f.Ca_lin)/(veh.Iz * Ux),...
         ((a * tire_f.Ca_lin - b * tire_r.Ca_lin)/(veh.Iz))-(K_la*a*x_la/veh.Iz),...
         -(a^2 * tire_f.Ca_lin + b^2 * tire_r.Ca_lin)/(veh.Iz*Ux)];


B = zeros(4,1);     C = eye(4);     D = B;
sys  = ss(A,B,C,D);
y    = lsim(sys,zeros(size(t)),t,x0)';  % u = 0, autonomous system
e    = y(1,:);
dPsi = y(3,:);
figure(7); 
subplot(2,1,1);
plot(t,e);
title('2.3 with xla = 25')
legend('Lateral Error')
xlabel('time [s]'); ylabel('error [m]');
hold on;
subplot(2,1,2);
plot(t,dPsi);
xlabel('time [s]'); ylabel('error [rad]');
%title('2.3 with xla = 25'); xlabel('time [s]'); ylabel('error [m]');
legend('Heading Error');
close(figure)
%% Problem 3 Code Here:
load('project_data.mat');

%IMPORTANT NOTE:
% Cleaned up code for project

% path information
init = [0;0;0];             % starting from the origin

% straight  path
%s = [0     150]; 
%k = [0     0];        % straight path
%k = [1/40 1/40];      % curved path

% Oval path information
s = path.s_m;               % distance along path
k = path.k_1pm;             % curvature along path

% integrate s and k to get path
path = integrate_path(s,k,init);        % Path integration 

% append speed profile to path
path.UxDes = 5.1*ones(size(path.s));    % Setting UxDesired to be 5.1 m/s for conservative run along clothoids

g = 9.81;                   	% gravity acceleration, meters/sec^2

% vehicle parameters
setup_niki;

% simulation time
t_final = 40;                   % 20s is the run for about 1 path
dT = 0.001; 
t_s = 0:dT:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);

% set initial conditions
e_m(1)      = 1;            % Initital condition for lateral error
ux_mps(1)   = 13;           % Initial condition for lateral speed

for modes = 1:2             % Running the plots codes for both modes

for idx = 1:N

    % look up K
    K = interp1(path.s, path.k, s_m(idx));
    
    % current states
    r = r_radps(idx);               
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);

    %Here Call the me227_controller function to calculate inputs to the
    %vehicle!
    
    %Note Mode will be used to select which controller is active for the
    %project, but for this project use Mode == 1 to select the LQR
    %controller and Mode == 2 to select the feedforward plus feedback
    %controller as given in Homework 4
    [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, modes, path);      
    % Mode = 1 [LQR]
    % Mode = 2 [Feedforward + Feedback [HW #4]]

    %Calculate the Dynamics with the Nonlinear Bike Model
    [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r  );
        
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler( r_radps(idx), r_dot, dT );
        ux_mps(idx+1) = integrate_euler( ux_mps(idx), ux_dot, dT );
        uy_mps(idx+1) = integrate_euler( uy_mps(idx), uy_dot, dT );
        s_m(idx+1) = integrate_euler( s_m(idx), s_dot, dT );
        e_m(idx+1) = integrate_euler( e_m(idx), e_dot, dT );
        dpsi_rad(idx+1) = integrate_euler( dpsi_rad(idx), dpsi_dot, dT );
    end
end

figure(11);
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
hold on;
if modes == 1                               % animate only for our LQR mode
    animate(path, veh, dpsi_rad, s_m, e_m, delta_rad);
end
end                                         % end loop for index
legend('Our LQR','HW #4 feedforward');

%% Problem 3.2

%Here are the paths for both the constant radius and 
%undulating paths
load
% path information
init = [0;0;0];

% constant radius  path
%s = [0      150];
%k = [1/40   1/40];
%k = [0 0];

% "undulating" path
%s_2 = [0      20      40   150];
%k_2 = [1/20    -1/20       0    0];

s_2 = [0 150];
k_2 = [1/40 1/40];

for index = 1:2

if (index ==1)
    path = integrate_path(s,k,init);
    %path_u = integrate_path(s,k,init);
else
    path = integrate_path(s_2,k_2,init);
    %path_u = integrate_path(s_2,k_2,init);
end
% integrate s and k to get path
%path_r = integrate_path(s(index),k(index),init);
    
% integrate s and k to get path
%path_u = integrate_path(s(index),k(index),init);

% append speed profile to path
path.UxDes = 5.1*ones(size(path.s));            % desired conservative path for velocity

g = 9.81;                   	% gravity acceleration, meters/sec^2

% vehicle parameters
setup_niki;

% simulation time
t_final = 8;
dT = 0.001; 
t_s = 0:dT:t_final;

% allocate space for simulation data
N = length(t_s);
r_radps     = zeros(N,1);
uy_mps      = zeros(N,1);
ux_mps      = zeros(N,1);
dpsi_rad    = zeros(N,1);
s_m         = zeros(N,1);
e_m         = zeros(N,1);
delta_rad   = zeros(N,1);


% set initial conditions
e_m(1)      = 1;
ux_mps(1)   = 15;

for idx = 1:N

    % look up K
    K = interp1(path.s, path.k, s_m(idx));
    
    % current states
    r = r_radps(idx);
    uy = uy_mps(idx);
    ux = ux_mps(idx);
    dpsi = dpsi_rad(idx);
    s = s_m(idx);
    e = e_m(idx);

    %Here Call the me227_controller function to calculate inputs to the
    %vehicle!
    
    %Note Mode will be used to select which controller is active for the
    %project, but for this homework use Mode == 1 to select the feedback
    %controller and Mode == 2 to select the feedforward plus feedback
    %controller
    [ delta, Fx ] = me227_controller( s, e, dpsi, ux, uy, r, 1, path);
    
    %Calculate the Dynamics with the Nonlinear Bike Model
    [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
            nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r  );
        
    % only update next state if we are not at end of simulation
    delta_rad(idx) = delta;
    if idx < N
        % Euler integration(Feel free to vectorize this)
        r_radps(idx+1) = integrate_euler( r_radps(idx), r_dot, dT );
        ux_mps(idx+1) = integrate_euler( ux_mps(idx), ux_dot, dT );
        uy_mps(idx+1) = integrate_euler( uy_mps(idx), uy_dot, dT );
        s_m(idx+1) = integrate_euler( s_m(idx), s_dot, dT );
        e_m(idx+1) = integrate_euler( e_m(idx), e_dot, dT );
        dpsi_rad(idx+1) = integrate_euler( dpsi_rad(idx), dpsi_dot, dT );
    end
end

%Here is the desired format for your plots for (2) and (3)
figure(8);
h_e = subplot(2,1,1);
plot(t_s, e_m);
hold on;
h_dpsi = subplot(2,1,2);
plot(t_s, dpsi_rad);
hold on;
end
grid(h_e, 'on')
title(h_e, 'Lateral Error')
xlabel(h_e, 'Time [s]')
ylabel(h_e, 'e [m]')
legend(h_e, 'Constant radius','Undulating Road')

grid(h_dpsi, 'on')
title(h_dpsi, 'Heading Error')
xlabel(h_dpsi, 'Time [s]')
ylabel(h_dpsi, '\Delta\psi [rad]')
legend(h_dpsi, 'Constant radius','Undulating Road')

animate(path, veh, dpsi_rad, s_m, e_m, delta_rad)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%STUDENT FUNCTIONS BELOW
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate Forces with the Fiala Nonlinear Tire Model
function Fy = fiala_model(alpha, tire)
%   Calculate tire forces with the fiala model

%%%%% STUDENT CODE HERE %%%%%
    alpha_sl = abs(atan(3*tire.mu*tire.Fz/tire.Ca));
    if (abs(alpha)<alpha_sl)
        Fy = -tire.Ca*tan(alpha) + (((tire.Ca^2)/(3*tire.mu*tire.Fz))*(2 - (tire.mu_s/tire.mu))*tan(alpha)*...
        abs(tan(alpha))) - (((tire.Ca^3)/(9*tire.mu^2*tire.Fz^2))*(tan(alpha))^3*...
        (1-(2*tire.mu_s/(3*tire.mu)))); 
    else
        Fy = -tire.mu*tire.Fz*sign(alpha);
    end

%%%%% END STUDENT CODE %%%%%
end

%Calculate the Nonlinear Bicycle Model Dynamics
function [ r_dot, uy_dot, ux_dot, s_dot, e_dot, dpsi_dot] = ...
    nonlinear_bicycle_model( r, uy, ux, dpsi, e, delta, Fx, K, veh, tire_f, tire_r )
% slip angles
[alphaf,alphar] = slip_angles(r, uy, ux, delta, veh);
%KINEMATIC_MODEL
%   Calculate state derivatives for the kinematic vehicle model
a = veh.a;
b = veh.b;
Caf = tire_f.Ca;
Car = tire_r.Ca;
%%%%% STUDENT CODE HERE %%%%%



% lateral tire forces
fyf = fiala_model (alphaf, tire_f);
fyr = fiala_model (alphar,tire_r);

%Split longitudinal force based on drive and brakedistribution
if Fx > 0
    fxf = Fx;
    fxr = 0;
else
    fxf = Fx/2;
    fxr = Fx/2;
end

% dynamics
%r_dot = (((b * Car - a * Caf)*uy/ux) - ((a^2 * Caf + b^2 * Car) * r/ux) + (a * Caf * delta))/veh.Iz;
r_dot = ((a * fyf * cos(delta)) + (a * fxf * sin(delta)) - (b * fyr))/veh.Iz;
ux_dot = (fxr - fyf*sin(delta))/veh.m + r*uy;
uy_dot = (fyf * cos(delta) + fyr)/veh.m - r * ux;
s_dot = (1/(1-e*K))*(ux * cos(dpsi) - uy * sin(dpsi));
%e_dot = ux * dpsi + uy;
e_dot = uy * cos(dpsi) + ux * sin(dpsi);
%dpsi_dot = r;
dpsi_dot = r - (K*s_dot);

%%%%% END STUDENT CODE %%%%%
end

%Calculate the Slip Angles Here:
function [alphaF, alphaR] = slip_angles( r, uy, ux, delta, veh)
%slip_angles
%   calculate the tire slip angles 

%%%%% STUDENT CODE HERE %%%%%
alphaF = atan2((uy + veh.a*r),ux) - delta ;
alphaR = atan2(uy - veh.b*r,ux);
%%%%% END STUDENT CODE %%%%%
end

%Use standard Euler Integration
function x1 = integrate_euler( x0, x0_dot, dt )
%INTEGRATE_EULER
%   simple zero-hold integration scheme to compute discrete next state

%%%%% STUDENT CODE HERE %%%%%

x1 = x0 + (x0_dot*dt);
%%%%% END STUDENT CODE %%%%%
end
