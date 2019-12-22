function [ delta, Fx,  Fx1 ,Fx2, K1 ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
%ME227 Controller:
% Spring 2019
% Team AutoBug
% Adit Desai, Ashar Alam and Kshitij Kumbar
% Desc : Controller design to be loaded on the vehicle compute platform. Contains code for
%		 both Lookahead and LQR lateral controllers as well as Longitudinal Controller

g = 9.81; 			                    % [m/s^2]  gravity
Caf = 80000;							% [N/rad]  linear model cornering stiffness
Car = 120000;     					    % [N/rad]  fiala model cornering stiffness
m = 1.926e+03;  		                % [kg]     mass
Iz = 2.763e+03;			                % [kg-m^2] rotational inertia
a = 1.2629;   			                % [m]      distance from CoM to front axle
b = 1.3681;    			                % [m]      distance from CoM to rear axle
L = 2.6310;       					    % [m]      wheelbase
Wf = 0.52;  							% [kg]     front axle weight
Wr = 0.48; 							    % [kg]     rear axle weight
Wf = Wf * m*g;
Wr = Wr * m*g;
K_grad = ((Wf/Caf) - (Wr/Car))*(1/g);   % Understeer Gradient

Kla = 8000;                             % Lookahead Gain
xla = 15;                               % Lookahead Distance

if Mode == 1
    Kdrive = 1350;                      % for LQR
elseif Mode == 2
    Kdrive = 5000;                      % for Lookahead control
else
    Kdrive = 5000;                      % for PD control (NOTE: BE VERY CAREFUL WITH GAINS. POSSIBLE PHYSICAL REPERCUSSIONS) 
end

% Uxdesired and Adesired for the current distance along the path via interpolation
ux_desired = interp1(path.s_m, path.UxDes, s);
ades_desired = interp1(path.s_m, path.axDes, s);

% Curvature for the current distance along the path via interpolation
K = interp1(path.s_m, path.k_1pm, s);
s_dot = (1/(1 - e*K))*(ux*cos(dpsi) - uy*sin(dpsi));
dot_e = uy*cos(dpsi) + ux*sin(dpsi);
dot_dpsi = r - K*s_dot;
Ux = ux;
K1 = [0.6071 0.066 1.134 0.05664];      %Values from mean LQR
K_lqr = K1;

if Mode == 1
    % LQR control with feedforward terms
    dff =  K*(((m*a*ux^2)/(L*Car)) - b) + K*(L + K_grad*ux^2)
    delta = - K1*[e;dot_e;dpsi;dot_dpsi] + dff;
    
elseif Mode == 2
    %steering command with both feedback and feedforward control
    dpsi_SS = K*(((m*a*ux^2)/(L*Car)) - b);
    delta_ff = ((Kla*xla*dpsi_SS)/Caf) + K*(L + K_grad*ux^2);
    delta = - (Kla*(e + xla*dpsi))/Caf + delta_ff;
else
    %Basic PD control with gains closer to LQR derived values
    K2 = [0.60 0.0966 1.5 0.045664]
    delta = - K2*[e;dot_e;dpsi;dot_dpsi];
end

%  Longitudinal Control Law to Calcuate Fx
Frr = 0.015*m*g;
Fd = 0.5*1.225*0.594*ux^2;
Fx = m*ades_desired + Frr + Fd + Kdrive*(ux_desired - ux);
Fx1 = m*ades_desired + Frr + Fd;
Fx2 = Kdrive*(ux_desired - ux);
end
