function [ delta, Fx,  Fx1 ,Fx2, K1 ] = me227_controller( s, e, dpsi, ux, uy, r, Mode, path)
%ME227 Controller:
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
%
% Here you should use the inputs from above to calculate the inputs to the
% vehicle. These will be delta and Fx and will be determined based upon
% your control laws below.
%
% For the project you wil use this same input output structure and in this
% homework you will use this control structure for defining the control
% inputs in your simulation.

% Define your vehicle parameters here (you must define all of your
% parameters here, you may not reference and outside function in order to
% function on the GTI)
g = 9.81;
Caf = 80000;
Car = 120000;
m = 1.926e+03;
Iz = 2.763e+03;
a = 1.2629;
b = 1.3681;
L = 2.6310;
Wf = 0.52;
Wr = 0.48;
Wf = Wf * m*g;
Wr = Wr * m*g;
K_grad = ((Wf/Caf) - (Wr/Car))*(1/g); %Understeer Gradient

Kla = 5000;
xla = 15;

if Mode == 1
    Kdrive = 1350;     % for LQR
else
    Kdrive = 2000;     % for 
end  
    %Uxdesired and adesired for the current distance along the path via interpolation
    ux_desired = interp1(path.s_m, path.UxDes, s);
    ades_desired = interp1(path.s_m, path.axDes, s);
    % Find Curvature for the current distance along the path via interpolation
    K = interp1(path.s_m, path.k_1pm, s);
    s_dot = (1/(1 - e*K))*(ux*cos(dpsi) - uy*sin(dpsi));
    dot_e = uy*cos(dpsi) + ux*sin(dpsi);
    dot_dpsi = r - K*s_dot;
    Ux = ux;
    A1 = [0 1 0 0;
        0 -((Caf + Car)/(m*Ux)) (Caf + Car)/m ((-a*Caf + b*Car)/(m*Ux));
        0 0 0 1;
        0 ((b*Car -a*Caf)/(Iz*Ux)) ((a*Caf - b*Car)/Iz) (-(a^2*Caf + b^2*Car)/(Iz*Ux))];
    %     Q = [3.5 0 0 0; 0 0.1 0 0;0 0 0 0;0 0 0 0]; %useful values
    %     R = 5.5*eye(1);
    
    
    Q = [0.9 0 0 0; 0 0.1 0 0;0 0 0 0;0 0 0 0];
    R = 3.5*eye(1);
    B1 = [0 Caf/m 0 (a*Caf)/m]';     C1 = eye(4);     D1 =zeros(4,1);
    sys1  = ss(A1,B1,C1,D1);
    %     [K1,S1,ea] = lqr(sys1,Q,R);
    
    % K1 = [1, 0.8, 2.12, 0.1]; % Mean LQR
    % K1= [0.5071, 0.205,1.43,0.1]
    % K1= [0.5071, 0.405,1.43,0.1];
    % K1 = [0.6 0.2688 1.551 0.078];
    K1 = [0.5071 0.066 1.134 0.05664]; %Values from mean LQR
    K_lqr = K1;
    if Mode == 1
        % Calculate the feedback steering command with lateral error and heading error
        dff =  K*(((m*a*ux^2)/(L*Car)) - b) + K*(L + K_grad*ux^2)
        delta = - K1*[e;dot_e;dpsi;dot_dpsi] + dff;
        
    else
        % Calculate the steering command with both feedback and feedforward control
        dpsi_SS = K*(((m*a*ux^2)/(L*Car)) - b);
        delta_ff = ((Kla*xla*dpsi_SS)/Caf) + K*(L + K_grad*ux^2);
        delta = - (Kla*(e + xla*dpsi))/Caf + delta_ff;
    end
    % Use the Longitudinal Control Law to Calcuate Fx
    %     Fx = Kdrive*(ux_desired - ux);
    Frr = 0.015*m*g;
    Fd = 0.5*1.225*0.594*ux^2;
    Fx = m*ades_desired + Frr + Fd + Kdrive*(ux_desired - ux);
    Fx1 = m*ades_desired + Frr + Fd;
    Fx2 = Kdrive*(ux_desired - ux);
end
