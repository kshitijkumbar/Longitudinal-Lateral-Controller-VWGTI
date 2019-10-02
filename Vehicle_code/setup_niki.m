%% Set Vehicle Parameters
% declare parameters for a VW GTI with 4 passengers
% to the MATLAB workspace

% Authors: John Alsterda & Nathan Speilberg for Stanford ME227

g = 9.81;                       % [m/s^2]  gravity

%% vehicle parameters
veh.m  = 1926.2;                % [kg]     mass
veh.Iz = 2763.49;               % [kg-m^2] rotational inertia
veh.a  = 1.264;                 % [m]      distance from CoM to front axle
veh.b  = 1.367;                 % [m]      distance from C0M to rear axle
veh.L  = veh.a + veh.b;         % [m]      wheelbase
veh.Wf = veh.m*(veh.b/veh.L);   % [kg]     front axle weight
veh.Wr = veh.m*(veh.a/veh.L);   % [kg]     rear axle weight
veh.rW = 0.318;                 % [m]      tire radius

%% Tire Parameters
% front tires
tire_f.Ca_lin =  80000;         % [N/rad]  linear model cornering stiffness
tire_f.Ca     = 110000;         % [N/rad]  fiala model cornering stiffness
tire_f.mu_s   = 0.90;           %          sliding friction coefficient
tire_f.mu     = 0.90;           %          peak friction coefficient
tire_f.Fz     = veh.Wf*g;       % [N]      static normal load on the axle
% rear tires
tire_r.Ca_lin = 120000;
tire_r.Ca     = 180000;
tire_r.mu_s   = 0.94;
tire_r.mu     = 0.94;
tire_r.Fz     = veh.Wr*g;
