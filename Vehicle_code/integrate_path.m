function [path] = integrate_path(s_in, k_in, z_0)
%INTEGRATE_PATH Generate a path.
%   PATH = INTEGRATE_PATH(S_IN, K_IN, Z_0)
%   generates a path by integrating path curvature K_IN along path distance
%   S_IN starting with initial heading, East, and North (z_0)
%   heading (psi) is measured CCW from North

% Authors: Matt Brown, Vincent Laurense, John Subosits
% Adapted by: Nathan Spielberg, John Alsterda for Stanford ME227

options    = odeset('RelTol', 1e-5, 'MaxStep',1);
[s_out, z] = ode45(@path_derivs, [s_in(1) s_in(end)], z_0, options);

path.s    = s_out;
path.k    = interp1(s_in, k_in, s_out, 'linear', 'extrap');
path.psi  = z(:,1);
path.posE = z(:,2);
path.posN = z(:,3);

    function dZ = path_derivs(s, Z)
        % Z  = [psi, E, N], for psi CCW from N
        K  = interp1(s_in, k_in, s);
        dZ = [K; -sin(Z(1)); cos(Z(1))];
    end
end
