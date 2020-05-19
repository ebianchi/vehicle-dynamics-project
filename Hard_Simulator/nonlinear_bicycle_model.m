function [Ux_dot, Uy_dot, r_dot, s_dot, e_dot, dpsi_dot] =...
    nonlinear_bicycle_model(F_yf, F_yr, F_xf, F_xr, Ux, Uy, r, e, dpsi, delta, kappa, veh)
%LATERAL_BICYCLE_MODEL
%   Calculates the vehicle state derivatives using the bicycle model
%
%   Inputs
%       F_yf:       Front lateral tire force [N]
%       F_yr:       Rear lateral tire force [N]
%       F_xf:       Front longitudinal force [N]
%       F_xr:       Rear longitudinal force [N]
%       Ux:         Longitudinal velocity [m/s]
%       Uy:         Lateral velocity [m/s]
%       r:          Yaw rate [rad/s]
%       e:          Error from path [m]
%       dpsi:       heading error [rad]
%       delta:      Steer angle [rad]
%       kappa:      Road curvature at current position [1/m]
%       veh:        Vehicle parameters struct
%
%   Output:
%       Ux_dot:     Derivative of longitudinal velocity [m/s^2]
%       Uy_dot:     Derivative of lateral velocity [m/s^2]
%       r_dot:      Derivative of yaw rate [rad/s^2]
%       s_dot:      Derivative of distance along path [m/s]
%       e_dot:      Derivative of error from path [m/s]
%       dpsi_dot:   Derivative of heading error [rad/s]

%--------------------------------------------------------------------------
%% CALCULATE VELOCITY STATE DERIVATIVES
%--------------------------------------------------------------------------
Ux_dot = (1/veh.m)*(F_xr + F_xf*cos(delta) - F_yf*sin(delta) -...
    veh.frr*veh.m*veh.g - 0.5*veh.rho*veh.cdA*Ux^2) + r*Uy; % [m/s^2]
Uy_dot = (1/veh.m)*(F_yf*cos(delta) + F_yr + F_xf*sin(delta)) - r*Ux; % [m/s^2]
r_dot = (1/veh.Iz)*(veh.a*F_yf*cos(delta) + veh.a*F_xf*sin(delta) - veh.b*F_yr); % [rad/s^2]

%--------------------------------------------------------------------------
%% CALCULATE POSITION STATE DERIVATIVES
%--------------------------------------------------------------------------
s_dot = (1/(1 - e*kappa))*(Ux*cos(dpsi) - Uy*sin(dpsi)); % [m/s]
e_dot = Uy*cos(dpsi) + Ux*sin(dpsi); % [m/s]
dpsi_dot = r - kappa*s_dot; % [rad/s]

end
