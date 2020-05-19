function [Ux_dot, Uy_dot, r_dot, s_dot, e_dot, dpsi_dot] =...
    nonlinear_bicycle_model(F_yf, F_yr, Fx, Ux, Uy, r, e, dpsi, delta, kappa, veh)
%LATERAL_BICYCLE_MODEL
%   Calculates the vehicle state derivatives using the bicycle model
%
%   Inputs
%       F_yf:       Front lateral tire force [N]
%       F_yr:       Rear lateral tire force [N]
%       F_x:        Total longitudinal force [N]
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
%% CALCULATE LONGITUDINAL FORCE AT EACH AXLE
%--------------------------------------------------------------------------
if (Fx < 0)
    % braking, so applied at front and rear
    F_xf = 0.5 * Fx;
    F_xr = F_xf;
    
else
    % accelerating, so applied only at front
    F_xf = Fx;
    F_xr = 0;
    
end

%--------------------------------------------------------------------------
%% CALCULATE VELOCITY STATE DERIVATIVES
%--------------------------------------------------------------------------
gradeTheta = 0; % degrees, set to 0 if no grade term
Ux_dot = (F_xr + F_xf*cos(delta) - F_yf*sin(delta))/veh.m + r*Uy;
Ux_dot = Ux_dot + ( 9.81 * sind(gradeTheta) );
Uy_dot = (F_yf*cos(delta) + F_xf*sin(delta) + F_yr)/veh.m - r*Ux;
r_dot  = (veh.a*F_yf*cos(delta) + veh.a*F_xf*sin(delta) - veh.b*F_yr)/veh.Iz;

%--------------------------------------------------------------------------
%% CALCULATE TRACKING STATE DERIVATIVES
%--------------------------------------------------------------------------
s_dot    = 1/(1-e*kappa) * (Ux*cos(dpsi) - Uy*sin(dpsi));
e_dot    = Uy*cos(dpsi) + Ux*sin(dpsi);
dpsi_dot = r - kappa*s_dot;

end
