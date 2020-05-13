function [ delta, Fx ] = me227_controller(s, e, dpsi, Ux, Uy, r, gains, ...
                                          control_mode, path)
%ME227 Controller:
% Spring 2019
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda
% 
% Here you should use the inputs from above to calculate the inputs to the
% vehicle. These will be delta and Fx and will be determined based upon
% your control laws below. 
%
% For the project you will use this same input output structure and in this
% homework you will use this control structure for defining the control
% inputs in your simulation.

%--------------------------------------------------------------------------
%% Constants
%--------------------------------------------------------------------------
g = 9.81;                       % [m/s^2]  gravity

%--------------------------------------------------------------------------
%% Vehicle Parameters
%--------------------------------------------------------------------------
veh.m  = 1926.2;                % [kg]     mass
veh.Iz = 2763.49;               % [kg-m^2] rotational inertia
veh.a  = 1.264;                 % [m]      distance from CoM to front axle
veh.b  = 1.367;                 % [m]      distance from C0M to rear axle
veh.L  = veh.a + veh.b;         % [m]      wheelbase
veh.Wf = veh.m*g*(veh.b/veh.L); % [N]      static front axle weight
veh.Wr = veh.m*g*(veh.a/veh.L); % [N]      static rear axle weight

%--------------------------------------------------------------------------
%% Tire Parameters
%--------------------------------------------------------------------------
% Front tires
f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
f_tire.Cy     = 110000;         % [N/rad]  fiala model cornering stiffness
f_tire.mu_s   = 0.90;           %          sliding friction coefficient
f_tire.mu     = 0.90;           %          peak friction coefficient

% Rear tires
r_tire.Ca_lin = 120000;
r_tire.Cy     = 180000;
r_tire.mu_s   = 0.94;
r_tire.mu     = 0.94;

%--------------------------------------------------------------------------
%% Understeer Gradient
%--------------------------------------------------------------------------
% Calculate the understeer gradient for Niki
K_radpmps2 = (veh.Wf / f_tire.Ca_lin - veh.Wr / r_tire.Ca_lin) / g;

%--------------------------------------------------------------------------
%% Control Parameters
%--------------------------------------------------------------------------
% !! Do not change the code in this block - change the desired gains in
% your simulator script!!
K_la = gains.K_la;
x_la = gains.x_la;
K_long = gains.K_long;

%--------------------------------------------------------------------------
%% Find Path Dependent Parameters
%--------------------------------------------------------------------------
% !! Do not change the code in this block - change the desired speed in
% your simulator script!!
Ux_des = interp1(path.s, path.UxDes, s);

%Find Curvature for the current distance along the path via interpolation
kappa = interp1(path.s, path.k, s);

%--------------------------------------------------------------------------
%% Lateral Control Law
%--------------------------------------------------------------------------
if control_mode == 1  % lookahead controller
    dpsi_ss  = kappa*(veh.m*veh.a*Ux^2 / (veh.L * r_tire.Ca_lin) - veh.b);
    delta_ff = K_la * x_la * dpsi_ss / f_tire.Ca_lin + kappa * (veh.L + ...
               K_radpmps2 * Ux^2);

    delta = -K_la * (e + x_la*dpsi)/f_tire.Ca_lin + delta_ff;
    
else  % custom controller

end

%--------------------------------------------------------------------------
%% Longitudinal Control Law
%--------------------------------------------------------------------------
if control_mode == 1  % lookahead controller
    Fx = K_long*(Ux_des - Ux);
    
else  % custom controller
    
end

end
