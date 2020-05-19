function [Ux_1, Uy_1, r_1, s_1, e_1, dpsi_1] =...
    simulate_hold(delta_0, Fx_0, kappa, dt, veh, f_tire, r_tire, mode)
%SIMULATE_STEP
%   Hold same position while passing through states to check for windup

%   Inputs
%       delta_0:    Current steer angle [rad]
%       Fx_0:       Current longitudinal force [N]
%       kappa:      Road curvature at current position [1/m]
%       dt:         Length of time step [s]
%       veh:        Vehicle parameters struct
%       f_tire:     Front tire parameters struct
%       r_tire:     Rear tire parameters struct
%       mode:       Simulation mode [1 or 2]
%                   Mode 1 - Nonlinear bicycle model
%                   Mode 2 - Nonlinear bicycle model with additional
%                   non-ideal terms
%
%   Output:
%       Ux_1:       Updated longitudinal velocity [m/s]
%       Uy_1:       Updated lateral velocity [m/s]
%       r_1:        Updated yaw rate [rad/s]
%       s_1:        Updated distance along path [m]
%       e_1:        Updated error from path [m]
%       dpsi_1:     Update heading error [rad]

%--------------------------------------------------------------------------
%% PERSISTENT VARIABLES TO HOLD THE TRUE STATE
%--------------------------------------------------------------------------
persistent Ux_ground_truth;
persistent Uy_ground_truth;
persistent r_ground_truth;
persistent dpsi_ground_truth;
persistent s_ground_truth;
persistent e_ground_truth;

persistent brake_command_;
persistent brake_actual_;
persistent engine_command_;
persistent engine_actual_;
persistent delta_command_;
persistent delta_actual_;

% initialize ground truth variables at the first function call
if isempty(Ux_ground_truth)
    
    Ux_ground_truth = 0;
    Uy_ground_truth = 0;
    r_ground_truth = 0;
    dpsi_ground_truth = deg2rad(2);
    s_ground_truth = 0;
    e_ground_truth = 0.15;
    
    % These vectors will hold the last 0.5 seconds of actual postitions and
    % commands. We will shift right and store the newest value in the
    % leftmost element each time through the simulation
    brake_command_ = zeros(1,0.5/dt);
    engine_command_ = zeros(1,0.5/dt);
    delta_command_ = zeros(1,0.5/dt);
    
    brake_actual_ = zeros(1,0.5/dt);
    engine_actual_ = zeros(1,0.5/dt);
    delta_actual_ = zeros(1,0.5/dt);
end

%--------------------------------------------------------------------------
%% SHIFT ACTUAL HISTORY
%--------------------------------------------------------------------------
brake_command_ = circshift(brake_command_, 1);
engine_command_ = circshift(engine_command_, 1);
delta_command_ = circshift(delta_command_, 1);

brake_actual_ = circshift(brake_actual_, 1);
engine_actual_ = circshift(engine_actual_, 1);
delta_actual_ = circshift(delta_actual_, 1);

%--------------------------------------------------------------------------
%% SPLIT LONGITUDINAL FORCES
%--------------------------------------------------------------------------
[Fx_lim] = Fx_limits(Fx_0, veh, f_tire, r_tire);

if Fx_lim > 0
    brake_command_(1) = 0;
    engine_command_(1) = Fx_lim;
else
    brake_command_(1) = Fx_lim;
    engine_command_(1) = 0;
end

delta_command_(1) = delta_0;

maxF_f = f_tire.mu*veh.Wf;
maxF_r = r_tire.mu*veh.Wr;
maxF_xf = veh.maxPower_W/Ux_ground_truth;

%--------------------------------------------------------------------------
%% IMPOSE ACTUATOR DYNAMICS & LIMITS
%--------------------------------------------------------------------------
brake_actual_(1) = brakeDynamics(brake_command_, brake_actual_, veh, dt);
engine_actual_(1) = engineDynamics(engine_command_, engine_actual_, Ux_ground_truth, veh, dt);
delta_actual_(1) = steeringDynamics(delta_command_, delta_actual_, veh, dt);

% Calculate forces
Fx_f = engine_actual_(1)*veh.driveDistro(1) + brake_actual_(1)*veh.brakeDistro(1);
Fx_r = engine_actual_(1)*veh.driveDistro(2) + brake_actual_(1)*veh.brakeDistro(2);
delta = delta_actual_(1);

%--------------------------------------------------------------------------
%% IF MODE 2, RETURN CORRUPTED STATE ESTIMATES
%--------------------------------------------------------------------------
Ux_1 = Ux_ground_truth;
Uy_1 = Uy_ground_truth;
r_1 = r_ground_truth;
e_1 = e_ground_truth;
s_1 = s_ground_truth;
dpsi_1 = dpsi_ground_truth;

if (mode == 2)
    [e_noise, s_noise, dpsi_noise] = gps_noise();
    e_1 = e_1 + e_noise;
    s_1 = s_1 + s_noise;
    dpsi_1 = dpsi_1 + dpsi_noise;
end

end
