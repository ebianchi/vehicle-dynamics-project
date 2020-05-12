%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Nonlinear Vehicle Simulation

clear; clc; close all

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Create time vector (!!Do not change these variable names - store
% your results in these variables!!)
dt = 0.001;
t_ = 0:dt:30;
lenT = length(t_);

% Set control gains -- lookahead controller
gains.K_la = 3500;              % [N/m]
gains.x_la = 15;                % [m]
gains.K_long = veh.m*0.1*g;     % [N/m]

% Set control gains -- PID controller
gains.K_long_p_PID = 5000;      % [] TODO:  fill in these units,
gains.K_long_i_PID = 3;         % []        and tune better
gains.K_long_d_PID = 1;         % []
gains.K_lat_p_PID = 0.08;       % []
gains.K_lat_i_PID = 0.00003;    % []
gains.K_lat_d_PID = 60;         % []

% Calculate understeer gradient (bibit added)
veh.K = (veh.Wf / f_tire.Ca_lin - veh.Wr / r_tire.Ca_lin) / g;

% Select controller
control_mode = 2;  % 1 - lookahead controller
                   % 2 - PID controller

% Load path and speed profile
load('path.mat')

%--------------------------------------------------------------------------
%% GENERATE PATH AND ALLOCATE MEMORY
%--------------------------------------------------------------------------
% Allocate space for results (!!Do not change these variable names - store
% your results in these variables!!)
s_ = zeros(1,lenT);
e_ = zeros(1,lenT);
dpsi_ = zeros(1,lenT);
r_ = zeros(1,lenT);
Ux_ = zeros(1,lenT);
Uy_ = zeros(1,lenT);
delta_ = zeros(1,lenT);
Fx_ = zeros(1,lenT);

%--------------------------------------------------------------------------
%% SET INITIAL CONDITIONS
%--------------------------------------------------------------------------
s_(1) = 0;
e_(1) = 1;
dpsi_(1) = 0;
r_(1) = 0;
Ux_(1) = 15;
Uy_(1) = 0;

%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through every time step
for i = 1:lenT-1

    % look up kappa
    kappa = interp1(path.s, path.k, s_(i));
    
    % look up Ux_des
    Ux_des = interp1(path.s, path.UxDes, s_(i));
    
    % Calculate actuator commands  
    if control_mode == 1  % lookahead controller
        [delta_(i), Fx_(i)] = look_ahead_controller(Ux_(i), e_(i), dpsi_(i), ...
                                                kappa, gains, Ux_des, veh, ...
                                                r_tire, f_tire);
                                            
    elseif control_mode == 2  % PID controller
        if i == 1  % take care of initial integral and derivative terms
            int_Ux = 0;
            int_e  = 0;
            dUx    = 0;
            de     = 0;
        else
            dUx = Ux_(i) - Ux_(i-1);  % no need to also calculate integral 
            de  = e_(i)  - e_(i-1);   % terms, as those are added with each
                                      % loop
        end
        
        [delta_(i), Fx_(i), int_Ux, int_e] = PID_controller(Ux_(i), dUx,...
                                               int_Ux, e_(i), de, int_e,...
                                               gains, Ux_des);
    end
    
    % Take simulation step
    [Ux_(i+1), Uy_(i+1), r_(i+1), s_(i+1), e_(i+1), dpsi_(i+1)] = ...
    simulate_step(Ux_(i), Uy_(i), r_(i), s_(i), e_(i), dpsi_(i), ...
                  delta_(i), Fx_(i), kappa, dt, veh, f_tire, r_tire);
    
    %%% END STUDENT CODE
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
% Plot all of the vehicle states
figure

% Plot Ux
subplot(2,3,1)
plot(t_, Ux_)
grid on
xlabel('Time [s]')
ylabel('Ux [m/s]')

% Plot Uy
subplot(2,3,2)
plot(t_, Uy_)
grid on
title('Vehicle States')
xlabel('Time [s]')
ylabel('Uy [m/s]')

% Plot r
subplot(2,3,3)
plot(t_, rad2deg(r_))
grid on
xlabel('Time [s]')
ylabel('r [deg/s]')

% Plot s
subplot(2,3,4)
plot(t_, s_)
grid on
xlabel('Time [s]')
ylabel('s [m]')

% Plot e
subplot(2,3,5)
plot(t_, e_)
grid on
xlabel('Time [s]')
ylabel('e [m]')

% Plot dpsi
subplot(2,3,6)
plot(t_, rad2deg(dpsi_))
grid on
xlabel('Time [s]')
ylabel('dpsi [deg]')


% Plot the actuator commands
figure

subplot(2,1,1)
plot(t_, rad2deg(delta_))
grid on
xlabel('Time [s]')
ylabel('Steer Angle [deg]')
title('Actuator Commands')

subplot(2,1,2)
plot(t_, Fx_)
grid on
xlabel('Time [s]')
ylabel('Total Longitudinal Force [Fx]')

%--------------------------------------------------------------------------
%% ANIMATE VEHICLE
%--------------------------------------------------------------------------
% animate(path, veh, dpsi_, s_, e_, delta_)
