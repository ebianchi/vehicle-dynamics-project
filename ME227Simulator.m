%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Nonlinear Vehicle Simulation

clear all; clc; close all

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
gains.K_la = 7000;              % [N/m]
gains.x_la = 12;                % [m]
gains.K_long = veh.m*.1*g;     % [N/m]

% Set control gains -- PID controller
gains.K_lat_p_PID = 0.1;           % [] TODO:  Fill in units here and tune
gains.K_lat_i_PID = 0.001;    % []        more thoughtfully
gains.K_lat_d_PID = 7;         % []

% Select controller
control_mode = 1;  % 1 - lookahead controller
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
Uxdes_long = zeros(1,lenT);
Uy_ = zeros(1,lenT);
delta_ = zeros(1,lenT);
Fx_ = zeros(1,lenT);
Uxerror_ = zeros(1,lenT);
%--------------------------------------------------------------------------
%% SET INITIAL CONDITIONS
%--------------------------------------------------------------------------
s_(1) = 0;
e_(1) = 0;
dpsi_(1) = 0;
r_(1) = 0;
Ux_(1) = 0;
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
    Uxdes_long(i+1) = Ux_des;
    Uxerror_(i) = Ux_(i) - Ux_des;
    
    % Calculate actuator commands  
    [delta_(i), Fx_(i)] = me227_controller(s_(i), e_(i), dpsi_(i), ...
                                           Ux_(i), Uy_(i), r_(i), gains,...
                                           control_mode, path);

    % Take simulation step
    [Ux_(i+1), Uy_(i+1), r_(i+1), s_(i+1), e_(i+1), dpsi_(i+1)] = ...
    simulate_step(Ux_(i), Uy_(i), r_(i), s_(i), e_(i), dpsi_(i), ...
                  delta_(i), Fx_(i), kappa, dt, veh, f_tire, r_tire);
    
              
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

figure
subplot(2,1,1);
plot(t_, Uxerror_);
grid on
xlabel('Time [s]')
ylabel('Ux error [m/s]')
title('Longitudinal Error, Longitudinal Speed vs Time')

subplot(2,1,2);
plot(t_, Ux_)
hold on;
plot(t_, Uxdes_long)
legend('Ux actual', 'Ux desired');
grid on
xlabel('Time [s]')
ylabel('Ux [m/s]')
%--------------------------------------------------------------------------
%% ANIMATE VEHICLE
%--------------------------------------------------------------------------
%animate(path, veh, dpsi_, s_, e_, delta_)
