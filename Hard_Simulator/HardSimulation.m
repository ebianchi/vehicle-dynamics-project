%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2020
% Nonlinear Vehicle Hard Simulation - Project

clear; clc; close all;

% Clear persistent variables
clear simulate_step; clear gps_model; 
clear brakeDynamics; clear engineDynamics; clear steeringDynamics;

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki

% Create time vector (!!Do not change these variable names - store
% your results in these variables!!)
dt = 0.001;
t_ = 0:dt:30;
lenT = length(t_);

% Set gains
gains.K_la = 3500; % [N/m]
gains.x_la = 15; % [m]
gains.K_long = 0.1*veh.m*veh.g; % [N/m/s]

% Select lookahead controller mode
% control_mode = 1; % Feedback only
control_mode = 2; % Feedforward + Feedback

% Load path and speed profile
load('path.mat')

% Set simulation mode
% sim_mode = 0;   % Nonlinear model from homework
% sim_mode = 1;   % Actuator dynamics only
% sim_mode = 2;   % Actuator dynamics and noise on measurements
sim_mode = 3;   % Actuator dynamics, noisy measurements, and hold period


%--------------------------------------------------------------------------
%% ALLOCATE MEMORY
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
Fy_f = zeros(1,lenT);
Fy_r = zeros(1,lenT);
Fx_f = zeros(1,lenT);
Fx_r = zeros(1,lenT);
Fyf_max = zeros(1,lenT);
Fyr_max = zeros(1,lenT);
ax_ = zeros(1,lenT);
ay_ = zeros(1,lenT);
atot_ = zeros(1,lenT);
delta_actual_ = zeros(1,lenT);

%--------------------------------------------------------------------------
%% SET INITIAL CONDITIONS
%--------------------------------------------------------------------------
s_(1) = 0;
e_(1) = 0.15;
dpsi_(1) = 0;
r_(1) = 0;
Ux_(1) = 1;
Uy_(1) = 0;


%--------------------------------------------------------------------------
%% SIMULATION LOOP
%--------------------------------------------------------------------------
% Loop through every time step
for i = 1:lenT-1
    % look up kappa
    kappa = interp1(path.s, path.k, s_(i));

    % Calculate actuator commands
    [delta_(i), Fx_(i)] = me227_controller(s_(i), e_(i), dpsi_(i), Ux_(i),...
        Uy_(i), r_(i), gains, control_mode, path);
    
    if (t_(i) < 5) && (sim_mode == 3)
        % Hold here
        [Ux_(i+1), Uy_(i+1), r_(i+1), s_(i+1), e_(i+1), dpsi_(i+1)] =...
        simulate_hold(delta_(i), Fx_(i), kappa, dt, veh, f_tire, r_tire, 1);
    else    
        % Take simulation step
        [Ux_(i+1), Uy_(i+1), r_(i+1), s_(i+1), e_(i+1), dpsi_(i+1),...
            delta_actual_(i), Fx_f(i), Fx_r(i), Fy_f(i), Fy_r(i),...
            Fyf_max(i), Fyr_max(i), ax_(i), ay_(i), atot_(i)] =...
            simulate_step(delta_(i), Fx_(i), kappa, dt, veh, f_tire, r_tire, sim_mode);
    end
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
hold on
plot(t_, rad2deg(delta_actual_))
grid on
xlabel('Time [s]')
ylabel('Steer Angle [deg]')
legend('Commanded','Actual')
title('Actuator Commands')

subplot(2,1,2)
plot(t_, Fx_)
hold on
plot(t_, Fx_r + Fx_f)
grid on
xlabel('Time [s]')
legend('Commanded','Actual')
ylabel('Total Longitudinal Force [Fx]')

% Plot tire forces
figure

subplot(2,1,1)
plot(t_, Fy_f)
hold on
plot(t_, Fyf_max, 'k--')
plot(t_, -Fyf_max, 'k--')
grid on
xlabel('Time [s]')
ylabel('Front Lateral Force [N]')
title('Front Lateral Force [$F_{yf}$]')

subplot(2,1,2)
plot(t_, Fy_r)
hold on
plot(t_, Fyr_max, 'k--')
plot(t_, -Fyr_max, 'k--')
grid on
xlabel('Time [s]')
ylabel('Rear Lateral Force [N]')
ylabel('Rear Lateral Force [$F_{yr}$]')

% Plot Accelerations
figure

plot(s_, ax_)
hold on
plot(s_, ay_)
plot(s_, atot_)
grid on
xlabel('Distance Along Path [m]')
ylabel('Acceleration [m/s^2]')
title('Acceleration vs. Path Position')
leg1 = legend('a_x','a_y','a_{tot}');
set(leg1,'Interpreter','tex')

% Plot speed profile
figure; a(1) = subplot(2,1,1); plot( path.s, path.UxDes, 'linew', 2); 
ylabel('U_x, m/s'); ylim([-1 15]); grid on
a(2) = subplot(2,1,2); plot(path.s, path.axDes, path.s, path.ayDes, path.s, sqrt(path.axDes.^2 + path.ayDes.^2), 'linew', 2); 
linkaxes(a, 'x');  ylabel('acceleration, m/s^2'); xlabel('s, m'); 
ylim([-4, 5]); grid on; legend('a_x', 'a_y', 'Location', 'best')


%--------------------------------------------------------------------------
%% ANIMATE VEHICLE
%--------------------------------------------------------------------------
% animate(path, veh, dpsi_, s_, e_, delta_)
