%% Set Vehicle Parameters
% declare parameters for a VW GTI with 4 passengers
% to the MATLAB workspace

% Authors: John Alsterda & Nathan Speilberg for Stanford ME227

%--------------------------------------------------------------------------
%% Constants
%--------------------------------------------------------------------------
g = 9.81;

veh.g = 9.81;                       % [m/s^2]  gravity
veh.rho = 1.225;        % [kg/m^3] Density of air at sea level

%--------------------------------------------------------------------------
%% Vehicle Parameters
%--------------------------------------------------------------------------
veh.m  = 1926.2;                    % [kg]     mass
veh.Iz = 2763.49;                   % [kg-m^2] rotational inertia
veh.a  = 1.264;                     % [m]      distance from CoM to front axle
veh.b  = 1.367;                     % [m]      distance from C0M to rear axle
veh.L  = veh.a + veh.b;             % [m]      wheelbase
veh.Wf = veh.m*veh.g*(veh.b/veh.L);     % [N]      static front axle weight
veh.Wr = veh.m*veh.g*(veh.a/veh.L);     % [N]      static rear axle weight
veh.rW = 0.318;                     % [m]      tire radius
veh.hcg = 0.55;                     % [m]      Distance between "CG" and ground
veh.brakeDistro = [0.68; 0.32];     % [frac]   Brake proportioning front/rear, sum = 1 % not used at present
veh.driveDistro = [1; 0];           % [frac]   Fraction front, rear wheel drive, sum = 1, currently used
veh.cdA = 0.594;                    % [m^2]    Coefficient of drag
veh.frr = 0.015;                    % [unitless] Coefficient of rolling resistance

%--------------------------------------------------------------------------
%% Actuator Dynamics
%--------------------------------------------------------------------------
veh.brake_tau = 0.2/3;              % [s]   Brakes reach 95% command after 0.3s
veh.brake_delay = 0.1;              % [s]   Pure delay in brakes

veh.maxPower_W = 131 * 1e3;         % [W]   Max engine power
veh.maxFxEngine_N = 5500;           % [N]   Max force from engine
veh.engine_tau = 0.6/3;             % [s]   Engine reachs 95% command after 0.8s
veh.engine_delay = 0.001;           % [s]   Pure delay in engine

veh.steer_tau = 0.2/3;             % [s]   Brakes reach 95% command after 0.05s
veh.steer_delay = 0.001;            % [s]   Pure delay in brakes
veh.steer_limit = deg2rad(30);      % [rad] Steering limit

%--------------------------------------------------------------------------
%% Tire Parameters
%--------------------------------------------------------------------------
% Front tires
f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
f_tire.Cy     = 120e3;         % [N/rad]  fiala model cornering stiffness
f_tire.mu_s   = 0.90;           %          sliding friction coefficient
f_tire.mu     = 0.90;           %          peak friction coefficient

% Rear tires
r_tire.Ca_lin = 120000;
r_tire.Cy     = 200e3;
r_tire.mu_s   = 0.91;
r_tire.mu     = 0.91;

%--------------------------------------------------------------------------
%% Helper Functions
%--------------------------------------------------------------------------
deg2rad = @(deg) (pi/180)*deg;
rad2deg = @(rad) (180/pi)*rad;
