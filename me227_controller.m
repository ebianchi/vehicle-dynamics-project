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
%% PID Variables
%--------------------------------------------------------------------------
persistent int_e;
dt = 0.005;             % Niki runs at 200Hz

% Set control gains -- PID controller
% TODO:  Fill in units here and tune more thoughtfully
gains.K_lat_p_PID = 1.2;
gains.K_lat_i_PID = 0.8;
gains.K_lat_d_PID = 0.3;

windup_thresh = 0.4;  % anti-windup

% n = 200;
% persistent past_n_e_measurements;   % filter the e measurements
% if isempty(past_n_e_measurements)
%     past_n_e_measurements = zeros(n,1);
%     for i = 1:n
%         past_n_e_measurements(i) = e;
%     end
% end
% 
% m = 1;
% persistent past_m_de_measurements;   % filter the de measurements
% if isempty(past_m_de_measurements)
%     past_m_de_measurements = zeros(m,1);
%     for i = 1:m
%         past_m_de_measurements(i) = Ux*sin(dpsi) + Uy*cos(dpsi);
%     end
% end

% persistent last_delta;  % save last command so we can perform smoothing
% smoothing_thresh = deg2rad(0.8);   % choose a threshold of [deg/timestep]

% persistent row;
% if isempty(row)
%     row = 1;
% end

%--------------------------------------------------------------------------
%% Control Parameters
%--------------------------------------------------------------------------
% !! Do not change the code in this block - change the desired gains in
% your simulator script!!
K_la = gains.K_la;
x_la = gains.x_la;
K_long = gains.K_long;

K_lat_p  = gains.K_lat_p_PID;
K_lat_i  = gains.K_lat_i_PID;
K_lat_d  = gains.K_lat_d_PID;

%--------------------------------------------------------------------------
%% Find Path Dependent Parameters
%--------------------------------------------------------------------------
% !! Do not change the code in this block - change the desired speed in
% your simulator script!!
Ux_des = interp1(path.s, path.UxDes, s);

% Find Curvature for the current distance along the path via interpolation
kappa = interp1(path.s, path.k, s);


%% Lateral Control Law
%--------------------------------------------------------------------------
if control_mode == 1  % lookahead controller
    dpsi_ss  = kappa*(veh.m*veh.a*Ux^2 / (veh.L * r_tire.Ca_lin) - veh.b);
    delta_ff = K_la * x_la * dpsi_ss / f_tire.Ca_lin + kappa * (veh.L + ...
               K_radpmps2 * Ux^2);

    delta = -K_la * (e + x_la*dpsi)/f_tire.Ca_lin + delta_ff;
    
else  % custom controller
%     % filter the e term
%     for i = n:-1:2
%         past_n_e_measurements(i) = past_n_e_measurements(i-1);
%     end
%     past_n_e_measurements(1) = e;
%     filtered_e = sum(past_n_e_measurements)/n;
%     
%     % filter the de term
%     for i = m:-1:2
%         past_m_de_measurements(i) = past_m_de_measurements(i-1);
%     end
%     past_m_de_measurements(1) = Ux*sin(dpsi) + Uy*cos(dpsi);
%     filtered_de = sum(past_m_de_measurements)/m;
    
    if isempty(int_e)                  % update integral term
        int_e = e*dt;
    else
        int_e = int_e + e*dt;
    end
    
    % perform anti-windup
    if abs(int_e) > windup_thresh
        int_e = windup_thresh * sign(int_e);
    end
    
    de = Ux*sin(dpsi) + Uy*cos(dpsi);  % calculate derivative term
    
    % Note:  the negative signs are because a positive lateral error is
    % compensated for by a negative steering angle.
    lat_i = K_lat_i * -int_e;          % calculate integral contribution
    lat_p = K_lat_p * -e;     % calculate proportional contribution
    lat_d = K_lat_d * -de;    % calculate derivative contribution
    delta = lat_p + lat_i + lat_d;     % get control for delta
    
%     % contain the commanded steering angle within limits of steering wheel
%     if abs(delta) > deg2rad(30)
%         delta = deg2rad(30) * sign(delta);
%     end
%     
%     % check if our new commanded steering angle is very different from the
%     % last command
%     if isempty(last_delta)
%         last_delta = delta;
%     end
%     
%     if abs(delta - last_delta) > smoothing_thresh
%         delta = last_delta + (smoothing_thresh * sign(delta-last_delta));
%     end
% 
%     last_delta = delta;                % save away this delta for next time
    
%     % Save away PID terms for exploration
%     row_to_write = join(string(['A', string(row), ':C', string(row)]), "");
%     writematrix([lat_p, lat_i, lat_d],'PID_terms.xls','Range',row_to_write);
%     row = row + 1;
%     disp(lat_p + "  " + lat_i + "  " + lat_d);
end

%--------------------------------------------------------------------------
%% Longitudinal Control Law
%--------------------------------------------------------------------------
a_des = interp1(path.s, path.axDes, s);     % desired acceleration
rho_air = 1.225;                            % density of air
Frr = veh.m * g * .015;                     % Rolling Resistance Force
Fd = .594 * rho_air * Ux^2 / 2;             % Drag Force
Ux_error = Ux_des - Ux;                     % Proportional Error Term
Fx = K_long * Ux_error + veh.m*a_des + Frr + Fd;

end
