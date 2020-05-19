function [ux_out, uy_out, r_out, e_out, s_out, dpsi_out] = ...
    gps_model(ux_0, uy_0, r_0, e_0, s_0, dpsi_0, elapsed_time)
% GPS_MODEL Returns the positional estimates. GPS model includes noise
% corrupting the positional states and refresh rates that are different
% than controller rate.
% 
% Assume Gaussian iid processes for location and dpsi
% 
% Inputs:
%   ux_0:           meters/sec
%   uy_0:           meters/sec
%   r_0:            rad/sec
%   e_0:            meters
%   s_0:            meters
%   dpsi_0:         radians
%   elapsed_time:   sec, since start of sim
% 
% Outputs:
%   ux_out:         meters/sec, forward velocity for this time interval
%   uy_out:         meters/sec, lateral velocity for this time interval
%   r_out:          rad/sec, yaw rate for this time interval
%   e_out:          meters, lateral error for this time interval
%   s_out:          meters, path position for this time interval
%   dpsi_out:       rad, heading error for this time interval

% noise values are based on RT4003 datasheet
std_loc = 0.01;         % 1 cm standard deviation of position error 
                        % from datasheet
std_dpsi = 1e-3;        % roughly 1 milliradian standard deviation 
                        % from datasheet (actually 0.05 degrees)
std_uxy = 0.014;        % from datasheet (roughly)
                        
% update rate from datasheet
update_rate = 100;      % Hz
update_time = 1 / update_rate;

% also add noise to r with an inverse notch filter at 15 Hz
f_notch = 15 / update_rate;
bw_notch = 0.05;
[b, a] = iirnotch(f_notch, bw_notch);

std_r = 0.01;           % rad/sec

% persistent variable to keep track of when we last were updated
persistent last_update;

% persistent variables to keep track of the most recent GPS measurement
persistent e_gps;
persistent s_gps;
persistent dpsi_gps;
persistent ux_gps;
persistent uy_gps;
persistent r_gps;

% persistent variables for notch filter
persistent r_noises;
persistent num_measurements;

if (isempty(last_update))
    last_update = elapsed_time;
    
    std_e = std_loc;
    std_s = std_loc;

    % actually sample noise model
    e_noise = std_e * randn();
    s_noise = std_s * randn();
    dpsi_noise = std_dpsi * randn();
    ux_noise = std_uxy * randn();
    uy_noise = std_uxy * randn();
    r_noise = std_r * randn();
    
    % gps measurements for the next time interval
    e_gps = e_0 + e_noise;
    s_gps = s_0 + s_noise;
    dpsi_gps = dpsi_0 + dpsi_noise;
    ux_gps = ux_0 + ux_noise;
    uy_gps = uy_0 + uy_noise;
    
    % initialize filter
    r_noises = zeros(100*30, 1);
    num_measurements = 1;
    r_noises(1) = r_noise;

    % hacky way to update matlab filter at each measurement
    r_filter = filter(a, b, r_noises(1:num_measurements));
    
    r_gps = r_0 + r_filter(end);
else
    if (elapsed_time - last_update > update_time)
        last_update = elapsed_time;

        std_e = std_loc;
        std_s = std_loc;

        % actually sample noise model
        e_noise = std_e * randn();
        s_noise = std_s * randn();
        dpsi_noise = std_dpsi * randn();
        ux_noise = std_uxy * randn();
        uy_noise = std_uxy * randn();
        r_noise = std_r * randn();
        
        % gps measurements for the next time interval
        e_gps = e_0 + e_noise;
        s_gps = s_0 + s_noise;
        dpsi_gps = dpsi_0 + dpsi_noise;
        ux_gps = ux_0 + ux_noise;
        uy_gps = uy_0 + uy_noise;
    
        % update yaw rate filter
        num_measurements = num_measurements + 1;
        r_noises(num_measurements) = r_noise;

        % hacky way to update matlab filter at each measurement
        r_filter = filter(a, b, r_noises(1:num_measurements));

        r_gps = r_0 + r_filter(end);
    end
end

ux_out = ux_gps;
uy_out = uy_gps;
r_out = r_gps;
e_out = e_gps;
s_out = s_gps;
dpsi_out = dpsi_gps;

end