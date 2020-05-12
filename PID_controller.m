function [delta, Fx, int_Ux_1, int_e_1] = PID_controller(Ux, dUx, int_Ux_0,...
                                           e, de, int_e_0, gains, UxDes)
% Calculates steer angle and longitudinal force to apply using a PID
% controller on both lateral and longitudinal errors.  This is the second
% of two controllers developed for the project.

    %% lateral control
    % note:  a positive lateral error is corrected with a negative steering
    % angle, hence the negative signs for each contribution
    int_e_1 = int_e_0 + e;                % update integral term
    lat_i = gains.K_lat_i_PID * -int_e_1; % calculate integral contribution
    lat_p = gains.K_lat_p_PID * -e;       % calculate proportional contribution
    lat_d = gains.K_lat_d_PID * -de;      % calculate derivative contribution
    delta = lat_p + lat_i + lat_d;        % get control for delta
    
    %% longitudinal control
    int_Ux_1 = int_Ux_0 + (UxDes - Ux);          % update integral term
    long_i = gains.K_long_i_PID * int_Ux_1;      % calculate integral contribution
    long_p = gains.K_long_p_PID * (UxDes - Ux);  % calculate proportional contribution
    long_d = gains.K_long_d_PID * dUx;           % calculate derivative contribution
    Fx = long_p + long_i + long_d;               % get control for Fx
    
end

