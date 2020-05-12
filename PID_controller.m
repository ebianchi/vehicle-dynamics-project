function [delta, Fx] = PID_controller(Ux, e, dpsi, kappa, gains, path, ...
                                      veh, r_tire, f_tire)
% Calculates steer angle and longitudinal force to apply using a PID
% controller on both lateral and longitudinal errors.  This is the second
% of two controllers developed for the project.

    %% lateral control
    
    
    %% longitudinal control

%     dpsi_ss = kappa*(veh.m*veh.a*Ux^2 / (veh.L * r_tire.Ca_lin) - veh.b);
%     delta_ff = gains.K_la * gains.x_la * dpsi_ss / f_tire.Ca_lin + ...
%                kappa * (veh.L + veh.K * Ux^2);
% 
%     delta = -gains.K_la * (e + gains.x_la*dpsi)/f_tire.Ca_lin ...
%                 + delta_ff;
%     Fx = gains.K_long*(path.UxDes(1) - Ux);
    
    
%     gains.K_long_p_PID = 0;
%     gains.K_long_i_PID = 0;
%     gains.K_long_d_PID = 0;
%     gains.K_lat_p_PID = 0;
%     gains.K_lat_i_PID = 0;
%     gains.K_lat_d_PID = 0;
    
end

