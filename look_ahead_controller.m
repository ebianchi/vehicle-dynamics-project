function [delta, Fx] = look_ahead_controller(Ux, e, dpsi, kappa, gains, ...
                                             Ux_des, veh, r_tire, f_tire)
% Calculates steer angle and longitudinal force to apply using a lookahead
% control scheme.  This is the first of two controllers developed for the
% project.
% 
% TODO:  add in drag, rolling resistance to feed-forward term,
% adjust gains appropriately.

    dpsi_ss = kappa*(veh.m*veh.a*Ux^2 / (veh.L * r_tire.Ca_lin) - veh.b);
    delta_ff = gains.K_la * gains.x_la * dpsi_ss / f_tire.Ca_lin + ...
               kappa * (veh.L + veh.K * Ux^2);

    delta = -gains.K_la * (e + gains.x_la*dpsi)/f_tire.Ca_lin ...
                + delta_ff;
    Fx = gains.K_long*(Ux_des - Ux);
    
end

