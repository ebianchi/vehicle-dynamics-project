function [delta_current] = steeringDynamics(delta_command_, delta_actual_, veh, dt)
%CALCWEIGHTTRANSFERLONG Calculates delta angle given a first order system with a
%time delay

persistent a b
if isempty(a)
    % If first time through - find difference equation coeffs
    deltaSys = tf([1/veh.steer_tau], [1 1/veh.steer_tau]);
    deltaSysD = c2d(deltaSys,dt);
    [num, den] = tfdata(deltaSysD);
    a = num{1}(2);
    b = -den{1}(2);    
end


% Pure time delay
elements_delay = veh.steer_delay/dt + 1;
command = delta_command_(elements_delay);

% Saturate delta
command = min(command, veh.steer_limit);
command = max(command, -veh.steer_limit);

% First order dynamics
% Use delta_actual_(2) because it's already been shifted
delta_current = b*delta_actual_(2) + a*command;

end

