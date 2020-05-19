function [brake_current] = brakeDynamics(brake_command_, brake_actual_, veh, dt)
%BRAKEDYNAMICS Calculates brake forces given a first order system with a
%time delay
persistent a b
if isempty(a)
    % If first time through - find difference equation coeffs
    brakeSys = tf([1/veh.brake_tau], [1 1/veh.brake_tau]);
    brakeSysD = c2d(brakeSys,dt);
    [num, den] = tfdata(brakeSysD);
    a = num{1}(2);
    b = -den{1}(2);    
end


% Pure time delay
elements_delay = veh.brake_delay/dt + 1;
command = brake_command_(elements_delay);

% First order dynamics
% Use brake_actual_(2) because it's already been shifted
brake_current = b*brake_actual_(2) + a*command;

end

