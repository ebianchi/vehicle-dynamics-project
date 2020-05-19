function [engine_current] = engineDynamics(engine_command_, engine_actual_, Ux, veh, dt)
%BRAKEDYNAMICS Calculates brake forces given a first order system with a
%time delay
persistent a b
if isempty(a)
    % If first time through - find difference equation coeffs
    engineSys = tf([1/veh.engine_tau], [1 1/veh.engine_tau]);
    engineSysD = c2d(engineSys,dt);
    [num, den] = tfdata(engineSysD);
    a = num{1}(2);
    b = -den{1}(2);    
end

% Saturate commands
max_command = min(veh.maxPower_W/Ux, veh.maxFxEngine_N);

% Pure time delay
elements_delay = veh.engine_delay/dt + 1;
command = engine_command_(elements_delay);
command = min(command, max_command);

% First order dynamics
% Use brake_actual_(2) because it's already been shifted
engine_current = b*engine_actual_(2) + a*command;

end

