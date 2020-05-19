function [Fx_lim] = Fx_limits(Fx, veh, f_tire, r_tire)
%FX_LIMITS Calculates limited longitudinal tire force including friction
%and engine limits

if (Fx) < 0
    Fx_f = veh.brakeDistro(1)*Fx;
    Fx_r = veh.brakeDistro(2)*Fx;
else
    Fx_f = veh.driveDistro(1)*Fx;
    Fx_r = veh.driveDistro(2)*Fx;
end

% Saturate front tire
Fx_f = max(Fx_f, -veh.Wf*f_tire.mu);
Fx_f = min(Fx_f, veh.Wf*f_tire.mu);

% Saturate rear tire
Fx_r = max(Fx_r, -veh.Wr*r_tire.mu);
Fx_r = min(Fx_r, veh.Wr*r_tire.mu);

% Give limited total longitudinal force
Fx_lim = Fx_f + Fx_r;

end

