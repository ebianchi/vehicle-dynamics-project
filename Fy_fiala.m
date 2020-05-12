function F_y = Fy_fiala(alpha, Fz, tire)
%FY_FIALA
%   Calculates the tire force at a single tire using a fiala tire model
%
%   Inputs
%       alpha:  Slip angle [rad]
%       Fz:     Normal load on tire [N]
%       tire:   Tire parameters struct
%
%   Output:
%       F_y:    Lateral tire force [N]

% Calculate alpha sliding
alpha_sl = atan(3*tire.mu*Fz/tire.Cy);

% Calculate tire force if not sliding 
% NOTE: We recommend making each of the 3 terms its own variable and then
% combining to find the tire force
a1 = -tire.Cy;
a2 = ((tire.Cy^2)/(3*tire.mu*Fz))*(2-(tire.mu_s/tire.mu));
a3 = -((tire.Cy^3)/(9*tire.mu^2*Fz^2))*(1-((2*tire.mu_s)/(3*tire.mu)));

% Check if tire is sliding or not
if abs(alpha) < alpha_sl
    F_y = a1*tan(alpha) + a2*abs(tan(alpha))*tan(alpha) + a3*tan(alpha)^3;
else
    F_y = -tire.mu_s*Fz*sign(alpha);
end

end

