function [Fy, Fy_max] = Fy_Fiala(alpha, Fz, f_tire)

% Unpack parameters
mup = f_tire.mu;
Ca = f_tire.Cy;

% calculate sliding slip angle
asl = atan( 3 * mup * Fz / Ca );

% calculate lateral force
if ( abs(alpha) < asl )
    Fy = - Ca * tan( alpha ) ...
        + (Ca^2/(3*mup*Fz)) * abs(tan(alpha)) * tan(alpha) ...
        - (Ca^3/(27*mup^2*Fz^2)) * tan(alpha)^3;
else
    Fy = - mup * Fz * sign( alpha );
end

Fy_max = mup * Fz;
