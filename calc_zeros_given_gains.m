function [zero1, zero2] = calc_zeros_given_gains(Kp,Ki,Kd)
    % a*s^2 + s + b
    a = Kd/Kp;
    b = Ki/Kp;
    
    zero1 = (-1 + sqrt(1^2 - 4*a*b))/(2*a);
    zero2 = (-1 - sqrt(1^2 - 4*a*b))/(2*a);    
end

