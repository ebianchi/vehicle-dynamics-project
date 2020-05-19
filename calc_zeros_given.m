function [zero1, zero2, Kp, Ki, Kd] = calc_zeros_given(a,b,Kp)
    % a*s^2 + s + b
    zero1 = (-1 + sqrt(1^2 - 4*a*b))/(2*a);
    zero2 = (-1 - sqrt(1^2 - 4*a*b))/(2*a);
    
    Ki = b * Kp;
    Kd = a * Kp;    
end

