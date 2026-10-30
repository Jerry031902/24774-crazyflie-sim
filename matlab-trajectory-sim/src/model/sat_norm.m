function [v, saturated] = sat_norm(v, vmax)
%SAT_NORM  Scale a vector down so its 2-norm does not exceed vmax.
%   Keeps the direction, which is how a speed or acceleration limit acts
%   on a multirotor (clipping each axis separately would let diagonal
%   motion exceed the limit and bend the direction).
n = norm(v);
saturated = n > vmax;
if saturated
    v = v * (vmax / n);
end
end
