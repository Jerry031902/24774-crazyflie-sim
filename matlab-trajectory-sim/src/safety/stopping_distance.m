function d = stopping_distance(speed, p)
%STOPPING_DISTANCE  Exact infinite-tail distance of the discrete zero-target model.
% Semi-implicit Euler, first-order velocity lag and acceleration saturation,
% matching dynamics_step. Assumes a stationary target ground velocity and
% constant, estimated drift compensation; not a hardware braking certificate.
speed = max(speed, 0);
tau = max(p.tau, p.dt);
n = max(ceil((speed - p.a_max * tau) / (p.a_max * p.dt)), 0);
remaining = max(speed - n .* p.a_max * p.dt, 0);
d = p.dt * (n .* speed - p.a_max * p.dt * n .* (n + 1) / 2) ...
    + remaining * (tau - p.dt);
end
