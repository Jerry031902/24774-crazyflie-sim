function result = primary_monitor(data, x, psi, u_body, yaw_rate, ground_velocity, now, p)
%PRIMARY_MONITOR  Assess one candidate step followed by controlled braking.
% This rule-based safety filter uses only observations and estimated state.
% Cone closure is worst-case within each observed cone. Unseen motion is
% rejected; unobserved obstacles between cones cannot be certified clear.
result = struct('trip', false, 'reason', '', 'margin', Inf, ...
    'required', zeros(4,1), 'available', inf(4,1), 'stop_distance', 0);
if any(~isfinite([x(:); psi; u_body(:); yaw_rate; ground_velocity(:)]))
    result.trip = true; result.reason = 'invalid_state_or_command'; return
end
frame = data.frame;
if any(isnan(data.ranges)) || any(data.ranges < 0)
    result.trip = true; result.reason = 'invalid_processed_ranges'; return
end
if frame.seq == 0 || any(~ismember(frame.status, [1 2])) || ...
        now-frame.sample_time > p.safety.max_data_age
    result.trip = true; result.reason = 'processed_data_unavailable'; return
end
D = [1 0; 0 1; -1 0; 0 -1];
R = rot2d(frame.psi);
command_world = rot2d(psi) * u_body;
% Require a fresh cone looking along a significant proposed translation.
if norm(command_world) > p.safety.approach_speed
    angle = acos(min(max(D * R' * command_world / norm(command_world), -1), 1));
    if min(angle) > max(p.ranger.fov/2, deg2rad(2)) + 1e-9
        result.trip = true; result.reason = 'unobserved_direction'; return
    end
end
drift = ground_velocity - x(3:4);
acc = sat_norm((sat_norm(command_world, p.v_max)-x(3:4))/max(p.tau,p.dt), p.a_max);
next_v = sat_norm(x(3:4)+acc*p.dt, p.v_max) + drift;
c_now = cone_closure(R' * ground_velocity, D, p.ranger.fov/2);
c_next = cone_closure(R' * next_v, D, p.ranger.fov/2);
closing = max(c_now, c_next);
v_next = norm(next_v);
result.stop_distance = stopping_distance(v_next, p);
stop = result.stop_distance * c_next / max(v_next, eps);
sigma = p.ranger.noise + p.ranger.noise_rel * min(data.ranges, p.ranger.range_max);
% Sensor-face offset is only credited along the worst ray of the cone.
offset = p.ranger.offset(:) .* ones(4, 1);
result.available = min(data.ranges, p.ranger.range_max) + offset*cos(p.ranger.fov/2)-p.radius;
% A decelerating vehicle may have travelled farther since sampling than
% current_speed * age suggests. Account for the observed sample-to-now motion.
travel = cone_closure(R'*(x(1:2)-frame.pos),D,p.ranger.fov/2);
reaction = max(closing*max(now-frame.sample_time,0),travel)+closing*p.dt;
result.required = p.safety.margin + p.safety.noise_sigma*sigma + reaction + stop;
look = closing > p.safety.approach_speed;
if any(look)
    result.margin = min(result.available(look)-result.required(look));
    result.trip = result.margin <= 0;
    if result.trip, result.reason = 'stopping_distance'; end
end
end

function c = cone_closure(v, D, half)
speed = norm(v);
if speed < eps, c = zeros(4,1); return; end
a = acos(min(max(D*v/speed, -1), 1));
c = max(speed*cos(max(a-half, 0)), 0);
end
