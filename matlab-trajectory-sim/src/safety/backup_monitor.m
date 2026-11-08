function result = backup_monitor(frame, x, psi, velocity, now, health, p)
%BACKUP_MONITOR  Raw-data path: never reads primary/filter output.
% An independent conservative continuous-time braking envelope is used here;
% unlike the primary it does not call stopping_distance or predict commands.
result = struct('trip', false, 'fault', false, 'reason', '', 'margin', Inf);
if any(~isfinite([x(:); psi; velocity(:)]))
    result = fail(result, 'invalid_estimate', true); return
end
if frame.seq == 0
    result = fail(result, 'waiting_for_ranges', now > p.safety.max_data_age); return
end
if ~isfinite(frame.seq) || frame.seq < 1 || frame.seq ~= fix(frame.seq) || ...
        ~isfinite(frame.sample_time) || ~isfinite(frame.delivery_time) || ...
        frame.sample_time < 0 || ...
        frame.sample_time > frame.delivery_time+1e-9 || frame.delivery_time > now+1e-9 || ...
        any(~ismember(frame.status,[1 2])) || any(isnan(frame.ranges)) || ...
        any(frame.ranges < 0) || ~isfinite(frame.psi) || any(~isfinite(frame.pos)) || ...
        any((frame.status == 1) ~= isfinite(frame.ranges)) || ...
        any(frame.ranges(isfinite(frame.ranges)) > p.ranger.range_max)
    result = fail(result, 'invalid_raw_frame', true); return
end
if isfield(health,'last_seq') && (frame.seq < health.last_seq || ...
        (frame.seq == health.last_seq && frame.sample_time ~= health.last_sample_time) || ...
        (frame.seq > health.last_seq && frame.sample_time <= health.last_sample_time))
    result = fail(result,'nonmonotonic_frame',true); return
end
if now-frame.sample_time > p.safety.max_data_age
    result = fail(result, 'ranger_timeout', true); return
end
if now-health.primary_time > p.safety.heartbeat_timeout
    result = fail(result, 'primary_timeout', true); return
end
if health.navigator_required && now-health.navigator_time > p.safety.heartbeat_timeout
    result = fail(result, 'navigator_timeout', true); return
end
offset = p.ranger.offset(:) .* ones(4,1);
gap = min(frame.ranges, p.ranger.range_max)+offset*cos(p.ranger.fov/2)-p.radius;
if any(gap(frame.status == 1) < p.safety.hard_gap)
    result = fail(result, 'hard_distance', false); return
end
v = rot2d(frame.psi)' * velocity;
speed = norm(v);
D = [1 0; 0 1; -1 0; 0 -1];
if speed < eps, return; end
a = acos(min(max(D*v/speed,-1),1));
closing = max(speed*cos(max(a-p.ranger.fov/2,0)),0);
tau = max(p.tau,p.dt);
% Upper envelope for first-order deceleration with saturation, including tail.
brake = tau*speed;
if speed > p.a_max*tau
    brake = speed^2/(2*p.a_max)+p.a_max*tau^2/2;
end
sigma = p.ranger.noise+p.ranger.noise_rel*min(frame.ranges,p.ranger.range_max);
displacement = rot2d(frame.psi)'*(x(1:2)-frame.pos);
length_m = norm(displacement);
travel = zeros(4,1);
if length_m > eps
    angle = acos(min(max(D*displacement/length_m,-1),1));
    travel = max(length_m*cos(max(angle-p.ranger.fov/2,0)),0);
end
required = p.safety.backup_margin+p.safety.backup_noise_sigma*sigma + ...
    max(closing*(p.ranger.latency+p.ranger.period),travel) + ...
    closing*p.dt + brake*closing/speed;
look = closing > p.safety.approach_speed;
if any(look)
    result.margin = min(gap(look)-required(look));
    if result.margin <= 0, result = fail(result, 'backup_distance', false); end
end
end

function r = fail(r, reason, fault)
r.trip = true; r.reason = reason; r.fault = fault;
end
