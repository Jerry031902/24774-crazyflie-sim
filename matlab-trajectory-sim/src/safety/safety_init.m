function s = safety_init(x, now)
%SAFETY_INIT  Safety and navigation have separate state machines.
s.state = 'HOLD';
s.reason = 'waiting_for_ranges';
s.source = 'startup';
s.history = [];
s.clear_count = 0;
s.clear_seq = 0;
s.need_replan = false;
s.recover_since = now;
s.last_pos = x(1:2);
s.last_time = now;
s.hold_pos = x(1:2);
s.primary_time = now;
s.navigator_time = now;
s.raw_seq = 0; s.raw_sample_time = -Inf;
s.last_nominal = [0; 0];
s.last_yaw_rate = 0;
s.last_error = '';
s.brake_since = NaN;
s.brake_distance = 0;
s.last_primary = struct('trip', false, 'reason', '', 'margin', Inf, ...
    'required', zeros(4,1), 'available', inf(4,1), 'stop_distance', 0);
s.events = struct('t', {}, 'type', {}, 'reason', {}, 'source', {}, ...
    'pos', {}, 'speed', {}, 'duration', {}, 'distance', {});
end
