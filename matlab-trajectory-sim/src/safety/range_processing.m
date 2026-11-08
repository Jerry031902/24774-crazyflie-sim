function [out, history] = range_processing(frame, history, now, p)
%RANGE_PROCESSING  Filter distinct samples only; compensate history for travel.
% Range closing rate is diagnostic for this static-world simulator, not a
% substitute for the vehicle velocity. Turning invalidates the short history.
if isempty(history)
    history = struct('seq', 0, 'frames', [], 'previous', []);
end
out = struct('ranges', frame.ranges, 'frame', frame, 'new_frame', false, ...
    'closing_rate', zeros(4, 1), 'age', now - frame.sample_time);
if frame.seq <= history.seq || frame.seq == 0
    if ~isempty(history.previous), out.ranges = history.previous.ranges; end
    return
end
out.new_frame = true;
if ~isempty(history.frames) && ...
        abs(wrap_to_pi(frame.psi - history.frames(end).psi)) > p.safety.filter_yaw_tol
    history.frames = [];
end
history.frames = [history.frames, frame];
history.frames = history.frames(max(1, end-p.safety.filter_window+1):end);
values = zeros(4, numel(history.frames));
for k = 1:numel(history.frames)
    % Without a bearing to a hit, subtract the full translation norm.
    values(:, k) = max(history.frames(k).ranges - norm(frame.pos-history.frames(k).pos), 0);
end
out.ranges = median(values, 2);
if ~isempty(history.previous)
    old = history.previous;
    dt = frame.sample_time - old.frame.sample_time;
    if dt > 0 && abs(wrap_to_pi(frame.psi-old.frame.psi)) <= p.safety.filter_yaw_tol
        ok = isfinite(old.ranges) & isfinite(out.ranges);
        out.closing_rate(ok) = (old.ranges(ok)-out.ranges(ok)) / dt;
    end
end
history.seq = frame.seq;
history.previous = out;
end
