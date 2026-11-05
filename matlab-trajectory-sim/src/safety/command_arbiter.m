function [s, accept, resume] = command_arbiter(s, primary, backup, x, velocity, now, seq, p)
%COMMAND_ARBITER  One command owner; either monitor can retain the override.
accept = false; resume = false;
speed = norm(velocity);
trip = primary.trip || backup.trip;
if trip
    s.clear_count = 0;
    if backup.trip
        reason = backup.reason; source = 'backup';
    else
        reason = primary.reason; source = 'primary';
    end
    if strcmp(s.state,'NORMAL') || strcmp(s.state,'RECOVER')
        s.state = 'BRAKE';
        % A paused, stationary scan is still valid when timestamped samples
        % are retained. Replan only when translation was interrupted.
        s.need_replan = s.need_replan || speed > p.safety.stop_speed || ...
            norm(s.last_nominal) > p.safety.approach_speed;
        s.brake_since = now; s.brake_distance = 0;
        s.reason = reason; s.source = source;
        s = event(s,now,'brake',x,speed,NaN,NaN);
    elseif ~strcmp(reason,'waiting_for_ranges')
        s.reason = reason; s.source = source;
    end
end
if strcmp(s.state,'BRAKE') && isfinite(speed) && speed <= p.safety.stop_speed
    s.state = 'HOLD'; s.hold_pos = x(1:2);
    s = event(s,now,'stopped',x,speed,now-s.brake_since,s.brake_distance);
end
if strcmp(s.state,'HOLD')
    if ~trip && speed <= p.safety.stop_speed && seq > s.clear_seq
        s.clear_count = s.clear_count+1;
    end
    s.clear_seq = max(s.clear_seq,seq);
    if ~trip && s.clear_count >= p.safety.clear_frames
        s.state = 'RECOVER'; s.recover_since = now;
        resume = s.need_replan; s.need_replan = false;
        s = event(s,now,'resume',x,speed,NaN,NaN);
        s.reason = ''; s.source = '';
    end
    return
end
if strcmp(s.state,'BRAKE'), return; end
if strcmp(s.state,'RECOVER') && now-s.recover_since >= p.safety.recover_time
    s.state = 'NORMAL';
end
accept = ~trip;
end

function s = event(s,t,type,x,speed,duration,distance)
s.events(end+1) = struct('t',t,'type',type,'reason',s.reason,'source',s.source, ...
    'pos',x(1:2)','speed',speed,'duration',duration,'distance',distance);
end
