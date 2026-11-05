function [u, yaw_rate, nav, s, diag] = safety_step(nav, s, x, psi, frame, now, p, availability)
%SAFETY_STEP  Sensor -> candidate -> parallel monitors -> arbiter -> command.
% availability is a deterministic simulation fault-injection interface only.
if nargin < 8
    availability = struct('primary',true,'navigator',true,'processed_bias',0);
end
dt = now-s.last_time;
if dt > 0
    velocity = (x(1:2)-s.last_pos)/dt;
else
    velocity = x(3:4);
end
if strcmp(s.state,'BRAKE')
    s.brake_distance = s.brake_distance+norm(x(1:2)-s.last_pos);
end
s.last_pos = x(1:2); s.last_time = now;
[data,s.history] = range_processing(frame,s.history,now,p);
data.ranges = data.ranges+availability.processed_bias;
candidate_nav = nav;
nominal = [0;0]; candidate = nominal; candidate_yaw = 0;
running = ismember(s.state,{'NORMAL','RECOVER'});
if running && availability.navigator
    try
        [nominal,candidate_yaw,candidate_nav] = navigator_step(nav,x,psi,data.ranges,now,p,frame);
        s.navigator_time = now;
        s.last_nominal = nominal; s.last_yaw_rate = candidate_yaw;
    catch err
        s.last_error = err.message;
        s.navigator_time = -Inf;
    end
elseif running
    nominal = s.last_nominal; candidate_yaw = s.last_yaw_rate;
end
candidate = nominal;
if strcmp(s.state,'RECOVER')
    f = min(max((now-s.recover_since)/p.safety.recover_time,0),1);
    cap = p.safety.recover_speed+f*(p.nav.v_cruise-p.safety.recover_speed);
    candidate = sat_norm(candidate,cap);
end
if availability.primary
    try
        primary = primary_monitor(data,x,psi,candidate,candidate_yaw,velocity,now,p);
        s.primary_time = now; s.last_primary = primary;
    catch err
        s.last_error = err.message;
        s.primary_time = -Inf;
        primary = s.last_primary;
    end
else
    primary = s.last_primary;
end
health = struct('primary_time',s.primary_time,'navigator_time',s.navigator_time, ...
    'navigator_required',running || ~availability.navigator, ...
    'last_seq',s.raw_seq,'last_sample_time',s.raw_sample_time);
backup = backup_monitor(frame,x,psi,velocity,now,health,p);
if frame.seq > s.raw_seq && isfinite(frame.sample_time)
    s.raw_seq = frame.seq; s.raw_sample_time = frame.sample_time;
end
[s,accept,resume] = command_arbiter(s,primary,backup,x,velocity,now,frame.seq,p);
if resume
    nav = navigator_resume(nav,x,psi,now);
end
if accept
    nav = candidate_nav;
    u = candidate; yaw_rate = candidate_yaw;
else
    [u,yaw_rate] = fallback_step(s,x,psi,velocity,p);
end
diag = struct('nominal',nominal,'candidate',candidate,'candidate_yaw',candidate_yaw, ...
    'processed',data.ranges,'closing_rate',data.closing_rate,'age',data.age, ...
    'primary',primary,'backup',backup,'accepted',accept,'ground_velocity',velocity);
end
