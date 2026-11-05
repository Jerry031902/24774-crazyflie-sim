function r = popup_summary(L, env, p)
%POPUP_SUMMARY  Ground-truth scoring; these values never drive the monitors.
assert(isfield(L,'obstacle_events') && ~isempty(L.obstacle_events), ...
    'popup_summary:noEvent','The run must reach an obstacle activation.');
e = L.obstacle_events(1); after = L.t >= e.t-1e-9;
i = find(after,1);
fresh = find(after & L.ranger_sample_time >= e.t-1e-9,1);
zero = find(after & vecnorm(L.u_body,2,2) < 1e-6,1);
override = find(after & ~L.command_accepted,1);
stopped = find(after & vecnorm(L.ground_velocity,2,2) <= p.safety.stop_speed,1);
% Actual grid-aligned sensor schedule, including held values and delivery.
sample_t = ceil((e.t-1e-9)/p.ranger.period)*p.ranger.period;
sample_t = ceil((sample_t-1e-9)/p.dt)*p.dt;
delivery_t = ceil((sample_t+p.ranger.latency-1e-9)/p.dt)*p.dt;
reaction = delivery_t-e.t;
zero_stop = stopping_distance(e.speed,p);
% Best achievable straight-line deceleration for this discrete plant at a_max.
n = floor(e.speed/(p.a_max*p.dt));
max_acc_stop = p.dt*(n*e.speed-p.a_max*p.dt*n*(n+1)/2);
gaps = obstacle_distance(env,L.x(:,1:2),L.t)-p.radius;
collided = L.collided || any(gaps(after) < 0);
if collided, outcome = 'collision';
elseif L.arrived, outcome = 'arrived';
elseif ~isempty(stopped), outcome = 'stopped_not_finished';
else, outcome = 'time_limit'; end
r = struct('enabled',L.safety_enabled,'noise_m',p.ranger.noise,'seed',p.seed, ...
    'appear_s',e.t,'gap_at_appear_m',e.gap,'speed_at_appear_m_s',e.speed, ...
    'scheduled_reaction_s',reaction,'expected_first_delivery_s',delivery_t, ...
    'first_fresh_delivery_s',value_at(L.t,fresh), ...
    'first_zero_command_s',value_at(L.t,zero),'first_override_s',value_at(L.t,override), ...
    'first_stopped_s',value_at(L.t,stopped),'zero_target_stop_m',zero_stop, ...
    'zero_target_required_gap_m',e.speed*reaction+zero_stop, ...
    'max_acc_required_gap_m',e.speed*reaction+max_acc_stop, ...
    'zero_target_margin_m',e.gap-e.speed*reaction-zero_stop, ...
    'min_gap_after_m',min(gaps(after)),'gap_at_stop_m',value_at(gaps,stopped), ...
    'travel_to_stop_m',NaN,'collided',collided,'arrived',L.arrived,'outcome',outcome);
if ~isempty(stopped)
    r.travel_to_stop_m = sum(vecnorm(diff(L.x(i:stopped,1:2)),2,2));
end
end

function v = value_at(values,index)
if isempty(index), v = NaN; else, v = values(index); end
end
