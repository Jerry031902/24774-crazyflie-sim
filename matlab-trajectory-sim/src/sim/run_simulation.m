function sim_log = run_simulation(start, waypoints, goal, env, p, faults)
%RUN_SIMULATION  Fly from start through the waypoints to the goal with the reactive navigator.
%   sim_log = RUN_SIMULATION(start, waypoints, goal, env, p) starts at rest
%   at start ([x y]), visits the rows of waypoints (Nx2) in order, then
%   flies to goal ([x y]). Pass waypoints = [] to fly straight to the goal.
%   Every tick:
%     1. read the four rangers: measured every p.ranger.period, delivered
%        p.ranger.latency later and held until the next delivery (Inf
%        before the first), as the firmware driver does
%     2. process ranges, evaluate candidate navigation and parallel monitors
%     3. arbitrate navigation/fallback commands, then update the model
%   The run ends when the goal is reached and the drone has stopped, when
%   it touches an obstacle or wall (if p.stop_on_collision), or at p.t_max.
%
%   sim_log has one row per tick (preallocated, then trimmed):
%     t, x = [px py vx vy], psi (yaw), ranges = [front left back right]
%     (Inf = no return), mode (1 GO_TO_GOAL, 2 AVOID, 3 DONE), wp (target
%     index), dir (direction flown: 1 forward, 2 left, 3 back, 4 right,
%     0 straight at the target), u_body = [forward left], yaw_rate,
%     phase (AVOID phase, an index into phase_names; 0 outside AVOID),
%     hand (wall side while wall-following: +1 left, -1 right, else 0),
%     subgoal (pass point while passing an edge, else NaN), hit_point
%     (NaN outside an avoidance episode), wind
%   and per run: start, targets (the waypoints, then the goal),
%   target_names, phase_names, events (hit, leave, reached, unreachable),
%   sweeps (each wedge scan with its edges and the side chosen; see
%   NAVIGATOR_INIT), unreachable (target indices), arrived, t_arrive,
%   collided, t_collide. Safety fields include safety_state (0 disabled,
%   otherwise index into safety_names), safety_reason/source, safety_events,
%   candidate/nominal commands, monitor trips/margins and measurement metadata.
%   Optional faults is for reproducible validation only: ranger_dropout,
%   primary_timeout, navigator_timeout are [start end] time windows;
%   processed_bias is [start end metres] and never changes the raw path.
%   env may contain SCHEDULE_OBSTACLE events. Active geometry changes before
%   sampling/collision checks; queued measurements keep their original geometry.

if nargin < 6, faults = struct(); end
enabled = isfield(p,'safety') && p.safety.enabled;

if isempty(waypoints)
    waypoints = zeros(0, 2);
elseif size(waypoints, 2) ~= 2
    error('run_simulation:waypoints', ...
        'waypoints must be an N-by-2 list of [x y] points, or [] for none.');
end
start   = start(:)';
targets = [waypoints; goal(:)'];
active_env = environment_at(env,0);
validate_waypoints(start, targets, active_env, p);
rng(p.seed);

N = floor(p.t_max / p.dt + 1e-9) + 1;
t = (0:N-1)' * p.dt;
modes  = {'GO_TO_GOAL', 'AVOID', 'DONE'};
phases = {'sweep', 'pass', 'align', 'follow', 'overshoot'};

PH   = zeros(N, 1);
HAND = zeros(N, 1);
SUB  = nan(N, 2);
X    = zeros(N, 4);
PSI  = zeros(N, 1);
YR   = zeros(N, 1);
RNG  = zeros(N, 4);
MODE = zeros(N, 1);
WP   = zeros(N, 1);
DIR  = zeros(N, 1);
UB   = zeros(N, 2);
HIT  = nan(N, 2);
W    = zeros(N, 2);
SS = zeros(N,1); REASON = repmat({''},N,1); SOURCE = REASON;
NOM = zeros(N,2); CAND = zeros(N,2); CYAW = zeros(N,1);
PROC = inf(N,4); AGE = nan(N,1); SEQ = zeros(N,1);
CLOSING = zeros(N,4);
SAMPLE = nan(N,1); DELIVERY = SAMPLE; SYAW = SAMPLE; SPOS = nan(N,2);
STATUS = zeros(N,4,'uint8'); GV = zeros(N,2);
PT = false(N,1); BT = PT; ACCEPT = true(N,1);
PM = inf(N,1); BM = PM; SD = zeros(N,1); REQUIRED = zeros(N,4); AVAILABLE = inf(N,4);
safety_names = {'NORMAL','BRAKE','HOLD','RECOVER'};

x     = [start(:); 0; 0];
psi   = p.psi0;
nav   = navigator_init(start, targets);
gust  = [0; 0];
tau_g = 2.0;   % gust correlation time [s]
next_read = 0;
pending   = struct('frame', {});       % measured, not yet delivered
ranges    = inf(4, 1);                  % nothing delivered yet
frame = ranger_frame(ranges,NaN,NaN,x(1:2),psi,0);
sequence = 0;
safety = safety_init(x,0);
arrived  = false;  t_arrive  = NaN;
collided = false;  t_collide = NaN;
obstacle_events = struct('id',{},'scheduled_time',{},'t',{},'pos',{},'speed',{},'gap',{});
if isfield(env,'scheduled'), schedule = env.scheduled; else, schedule = struct('time',{},'obstacle',{}); end
active_ids = false(1,numel(schedule));

for k = 1:N
    newly_active = find(~active_ids & [schedule.time] <= t(k)+1e-9);
    if ~isempty(newly_active)
        active_env = environment_at(env,t(k));
        for id = newly_active
            one = env; one.obstacles = schedule(id).obstacle;
            if isfield(one,'scheduled'), one = rmfield(one,'scheduled'); end
            one.xlim = [-Inf Inf]; one.ylim = [-Inf Inf]; % distance to this new polygon only
            gap = obstacle_distance(one,x(1:2)')-p.radius;
            obstacle_events(end+1) = struct('id',id,'scheduled_time',schedule(id).time, ...
                't',t(k),'pos',x(1:2)','speed',norm(x(3:4)),'gap',gap); %#ok<AGROW>
        end
        active_ids(newly_active) = true;
    end
    if t(k) >= next_read - 1e-9
        sequence = sequence+1;
        f = ranger_frame(read_rangers(x(1:2),psi,active_env,p),t(k), ...
            t(k)+p.ranger.latency,x(1:2),psi,sequence);
        pending(end+1).frame = f; %#ok<AGROW>
        next_read = next_read + p.ranger.period;
    end
    while ~isempty(pending) && pending(1).frame.delivery_time <= t(k) + 1e-9
        if ~fault_active(faults,'ranger_dropout',t(k))
            frame = pending(1).frame;
            ranges = frame.ranges;
        end
        pending(1) = [];
    end
    if enabled
        availability = struct('primary',~fault_active(faults,'primary_timeout',t(k)), ...
            'navigator',~fault_active(faults,'navigator_timeout',t(k)), ...
            'processed_bias',0);
        if fault_active(faults,'processed_bias',t(k))
            rows = faults.processed_bias;
            i = find(t(k) >= rows(:,1) & t(k) < rows(:,2),1,'last');
            availability.processed_bias = rows(i,3);
        end
        [u_body,yaw_rate,nav,safety,d] = safety_step(nav,safety,x,psi,frame,t(k),p,availability);
        SS(k) = find(strcmp(safety.state,safety_names));
        REASON{k} = safety.reason; SOURCE{k} = safety.source;
        NOM(k,:) = d.nominal'; CAND(k,:) = d.candidate'; CYAW(k) = d.candidate_yaw;
        PROC(k,:) = d.processed'; AGE(k) = d.age; GV(k,:) = d.ground_velocity';
        CLOSING(k,:) = d.closing_rate';
        PT(k) = d.primary.trip; BT(k) = d.backup.trip; ACCEPT(k) = d.accepted;
        PM(k) = d.primary.margin; BM(k) = d.backup.margin;
        SD(k) = d.primary.stop_distance;
        REQUIRED(k,:) = d.primary.required'; AVAILABLE(k,:) = d.primary.available';
    else
        [u_body,yaw_rate,nav] = navigator_step(nav,x,psi,ranges,t(k),p);
        NOM(k,:) = u_body'; CAND(k,:) = u_body'; CYAW(k) = yaw_rate;
        PROC(k,:) = ranges'; AGE(k) = t(k)-frame.sample_time;
        GV(k,:) = x(3:4)';
    end
    SEQ(k) = frame.seq; SAMPLE(k) = frame.sample_time; DELIVERY(k) = frame.delivery_time;
    SYAW(k) = frame.psi; SPOS(k,:) = frame.pos'; STATUS(k,:) = frame.status';
    wind = p.wind(:) + gust;

    X(k,:)   = x';
    PSI(k)   = psi;
    YR(k)    = yaw_rate;
    RNG(k,:) = ranges';
    MODE(k)  = find(strcmp(nav.mode, modes));
    WP(k)    = nav.wp;
    DIR(k)   = nav.exec_dir;
    UB(k,:)  = u_body';
    W(k,:)   = wind';
    if nav.episode
        HIT(k,:) = nav.hit_point';
    end
    if strcmp(nav.mode, 'AVOID')
        PH(k) = find(strcmp(nav.phase, phases));
        if PH(k) == 2
            SUB(k,:) = nav.subgoal';
        elseif PH(k) >= 3
            HAND(k) = nav.hand;
        end
    end

    % A newly appearing polygon can enclose the centre without any edge
    % touching the body; include signed distance on activation as well.
    overlap_at_appearance = ~isempty(newly_active) && ...
        obstacle_distance(active_env,x(1:2)') < p.radius;
    if p.stop_on_collision && (touches(x(1:2),active_env.edges,p.radius) || overlap_at_appearance)
        collided = true;  t_collide = t(k);
        break
    end
    if strcmp(nav.mode, 'DONE') && norm(GV(k,:)) < p.goal_vtol && ...
            (~enabled || ACCEPT(k))
        arrived = ~ismember(size(targets, 1), nav.unreachable);
        t_arrive = t(k);
        break
    end
    if k == N
        break
    end

    [x, psi] = dynamics_step(x, psi, u_body, yaw_rate, wind, p);
    if p.wind_gust > 0   % Ornstein-Uhlenbeck gusts with standard deviation p.wind_gust
        gust = gust - gust * (p.dt / tau_g) + p.wind_gust * sqrt(2 * p.dt / tau_g) * randn(2, 1);
    end
end

names = target_names(size(targets, 1));
for i = nav.unreachable
    e = nav.events(strcmp({nav.events.type}, 'unreachable') & [nav.events.wp] == i);
    warning('run_simulation:unreachable', ...
        '%s (%.2f, %.2f) is unreachable: the drone came back round the obstacle without getting closer (t = %.1f s). Skipped it.', ...
        names{i}, targets(i,1), targets(i,2), e(1).t);
end

sim_log.t         = t(1:k);
sim_log.x         = X(1:k,:);
sim_log.psi       = PSI(1:k);
sim_log.ranges    = RNG(1:k,:);
sim_log.mode      = MODE(1:k);
sim_log.wp        = WP(1:k);
sim_log.dir       = DIR(1:k);
sim_log.u_body    = UB(1:k,:);
sim_log.yaw_rate  = YR(1:k);
sim_log.phase     = PH(1:k);
sim_log.hand      = HAND(1:k);
sim_log.subgoal   = SUB(1:k,:);
sim_log.hit_point = HIT(1:k,:);
sim_log.wind      = W(1:k,:);
sim_log.start        = start(:)';
sim_log.targets      = targets;
sim_log.target_names = names;
sim_log.phase_names  = phases;
sim_log.events       = nav.events;
sim_log.sweeps       = nav.sweeps;
sim_log.unreachable  = nav.unreachable;
sim_log.arrived   = arrived;
sim_log.t_arrive  = t_arrive;
sim_log.collided  = collided;
sim_log.t_collide = t_collide;
sim_log.safety_enabled = enabled;
sim_log.safety_names = safety_names;
sim_log.safety_state = SS(1:k);
sim_log.safety_reason = REASON(1:k);
sim_log.safety_source = SOURCE(1:k);
sim_log.safety_events = safety.events;
sim_log.nominal_u_body = NOM(1:k,:);
sim_log.candidate_u_body = CAND(1:k,:);
sim_log.candidate_yaw_rate = CYAW(1:k);
sim_log.processed_ranges = PROC(1:k,:);
sim_log.ranger_closing_rate = CLOSING(1:k,:);
sim_log.ranger_age = AGE(1:k);
sim_log.ranger_seq = SEQ(1:k);
sim_log.ranger_sample_time = SAMPLE(1:k);
sim_log.ranger_delivery_time = DELIVERY(1:k);
sim_log.ranger_sample_yaw = SYAW(1:k);
sim_log.ranger_sample_pos = SPOS(1:k,:);
sim_log.ranger_status = STATUS(1:k,:);
sim_log.ground_velocity = GV(1:k,:);
sim_log.primary_trip = PT(1:k); sim_log.backup_trip = BT(1:k);
sim_log.command_accepted = ACCEPT(1:k);
sim_log.primary_margin = PM(1:k); sim_log.backup_margin = BM(1:k);
sim_log.predicted_stop_distance = SD(1:k);
sim_log.required_clearance = REQUIRED(1:k,:);
sim_log.observed_clearance = AVAILABLE(1:k,:);
sim_log.safety_last_error = safety.last_error;
sim_log.obstacle_events = obstacle_events;
end

function yes = fault_active(faults,name,t)
yes = false;
if isfield(faults,name) && ~isempty(faults.(name))
    windows = faults.(name);
    yes = any(t >= windows(:,1) & t < windows(:,2));
end
end

function hit = touches(pos, edges, r)
% True if a disc of radius r at pos overlaps any wall segment. A step
% moves far less than r, so the disc meets an edge before its centre
% could get inside an obstacle.
A  = edges(:, 1:2);
AB = edges(:, 3:4) - A;
AP = pos' - A;
s  = min(max(sum(AP .* AB, 2) ./ sum(AB.^2, 2), 0), 1);
hit = min(vecnorm(AP - s .* AB, 2, 2)) < r;
end
