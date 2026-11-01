function sim_log = run_simulation(start, waypoints, goal, env, p)
%RUN_SIMULATION  Fly from start through the waypoints to the goal with the reactive navigator.
%   sim_log = RUN_SIMULATION(start, waypoints, goal, env, p) starts at rest
%   at start ([x y]), visits the rows of waypoints (Nx2) in order, then
%   flies to goal ([x y]). Pass waypoints = [] to fly straight to the goal.
%   Every tick:
%     1. read the four rangers (held between updates, every p.ranger.period)
%     2. update the GO_TO_GOAL / AVOID state machine
%     3. apply its body-frame velocity and yaw-rate commands to the model
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
%   collided, t_collide.

if isempty(waypoints)
    waypoints = zeros(0, 2);
elseif size(waypoints, 2) ~= 2
    error('run_simulation:waypoints', ...
        'waypoints must be an N-by-2 list of [x y] points, or [] for none.');
end
start   = start(:)';
targets = [waypoints; goal(:)'];
validate_waypoints(start, targets, env, p);
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

x     = [start(:); 0; 0];
psi   = p.psi0;
nav   = navigator_init(start, targets);
gust  = [0; 0];
tau_g = 2.0;   % gust correlation time [s]
next_read = 0;
arrived  = false;  t_arrive  = NaN;
collided = false;  t_collide = NaN;

for k = 1:N
    if t(k) >= next_read - 1e-9
        ranges = read_rangers(x(1:2), psi, env, p);
        next_read = next_read + p.ranger.period;
    end
    [u_body, yaw_rate, nav] = navigator_step(nav, x, psi, ranges, t(k), p);
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

    if p.stop_on_collision && touches(x(1:2), env.edges, p.radius)
        collided = true;  t_collide = t(k);
        break
    end
    if strcmp(nav.mode, 'DONE') && norm(x(3:4)) < p.goal_vtol
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
