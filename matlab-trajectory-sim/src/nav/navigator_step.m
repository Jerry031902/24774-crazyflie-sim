function [u_body, yaw_rate, nav] = navigator_step(nav, x, psi, ranges, t, p)
%NAVIGATOR_STEP  GO_TO_GOAL / AVOID state machine using WedgeBug sweeps.
%   [u_body, yaw_rate, nav] = NAVIGATOR_STEP(nav, x, psi, ranges, t, p)
%   takes the state x = [px; py; vx; vy], the yaw psi and the ranger
%   readings [front; left; back; right], and returns the body-frame
%   velocity command [forward; left] and a yaw-rate command.
%
%   GO_TO_GOAL  Turn on the spot to face the current target, then fly
%               straight at it nose first. An obstacle ahead and short of
%               the target: slow down, stop p.nav.d_sweep from it and
%               switch to AVOID with a sweep. A side ranger reading below
%               p.nav.d_side (a wall closing in at a shallow angle):
%               switch to AVOID wall-following.
%   AVOID       WedgeBug:
%     sweep     Hover and yaw the front ranger across +/- p.nav.sweep_half
%               (45 deg) about the blocked heading, recording the range
%               at each angle: the wedge.
%     decide    On each side, walk out from the middle of the wedge to
%               where the obstacle's outline ends (the range jumps by more
%               than p.nav.edge_jump, or there is no return). A side costs
%               |drone -> edge| + |edge -> target|. The first sweep of an
%               episode picks the cheaper side and the drone keeps it, so
%               it cannot dither between the two.
%     pass      If that edge was inside the wedge, fly nose first to a
%               point p.nav.pass_margin outside it, then head for the
%               target again. Blocked on the way: sweep again from there.
%     follow    If the obstacle runs on past the wedge on that side,
%               follow it instead, wall on the right when going round the
%               left and vice versa, moving only along body axes, until the
%               drone is closer to the target than the hit point and the
%               way toward it is clear.
%   An episode lasts from first contact until the drone is closer to the
%   target than where that contact happened, so each new episode starts
%   closer than the last. Coming back to an earlier sweep spot, or to where
%   wall-following began, without getting closer means the target is
%   unreachable: it is recorded and skipped. Within p.nav.wp_tol of a
%   target, move on to the next; after the last the mode is DONE.
%
%   The drone brakes below p.nav.v_turn before changing direction and only
%   yaws while stopped (or slightly, flying nose first), so the rangers
%   always face the way it moves.

pos   = x(1:2);
speed = norm(x(3:4));
moved = norm(pos - nav.last_pos);
nav.last_pos = pos;
if nav.episode
    nav.ep_travel = nav.ep_travel + moved;
    % Track how far the drone has circled round the target. More than
    % p.nav.max_winding without getting closer means it is walled in.
    d = pos - nav.targets(nav.wp,:)';
    bearing = atan2(d(2), d(1));
    nav.winding = nav.winding + wrap_to_pi(bearing - nav.last_bearing);
    nav.last_bearing = bearing;
    if abs(nav.winding) > p.nav.max_winding
        nav = log_event(nav, t, 'unreachable', pos);
        nav.unreachable(end+1) = nav.wp;
        nav = next_target(nav);
    end
end

if ~strcmp(nav.mode, 'DONE') && norm(nav.targets(nav.wp,:)' - pos) < p.nav.wp_tol
    nav = log_event(nav, t, 'reached', pos);
    nav = next_target(nav);
end

% A mode or phase switch is handled within the same tick: the new one then
% computes the command.
cmd = command(nav.exec_dir, 0, 0);
for pass_no = 1:4
    if strcmp(nav.mode, 'DONE')
        u_body = [0; 0];
        yaw_rate = 0;
        return
    end
    if strcmp(nav.mode, 'GO_TO_GOAL')
        [cmd, nav, switched] = go_to_goal(nav, pos, psi, speed, ranges, t, p);
    else
        [cmd, nav, switched] = avoid(nav, pos, psi, speed, ranges, moved, t, p);
    end
    if ~switched
        break
    end
end

% Fly the command, braking first if it changes direction.
if cmd.dir == 0
    u_body = cmd.u;
    yaw_rate = cmd.yaw_rate;
    nav.exec_dir = 0;
    return
end
if cmd.dir ~= nav.exec_dir
    if speed > p.nav.v_turn
        u_body = [0; 0];
        yaw_rate = 0;
        return
    end
    nav.exec_dir = cmd.dir;
end
D = dirs();
u_body = cmd.speed * D(cmd.dir, :)';
yaw_rate = cmd.yaw_rate;
end

% =========================================================================
function [cmd, nav, switched] = go_to_goal(nav, pos, psi, speed, ranges, t, p)
switched = false;
target = nav.targets(nav.wp,:)';
if nav.episode && norm(target - pos) < nav.hit_dist - p.nav.leave_margin
    nav = end_episode(nav);   % past the obstacle: the next one is a fresh start
end

[cmd, nav, event] = fly_to(nav, pos, psi, speed, ranges, target, p.nav.d_sweep, p);
switch event
    case 'blocked'
        nav = start_avoid(nav, pos, t);
        nav = start_sweep(nav, psi);
        switched = true;
    case 'side_left'
        nav = start_avoid(nav, pos, t);
        nav = start_follow(nav, psi, +1);
        switched = true;
    case 'side_right'
        nav = start_avoid(nav, pos, t);
        nav = start_follow(nav, psi, -1);
        switched = true;
end
end

function [cmd, nav, event] = fly_to(nav, pos, psi, speed, ranges, point, d_block, p)
% Face point, then fly straight at it nose first. event reports what
% stopped the drone: 'blocked' (an obstacle ahead and short of the point,
% now within d_block, where the drone comes to rest), 'side_left' /
% 'side_right' (a side ranger below d_side), or '' (nothing).
event = '';
d = point - pos;
dist = norm(d);

if dist <= p.nav.yaw_free_radius
    % The last few centimetres: slide straight onto the point, no turning.
    cmd = command(0, 0, 0);
    cmd.u = min(p.nav.v_cruise, p.nav.k_stop * dist) * (rot2d(psi)' * d) / dist;
    return
end

err = wrap_to_pi(atan2(d(2), d(1)) - psi);
if abs(err) > p.nav.yaw_tol
    nav.facing = false;
elseif abs(err) < deg2rad(2)
    nav.facing = true;
end
if ~nav.facing
    % Not facing the point: stop, then turn on the spot until lined up.
    cmd = command(1, 0, (speed < p.nav.v_turn) * yaw_command(err, p));
    return
end

front = ranges(1);
short_of_point = front - p.nav.d_wall < dist - 0.02;
if short_of_point && front <= d_block + 0.02
    event = 'blocked';
    cmd = command(1, 0, 0);
    return
end
if ranges(2) < p.nav.d_side
    event = 'side_left';
    cmd = command(1, 0, 0);
    return
end
if ranges(4) < p.nav.d_side
    event = 'side_right';
    cmd = command(1, 0, 0);
    return
end

stop_at = p.nav.d_wall;
if short_of_point
    stop_at = d_block;         % come to rest where the drone will stop and look
end
speed_cmd = min([p.nav.v_cruise, p.nav.k_stop * dist, p.nav.k_stop * max(front - stop_at, 0)]);
cmd = command(1, speed_cmd, yaw_command(err, p));   % keep the nose on the point
end

% =========================================================================
function [cmd, nav, switched] = avoid(nav, pos, psi, speed, ranges, moved, t, p)
switch nav.phase
    case 'sweep'
        [cmd, nav, switched] = sweep(nav, pos, psi, speed, ranges, t, p);
    case 'pass'
        [cmd, nav, switched] = pass_edge(nav, pos, psi, speed, ranges, t, p);
    otherwise
        [cmd, nav, switched] = follow(nav, pos, psi, speed, ranges, moved, t, p);
end
end

% ----- sweep: hover and scan the wedge -------------------------------------
function [cmd, nav, switched] = sweep(nav, pos, psi, speed, ranges, t, p)
switched = false;
half = p.nav.sweep_half;
cmd = command(nav.exec_dir, 0, 0);

switch nav.sweep_stage
    case 'stop'
        if speed >= p.nav.v_turn
            return                          % hover first
        end
        % Back at an earlier sweep spot of this episode without having got
        % closer, or swept too often: the target cannot be reached.
        if ~isempty(nav.sweep_spots)
            again = vecnorm(nav.sweep_spots(:,1:2) - pos', 2, 2) < 2 * p.nav.loop_tol & ...
                    nav.ep_travel - nav.sweep_spots(:,3) > p.nav.loop_min_travel;
            if any(again) || size(nav.sweep_spots, 1) >= p.nav.max_sweeps
                [nav, switched] = give_up(nav, pos, t);
                return
            end
        end
        nav.sweep_stage = 'to_start';

    case 'to_start'                         % turn to the right end of the wedge
        err = wrap_to_pi(nav.sweep_center - half - psi);
        cmd = command(nav.exec_dir, 0, yaw_command(err, p));
        if abs(err) < deg2rad(1)
            nav.sweep_stage = 'scan';
            nav.scan = struct('t', [], 'ang', [], 'rng', []);
        end

    case 'scan'
        % Sweep to the left end. Log the front, left and right rangers: the
        % sides sit 90 deg off the nose, so this +/-45 deg sweep sees
        % -135..+135 deg around the blocked heading.
        nav.scan.t(end+1:end+3)   = t;
        nav.scan.ang(end+1:end+3) = psi + [0, pi/2, -pi/2];
        nav.scan.rng(end+1:end+3) = ranges([1 2 4])';
        if wrap_to_pi(psi - (nav.sweep_center - half)) < 2 * half - 1e-3
            cmd = command(nav.exec_dir, 0, p.nav.sweep_rate);
            return
        end
        [nav, switched] = decide(nav, pos, t, p);
end
end

function [nav, switched] = decide(nav, pos, t, p)
% Turn the wedge into a way round the obstacle.
switched = true;
target = nav.targets(nav.wp,:)';
w = read_wedge(nav.scan, pos, target, nav.sweep_center, p);

if ~w.blocked
    nav.mode = 'GO_TO_GOAL';                % nothing in the way after all
    nav.facing = false;
    return
end
if nav.side == 0
    [~, i] = min(w.cost);
    nav.side = 3 - 2 * i;                   % +1 = round the left, -1 = round the right
end
i = (3 - nav.side) / 2;                     % row of w.edge for that side

pass_pt = [NaN; NaN];
if w.seen(i)
    pass_pt = choose_pass_point(w.edge(i,:)', pos, target, nav.side, w.outline, nav.scan, p);
end
nav.sweeps(end+1) = struct('t', nav.scan.t, 'ang', nav.scan.ang, 'rng', nav.scan.rng, ...
    'pos', pos', 'center', nav.sweep_center, 'edge', w.edge, 'seen', w.seen, ...
    'cost', w.cost, 'side', nav.side, 'pass', pass_pt');
nav.sweep_spots(end+1,:) = [pos' nav.ep_travel];

if w.seen(i)
    nav.subgoal = pass_pt;
    nav.phase = 'pass';
    nav.facing = false;
else
    % Round the left = wall on the right. Square up from the heading the
    % sweep looked along, not from where the sweep ended.
    nav = start_follow(nav, nav.sweep_center, -nav.side);
end
end

function P = choose_pass_point(E, pos, target, side, outline, scan, p)
% Where to fly to get round the edge E. Start pass_margin out from it,
% square to the line of sight and away from the obstacle, then keep moving
% out until the next leg, from the pass point straight to the target,
% stays tangent_clearance from the obstacle outline the sweep saw. Only
% move out through space the sweep saw as empty.
ray = (E - pos) / norm(E - pos);
out = side * [-ray(2); ray(1)];
P = E + p.nav.pass_margin * out;
for s = p.nav.pass_margin:0.05:p.nav.pass_max
    Q = E + s * out;
    if ~seen_free(Q, pos, scan, p)
        break                                % beyond what the sweep can vouch for
    end
    P = Q;
    if min(segment_distance(outline, P, target)) >= p.nav.tangent_clearance
        break                                % the next leg clears the obstacle
    end
end
end

function ok = seen_free(Q, pos, scan, p)
% A reading r means nothing lies within r anywhere in that ranger's cone,
% so Q, and the straight line from the sweep spot to it, is clear if some
% reading whose cone covers Q's direction reaches d_wall past it.
v  = Q - pos;
on = abs(wrap_to_pi(atan2(v(2), v(1)) - scan.ang)) <= p.ranger.fov / 2;
r  = min(scan.rng(on), p.ranger.range_max);
ok = any(norm(v) <= r - p.nav.d_wall);
end

function d = segment_distance(pts, a, b)
% Distance from each row of pts to the segment a-b.
ab = (b - a)';
s  = min(max(((pts - a') * ab') / (ab * ab'), 0), 1);
d  = vecnorm(pts - (a' + s .* ab), 2, 2);
end

function w = read_wedge(scan, pos, target, center, p)
% Edges of the obstacle in a sweep, one per side (row 1 left, row 2 right).
% Walk out from the middle of the scan; an edge is the last point of the
% outline before the range jumps up by more than edge_jump or returns
% nothing. If the outline runs to the end of the scan, the edge is unseen
% and its last point stands in for it.
%
% A ranger reports the nearest hit anywhere in its cone, so while the
% outline continues the reading comes from the cone ray nearest the
% middle. Points are placed on that ray, half a cone inward, which puts
% an edge on the obstacle's corner rather than half a cone past it.
%
% Also returned: w.outline, the obstacle's points between its two edges.
[rel, order] = sort(wrap_to_pi(scan.ang - center));
r = scan.rng(order);
core = find(abs(rel) <= deg2rad(10));
[r_core, k] = min(r(core));
w.blocked = ~isempty(core) && r_core <= p.nav.d_sweep + 0.3;
w.edge    = nan(2, 2);
w.seen    = false(1, 2);
w.cost    = inf(1, 2);
w.outline = zeros(0, 2);
if ~w.blocked
    return
end
c = core(k);
h = p.ranger.fov / 2;
ends = [c c];
for i = 1:2
    step = 3 - 2 * i;                       % +1 walks left (increasing angle), -1 right
    j = c;
    while true
        jn = j + step;
        if jn < 1 || jn > numel(r)
            break                           % reached the end of the scan
        end
        if isinf(r(jn)) || r(jn) - r(j) > p.nav.edge_jump
            w.seen(i) = true;
            break
        end
        j = jn;
    end
    ends(i) = j;
    th = center + rel(j) - step * h;
    E = pos + r(j) * [cos(th); sin(th)];
    w.edge(i,:) = E';
    w.cost(i) = norm(E - pos) + norm(target - E);
end

idx = (1:numel(r))';
on  = idx >= ends(2) & idx <= ends(1) & isfinite(r(:));
th  = center + reshape(rel(on), [], 1) - sign(idx(on) - c) * h;   % half a cone inward, as for the edges
w.outline = pos' + reshape(r(on), [], 1) .* [cos(th) sin(th)];
end

% ----- pass: fly to the point outside the chosen edge -----------------------
function [cmd, nav, switched] = pass_edge(nav, pos, psi, speed, ranges, t, p)
switched = false;
if norm(nav.subgoal - pos) < p.nav.wp_tol + 0.05
    nav.mode = 'GO_TO_GOAL';                % round the edge: head for the target
    nav.subgoal = [NaN; NaN];
    nav.facing = false;
    cmd = command(nav.exec_dir, 0, 0);
    switched = true;
    return
end
% Only something close ahead counts: the corner being rounded sits just
% beside this path and shows in the front cone from further out.
[cmd, nav, event] = fly_to(nav, pos, psi, speed, ranges, nav.subgoal, p.nav.d_detect, p);
switch event
    case 'blocked'
        nav = start_sweep(nav, psi);        % more of the obstacle: look again from here
        switched = true;
    case 'side_left'
        nav = start_follow(nav, psi, +1);
        switched = true;
    case 'side_right'
        nav = start_follow(nav, psi, -1);
        switched = true;
end
end

% ----- follow: wall-following with the wall on side 'hand' -----------------
function [cmd, nav, switched] = follow(nav, pos, psi, speed, ranges, moved, t, p)
switched = false;
hold_rate = yaw_command(wrap_to_pi(nav.psi_hold - psi), p);

if strcmp(nav.phase, 'align')
    squared = abs(wrap_to_pi(nav.psi_hold - psi)) < deg2rad(0.5);
    if ~(squared && speed < p.nav.v_turn)
        % Stop, then turn on the spot to the nearest room axis.
        cmd = command(nav.exec_dir, 0, (speed < p.nav.v_turn) * hold_rate);
        return
    end
    % Head for the obstacle along whichever of front, left, right sees it
    % nearest; the following rules take it from there.
    sides = [1 2 4];
    [r_min, i] = min(ranges(sides));
    nav.travel_dir = sides(i);
    if r_min > p.nav.d_detect + p.nav.corner_margin
        nav.travel_dir = 1;
    end
    nav.phase = 'follow';
    nav.since_turn = 0;
end

k = nav.travel_dir;
flying = nav.exec_dir == k;               % false while braking for a turn
if flying
    nav.since_turn = nav.since_turn + moved;
    if nav.following
        nav.travel = nav.travel + moved;
    end
end

% Exit: closer to the target than the hit point, and (TangentBug's rule)
% the free space seen toward the target reaches closer to it than any
% point of the wall followed so far. The second part keeps the drone from
% leaving inside a pocket whose back wall stands between it and the target.
target = nav.targets(nav.wp, :)';
rel = rot2d(psi)' * (target - pos);
D = dirs();
for s = unique([k, toward(k, nav.hand)])     % front and wall-side readings are wall points
    if isfinite(ranges(s))
        wall_pt = pos + rot2d(psi) * D(s,:)' * ranges(s);
        nav.d_followed = min(nav.d_followed, norm(target - wall_pt));
    end
end
reach = max(norm(rel) - free_toward(rel, ranges, p), 0);
if nav.following && norm(rel) < nav.hit_dist - p.nav.leave_margin && ...
        reach < nav.d_followed - p.nav.leave_margin
    nav = log_event(nav, t, 'leave', pos);
    nav = end_episode(nav);
    nav.mode = 'GO_TO_GOAL';
    nav.facing = false;
    cmd = command(k, 0, 0);
    switched = true;
    return
end

% Full loop around the obstacle without an exit: the target is unreachable
if nav.following && nav.travel > p.nav.loop_min_travel && norm(pos - nav.loop_point) < p.nav.loop_tol
    [nav, switched] = give_up(nav, pos, t);
    cmd = command(k, 0, 0);
    return
end

% Nothing blocked the way in after squaring up: a false alarm, carry on.
if ~nav.following && flying && nav.since_turn > p.nav.d_detect + p.nav.corner_margin
    nav.mode = 'GO_TO_GOAL';
    nav.facing = false;
    cmd = command(k, 0, 0);
    switched = true;
    return
end

% Wall-following rules, only while actually flying the travel direction
% (not while braking for a turn just decided).
front = ranges(k);
wall  = ranges(toward(k, nav.hand));
if flying
    if wall <= p.nav.d_open
        nav.wall_seen = true;
    end
    if front <= p.nav.d_wall + 0.02 || wall < p.nav.d_close
        nav = turn(nav, away(k, nav.hand), pos, true);     % blocked ahead, or too close to the wall
    elseif strcmp(nav.phase, 'follow')
        if nav.wall_seen && wall > p.nav.d_open
            nav.phase = 'overshoot';                        % the wall has ended
            nav.overshoot_left = p.nav.corner_margin;
        elseif nav.following && ~nav.wall_seen && ...
                nav.since_turn > 2 * (p.nav.d_wall + p.nav.corner_margin)
            nav = turn(nav, toward(k, nav.hand), pos, false);  % no wall came back: wrap again
        end
    else
        nav.overshoot_left = nav.overshoot_left - moved;
        if nav.overshoot_left <= 0.02
            nav = turn(nav, toward(k, nav.hand), pos, false);  % clear of the corner: turn back toward the wall
        end
    end
end

k = nav.travel_dir;
speed_cmd = min(p.nav.v_follow, p.nav.k_stop * max(ranges(k) - p.nav.d_wall, 0));
if strcmp(nav.phase, 'overshoot')
    speed_cmd = min(speed_cmd, p.nav.k_stop * max(nav.overshoot_left, 0));
end
cmd = command(k, speed_cmd, hold_rate);
end

function free = free_toward(rel, ranges, p)
% How far the drone can see it could fly toward the target (rel, body
% frame) before keeping d_wall from an obstacle. A ranger only reports the
% nearest hit somewhere in its cone, so take the cone ray closest to the
% target direction, at angle a: a wall square to that ray at range r is
% r/cos(a) away along the target direction. Rangers whose whole cone faces
% away are ignored; no return counts as free out to the ranger's range.
dist = norm(rel);
ang  = acos(min(max(dirs() * (rel / dist), -1), 1));   % sensor axis to target direction
a    = max(ang - p.ranger.fov / 2, 0);
look = a < deg2rad(80);
r    = min(ranges(look), p.ranger.range_max);
free = max(min(r ./ cos(a(look))) - p.nav.d_wall, 0);
if all(isinf(ranges(look)))
    free = inf;
end
end

% =========================================================================
function nav = start_avoid(nav, pos, t)
nav.mode = 'AVOID';
if ~nav.episode                      % first contact: this is the hit point
    nav.episode     = true;
    nav.hit_point   = pos;
    nav.hit_dist    = norm(nav.targets(nav.wp,:)' - pos);
    nav.side        = 0;
    nav.ep_travel   = 0;
    nav.sweep_spots = zeros(0, 3);
    d = pos - nav.targets(nav.wp,:)';
    nav.last_bearing = atan2(d(2), d(1));
    nav.winding     = 0;
    nav = log_event(nav, t, 'hit', pos);
end
end

function nav = start_sweep(nav, psi)
nav.phase        = 'sweep';
nav.sweep_stage  = 'stop';
nav.sweep_center = psi;
end

function nav = start_follow(nav, psi, hand)
nav.phase      = 'align';
nav.hand       = hand;
nav.psi_hold   = round(psi / (pi/2)) * (pi/2);   % nearest room axis
nav.d_followed = inf;
nav.following  = false;
nav.wall_seen  = false;
nav.since_turn = 0;
nav.travel     = 0;
end

function nav = turn(nav, new_dir, pos, wall_seen)
nav.travel_dir = new_dir;
nav.phase      = 'follow';
nav.wall_seen  = wall_seen;
nav.since_turn = 0;
if ~nav.following                     % first turn: the wall is now beside the drone
    nav.following  = true;
    nav.loop_point = pos;
    nav.travel     = 0;
end
end

function [nav, switched] = give_up(nav, pos, t)
nav = log_event(nav, t, 'unreachable', pos);
nav.unreachable(end+1) = nav.wp;
nav = next_target(nav);
switched = true;
end

function nav = end_episode(nav)
nav.episode   = false;
nav.side      = 0;
nav.subgoal   = [NaN; NaN];
nav.hit_point = [NaN; NaN];
nav.hit_dist  = Inf;
end

function nav = next_target(nav)
nav = end_episode(nav);
nav.following = false;
nav.facing    = false;
nav.phase     = '';
if nav.wp < size(nav.targets, 1)
    nav.wp   = nav.wp + 1;
    nav.mode = 'GO_TO_GOAL';
else
    nav.mode = 'DONE';
end
end

function nav = log_event(nav, t, type, pos)
nav.events(end+1) = struct('t', t, 'type', type, 'pos', pos', 'wp', nav.wp);
end

% =========================================================================
function cmd = command(dir, speed, yaw_rate)
cmd = struct('dir', dir, 'speed', speed, 'yaw_rate', yaw_rate, 'u', [0; 0]);
end

function r = yaw_command(err, p)
r = min(max(p.nav.k_yaw * err, -p.yaw_rate_max), p.yaw_rate_max);
end

function D = dirs()
D = [1 0; 0 1; -1 0; 0 -1];   % forward, left, back, right
end

function d = toward(d, hand)
% Direction of the wall side: left for hand = +1, right for hand = -1.
d = mod(d + (hand < 0) * 2, 4) + 1;
end

function d = away(d, hand)
% Direction away from the wall: right for hand = +1, left for hand = -1.
d = mod(d + (hand > 0) * 2, 4) + 1;
end
