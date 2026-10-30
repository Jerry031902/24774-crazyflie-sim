function fig = plot_results(sim_log, env, p, m)
%PLOT_RESULTS  Map of the run coloured by mode, plus sensors, speed and clearance.
%   fig = PLOT_RESULTS(sim_log, env, p, m) with m from COMPUTE_METRICS.
%   The map shows each sweep's scan points, the pass point it chose, where
%   each AVOID episode started (hit) and where the drone left a wall. The
%   time plots shade AVOID in orange.
c   = viz_colors();
t   = sim_log.t;
pos = sim_log.x(:, 1:2);
ev  = sim_log.events;

fig = figure('Name', 'Drone run', 'Color', c.surface, 'Position', [60 60 1250 660]);
tl  = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% ---- Map -----------------------------------------------------------------
ax = nexttile(tl, 1, [3 2]);
plot_environment(ax, env);
h_route = plot_route(ax, sim_log);
h_sweep = plot_sweeps(ax, sim_log.sweeps, c);

[gx, gy] = mode_path(pos, sim_log.mode, 1);
[ax_, ay_] = mode_path(pos, sim_log.mode, 2);
h_go = plot(ax, gx, gy, '-', 'Color', c.go, 'LineWidth', 2, 'DisplayName', 'GO_TO_GOAL');
h_av = plot(ax, ax_, ay_, '-', 'Color', c.avoid, 'LineWidth', 2, 'DisplayName', 'AVOID');
h_ev = [mark_events(ax, ev, 'hit', 'v', c.avoid, 'Hit (AVOID starts)'), ...
        mark_events(ax, ev, 'leave', '^', c.go, 'Leave wall')];

if sim_log.collided
    plot(ax, pos(end,1), pos(end,2), 'x', 'Color', c.critical, 'MarkerSize', 14, ...
        'LineWidth', 2.5, 'HandleVisibility', 'off');
    text(ax, pos(end,1), pos(end,2), '  Collision', 'Color', c.critical, 'FontWeight', 'bold');
end
handles = [h_go h_av h_ev h_sweep h_route];
legend(ax, handles, 'Location', 'southoutside', 'NumColumns', 3, ...
    'Box', 'off', 'Interpreter', 'none');
title(ax, run_summary(m), 'FontWeight', 'normal', 'Color', c.ink);

% ---- Rangers, seen from the direction of travel --------------------------
ax = nexttile(tl, 3);
style_axes(ax);
shade_avoid(ax, t, sim_log.mode, c);
d = max(sim_log.dir, 1);
n = numel(t);
side = mod(d + 2 * (sim_log.hand < 0), 4) + 1;   % left of travel, or right when the wall is on the right
front = sim_log.ranges(sub2ind([n 4], (1:n)', d));
wall  = sim_log.ranges(sub2ind([n 4], (1:n)', side));
front(isinf(front)) = NaN;   % no return: leave a gap
wall(isinf(wall))   = NaN;
h_f = plot(ax, t, front, 'Color', c.front, 'LineWidth', 1.3);
h_l = plot(ax, t, wall, 'Color', c.left, 'LineWidth', 1.3);
yline(ax, p.nav.d_detect, '--', 'd_{detect}', 'Color', c.ink2, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'left');
yline(ax, p.nav.d_wall, ':', 'd_{wall}', 'Color', c.ink2, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom');
yline(ax, p.ranger.range_max, '-', 'max range', 'Color', c.muted, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'top');
ylim(ax, [0, 1.1 * p.ranger.range_max]);   % every reading fits; gaps mean no return
ylabel(ax, 'Range [m]');
legend(ax, [h_f h_l], {'front of travel', 'side of travel (wall side when following)'}, 'Location', 'northoutside', ...
    'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 8, 'AutoUpdate', 'off');
ax_rng = ax;

% ---- Speed ---------------------------------------------------------------
ax = nexttile(tl, 6);
style_axes(ax);
shade_avoid(ax, t, sim_log.mode, c);
plot(ax, t, vecnorm(sim_log.x(:, 3:4), 2, 2), 'Color', c.ink, 'LineWidth', 1.3);
yline(ax, p.nav.v_cruise, ':', 'v_{cruise}', 'Color', c.ink2, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'left');
yline(ax, p.nav.v_follow, ':', 'v_{follow}', 'Color', c.ink2, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'left');
ylim(ax, [0, 1.25 * p.nav.v_cruise]);
ylabel(ax, 'Speed [m/s]');
ax_spd = ax;

% ---- Clearance between the body and the nearest obstacle or wall ---------
ax = nexttile(tl, 9);
style_axes(ax);
shade_avoid(ax, t, sim_log.mode, c);
gap = obstacle_distance(env, pos) - p.radius;
plot(ax, t, gap, 'Color', c.ink, 'LineWidth', 1.3);
yline(ax, 0, '-', 'contact', 'Color', c.critical, 'FontSize', 8, 'LabelHorizontalAlignment', 'left');
yline(ax, p.near_miss, '--', 'near miss', 'Color', c.serious, 'FontSize', 8, ...
    'LabelHorizontalAlignment', 'left');
ylim(ax, [min(0, min(gap)) - 0.05, 1.5]);
ylabel(ax, 'Clearance [m]');
xlabel(ax, 'Time [s]');

linkaxes([ax_rng ax_spd ax], 'x');
xlim(ax, [0 t(end)]);
end

% -------------------------------------------------------------------------
function h = plot_sweeps(ax, sweeps, c)
% Scan points of every sweep, and the pass point each one chose.
h = gobjects(1, 0);
if isempty(sweeps)
    return
end
P = zeros(0, 2);
for s = sweeps
    ok = isfinite(s.rng);
    P = [P; s.pos + s.rng(ok)' .* [cos(s.ang(ok))' sin(s.ang(ok))']]; %#ok<AGROW>
end
h = plot(ax, P(:,1), P(:,2), '.', 'Color', c.muted, 'MarkerSize', 5, ...
    'DisplayName', 'Sweep scan (front + side rangers)');
pass = vertcat(sweeps.pass);
pass = pass(~any(isnan(pass), 2), :);
if ~isempty(pass)
    h(2) = plot(ax, pass(:,1), pass(:,2), 'd', 'MarkerSize', 7, 'LineWidth', 1.2, ...
        'MarkerEdgeColor', c.avoid, 'MarkerFaceColor', c.surface, 'DisplayName', 'Pass point');
end
end

function h = mark_events(ax, ev, type, marker, color, name)
h = gobjects(1, 0);
e = ev(strcmp({ev.type}, type));
if isempty(e)
    return
end
P = vertcat(e.pos);
h = plot(ax, P(:,1), P(:,2), marker, 'MarkerSize', 7, 'MarkerFaceColor', color, ...
    'MarkerEdgeColor', '#fcfcfb', 'DisplayName', name);
end

function shade_avoid(ax, t, mode, c)
% Orange band behind each stretch of AVOID.
in = mode(:) == 2;
starts = find(diff([false; in]) == 1);
stops  = find(diff([in; false]) == -1);
for i = 1:numel(starts)
    xregion(ax, t(starts(i)), t(stops(i)), 'FaceColor', c.avoid, 'FaceAlpha', 0.12, ...
        'EdgeColor', 'none');
end
end

function s = run_summary(m)
if m.collided
    outcome = 'Collided';
elseif m.arrived
    outcome = sprintf('Reached the goal in %.1f s', m.t_finish);
else
    outcome = 'Did not reach the goal';
end
s = sprintf('%s   |   %d AVOID episodes, %d sweeps   |   path x%.2f of straight lines   |   min clearance %.2f m', ...
    outcome, m.avoid_episodes, m.sweeps, m.path_ratio, m.min_clearance);
end
