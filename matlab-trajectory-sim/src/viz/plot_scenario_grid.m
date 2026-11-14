function fig = plot_scenario_grid(results, opts)
%PLOT_SCENARIO_GRID  Arena maps of several runs side by side, path coloured by mode.
%   fig = PLOT_SCENARIO_GRID(results) draws one map per run, three to a row.
%   results is a struct array with fields scenario (from ARENA_SCENARIOS),
%   env, p, log (from RUN_SIMULATION) and m (from COMPUTE_METRICS), as
%   DEMO_ARENA_GRID builds it.
%
%   Each map shows the direct line from start (square) to goal (star),
%   dashed, and the path flown: blue in GO_TO_GOAL, orange in AVOID. A triangle marks every mode
%   change (down: AVOID starts, up: back to GO_TO_GOAL) and a diamond each
%   pass point a sweep chose. The title gives PASS, COLLISION, GAVE UP (the
%   goal was found unreachable) or TIMEOUT, the closest body-to-obstacle
%   gap, the finish time and the time spent in AVOID.
%
%   fig = PLOT_SCENARIO_GRID(results, opts) with optional fields
%     columns   maps per row (default 3)
%     title     figure title (default: the settings in results(1).p)

if nargin < 2, opts = struct(); end
c    = viz_colors();
n    = numel(results);
cols = min(get_opt(opts, 'columns', 3), n);
rows = ceil(n / cols);

fig = figure('Name', 'Scenario grid', 'Color', c.surface, ...
             'Position', [40 40 480*cols 470*rows + 70]);
tl  = tiledlayout(fig, rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, get_opt(opts, 'title', settings_title(results(1).p)), 'FontSize', 12, 'Color', c.ink);

for k = 1:n
    r   = results(k);
    ax  = nexttile(tl);
    pos = r.log.x(:, 1:2);
    mode = r.log.mode(:);

    s = r.scenario.start;
    g = r.scenario.goal;

    plot_environment(ax, r.env);
    label_obstacles(ax, r.env, r.scenario.labels, c);
    h_line = plot(ax, [s(1) g(1)], [s(2) g(2)], '--', 'Color', c.muted, 'LineWidth', 0.9, ...
                  'DisplayName', 'Direct line');

    [gx, gy] = mode_path(pos, mode, 1);
    [vx, vy] = mode_path(pos, mode, 2);
    h_go = plot(ax, gx, gy, '-', 'Color', c.go, 'LineWidth', 2.4, 'DisplayName', 'GO_TO_GOAL');
    h_av = plot(ax, vx, vy, '-', 'Color', c.avoid, 'LineWidth', 2.4, 'DisplayName', 'AVOID');

    to_avoid = find(mode(1:end-1) ~= 2 & mode(2:end) == 2) + 1;   % first AVOID sample
    to_go    = find(mode(1:end-1) == 2 & mode(2:end) == 1) + 1;   % first GO_TO_GOAL sample after it
    h_in  = mark(ax, pos(to_avoid, :), 'v', c.avoid, c.surface, 'AVOID starts');
    h_out = mark(ax, pos(to_go, :), '^', c.go, c.surface, 'Back to GO_TO_GOAL');
    pass  = zeros(0, 2);
    if ~isempty(r.log.sweeps)
        pass = vertcat(r.log.sweeps.pass);
        pass = pass(all(isfinite(pass), 2), :);
    end
    h_pass = mark(ax, pass, 'd', c.surface, c.avoid, 'Pass point');
    h_start = plot(ax, s(1), s(2), 's', 'MarkerSize', 10, 'MarkerFaceColor', c.ink, ...
                   'MarkerEdgeColor', c.surface, 'DisplayName', 'Start');
    h_goal  = plot(ax, g(1), g(2), 'p', 'MarkerSize', 16, 'MarkerFaceColor', c.ink, ...
                   'MarkerEdgeColor', c.surface, 'DisplayName', 'Goal');

    if r.m.collided
        plot(ax, pos(end,1), pos(end,2), 'x', 'Color', c.critical, 'MarkerSize', 14, ...
            'LineWidth', 2.5, 'HandleVisibility', 'off');
    end

    [status, color] = run_status(r.m);
    t_end = r.log.t(end);
    if r.m.arrived, t_end = r.m.t_finish; end
    title(ax, {sprintf('%s: %s', r.scenario.name, status), ...
               sprintf('min clearance %.3f m  |  %.1f s, %.1f s in AVOID', ...
                       r.m.min_clearance, t_end, r.m.avoid_time)}, ...
          'FontWeight', 'normal', 'FontSize', 10, 'Color', color, 'Interpreter', 'none');

    if k == 1
        legend_handles = [h_go h_av h_in h_out h_pass h_start h_goal h_line];
        legend_axes = ax;
    end
end

lgd = legend(legend_axes, legend_handles, 'Orientation', 'horizontal', 'Box', 'off', ...
             'Interpreter', 'none', 'FontSize', 10);
lgd.Layout.Tile = 'south';
end

% -------------------------------------------------------------------------
function h = mark(ax, P, marker, face, edge, name)
% Markers at the rows of P. An empty P still makes a handle for the legend.
if isempty(P), P = [NaN NaN]; end
h = plot(ax, P(:,1), P(:,2), marker, 'LineStyle', 'none', 'MarkerSize', 8, 'LineWidth', 1.2, ...
         'MarkerFaceColor', face, 'MarkerEdgeColor', edge, 'DisplayName', name);
end

function label_obstacles(ax, env, labels, c)
for i = 1:min(numel(labels), numel(env.obstacles))
    if ~isempty(labels{i})
        V = env.obstacles(i).vertices;
        text(ax, max(V(:,1)), max(V(:,2)), [' ' labels{i}], 'Color', c.ink2, ...
             'FontSize', 8, 'VerticalAlignment', 'bottom');
    end
end
end

function [status, color] = run_status(m)
c = viz_colors();
color = c.critical;
if m.collided
    status = 'COLLISION';
elseif m.arrived
    status = 'PASS';
    color = c.ink;
elseif ~isempty(m.unreachable)
    status = 'GAVE UP';
else
    status = 'TIMEOUT';
end
end

function s = settings_title(p)
safety = 'safety OFF';
if isfield(p, 'safety') && p.safety.enabled, safety = 'safety ON'; end
noise = 'noise OFF';
if p.ranger.noise > 0 || p.ranger.noise_rel > 0
    noise = sprintf('noise %.0f cm + %.0f%% of range', 100 * p.ranger.noise, 100 * p.ranger.noise_rel);
end
s = sprintf('MATLAB WedgeBug  |  kinematic model  |  %s  |  ground truth  |  %s', safety, noise);
end

function v = get_opt(opts, name, default)
v = default;
if isfield(opts, name) && ~isempty(opts.(name)), v = opts.(name); end
end
