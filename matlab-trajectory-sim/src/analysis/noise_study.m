function results = noise_study(start, waypoints, goal, obstacles, p, sigmas, n_runs)
%NOISE_STUDY  Fly one mission many times with noisy rangers and summarise.
%   results = NOISE_STUDY(start, waypoints, goal, obstacles, p) flies the
%   mission n_runs times (default 10) at each ranger noise level in sigmas
%   (default [0 0.01 0.02 0.05 0.1 0.2] m, the std dev of p.ranger.noise),
%   each run with its own random seed. It prints a summary per level, plots
%   the results against the noise level, and returns one table row per run.
%   p.ranger.noise_rel and every other setting are used as given in p.
%
%   NOISE_STUDY(..., sigmas, n_runs) sets the noise levels and the number
%   of runs per level.
%
%   A run succeeds when it reaches every waypoint and the goal without
%   touching anything. A missed target is one given up on as unreachable,
%   or not reached before p.t_max.

if nargin < 6 || isempty(sigmas), sigmas = [0 0.01 0.02 0.05 0.1 0.2]; end
if nargin < 7 || isempty(n_runs), n_runs = 10; end

env  = make_environment(p, obstacles);
quiet = warning('off', 'run_simulation:unreachable');
restore = onCleanup(@() warning(quiet));

rows = [];
for s = sigmas
    for k = 1:n_runs
        q = p;
        q.ranger.noise = s;
        q.seed = k;
        sim_log = run_simulation(start, waypoints, goal, env, q);
        m = compute_metrics(sim_log, env, q);
        missed = m.n_targets - m.targets_reached;
        rows = [rows; struct('noise', s, 'seed', k, ...
            'success', m.arrived && ~m.collided && missed == 0, ...
            'collided', m.collided, 'missed_targets', missed, ...
            't_finish', m.t_finish, 'min_clearance', m.min_clearance, ...
            'near_misses', m.near_misses, 'path_ratio', m.path_ratio, ...
            'sweeps', m.sweeps)]; %#ok<AGROW>
    end
end
results = struct2table(rows);

print_summary(results, sigmas, n_runs);
plot_study(results, sigmas, p);
end

% -------------------------------------------------------------------------
function print_summary(R, sigmas, n_runs)
fprintf('\n  Ranger noise study: %d runs per level\n', n_runs);
fprintf('    %9s  %8s  %10s  %13s  %15s  %10s  %10s  %7s\n', 'noise', 'success', 'collisions', 'missed target', 'min clearance', 'time', 'path', 'sweeps');
fprintf('    %9s  %8s  %10s  %13s  %15s  %10s  %10s  %7s\n', '(std dev)', '', '', '(runs)', '(worst run)', '(median)', '(median)', '(median)');
for s = sigmas
    r = R(R.noise == s, :);
    ok = r.success;
    fprintf('    %6.0f cm  %7.0f%%  %10d  %13d  %13.2f m  %8.1f s  %9.2fx  %7.0f\n', 100 * s, ...
        100 * mean(ok), nnz(r.collided), nnz(r.missed_targets > 0), min(r.min_clearance), ...
        median(r.t_finish(ok)), median(r.path_ratio(ok)), median(r.sweeps));
end
fprintf('\n');
end

function plot_study(R, sigmas, p)
c = viz_colors();
x = arrayfun(@(s) find(sigmas == s), R.noise);           % position of each run's level
jit = 0.12 * (mod(R.seed, 5) - 2) / 2;                   % spread runs sideways a little
labels = arrayfun(@(s) sprintf('%g', 100 * s), sigmas, 'UniformOutput', false);
lev = 1:numel(sigmas);

fig = figure('Name', 'Ranger noise study', 'Color', c.surface, 'Position', [100 100 1100 640]);
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Same mission, %d runs per noise level (each a different seed)', ...
    max(R.seed)), 'FontSize', 10, 'Color', c.ink);

% Outcomes
ax = nexttile(tl);
style_axes(ax);
succ = arrayfun(@(s) 100 * mean(R.success(R.noise == s)), sigmas);
coll = arrayfun(@(s) 100 * mean(R.collided(R.noise == s)), sigmas);
miss = arrayfun(@(s) 100 * mean(R.missed_targets(R.noise == s) > 0), sigmas);
plot(ax, lev, succ, '-o', 'Color', c.go, 'LineWidth', 2, 'MarkerFaceColor', c.go, 'MarkerSize', 6);
plot(ax, lev, miss, '-s', 'Color', c.avoid, 'LineWidth', 2, 'MarkerFaceColor', c.avoid, 'MarkerSize', 6);
plot(ax, lev, coll, '-^', 'Color', c.critical, 'LineWidth', 2, 'MarkerFaceColor', c.critical, 'MarkerSize', 6);
ylim(ax, [-5 105]);
ylabel(ax, 'Runs [%]');
legend(ax, {'succeeded', 'missed a target', 'collided'}, 'Location', 'west', 'Box', 'off', 'FontSize', 8);
title(ax, 'Outcome', 'FontWeight', 'normal');
finish_x(ax, lev, labels);

% Closest approach
ax = nexttile(tl);
style_axes(ax);
scatter(ax, x + jit, R.min_clearance, 18, 'filled', 'MarkerFaceColor', c.muted, 'MarkerFaceAlpha', 0.7);
plot(ax, lev, arrayfun(@(s) median(R.min_clearance(R.noise == s)), sigmas), '-', 'Color', c.go, 'LineWidth', 2);
yline(ax, p.near_miss, '--', 'near miss', 'Color', c.serious, 'FontSize', 8, 'LabelHorizontalAlignment', 'left');
yline(ax, 0, '-', 'contact', 'Color', c.critical, 'FontSize', 8, 'LabelHorizontalAlignment', 'left');
ylabel(ax, 'Closest body gap [m]');
title(ax, 'Closest approach to an obstacle (dots: runs, line: median)', 'FontWeight', 'normal');
finish_x(ax, lev, labels);

% Finish time, successful runs
ax = nexttile(tl);
style_axes(ax);
ok = R.success;
scatter(ax, x(ok) + jit(ok), R.t_finish(ok), 18, 'filled', 'MarkerFaceColor', c.muted, 'MarkerFaceAlpha', 0.7);
med = arrayfun(@(s) median(R.t_finish(ok & R.noise == s)), sigmas);
plot(ax, lev, med, '-', 'Color', c.go, 'LineWidth', 2);
ylabel(ax, 'Time to goal [s]');
title(ax, 'Time to goal, successful runs', 'FontWeight', 'normal');
finish_x(ax, lev, labels);

% Sweeps
ax = nexttile(tl);
style_axes(ax);
scatter(ax, x + jit, R.sweeps, 18, 'filled', 'MarkerFaceColor', c.muted, 'MarkerFaceAlpha', 0.7);
plot(ax, lev, arrayfun(@(s) median(R.sweeps(R.noise == s)), sigmas), '-', 'Color', c.go, 'LineWidth', 2);
ylabel(ax, 'Sweeps per run');
title(ax, 'Sweeps needed', 'FontWeight', 'normal');
finish_x(ax, lev, labels);
end

function finish_x(ax, lev, labels)
xlim(ax, [0.5, numel(lev) + 0.5]);
xticks(ax, lev);
xticklabels(ax, labels);
xlabel(ax, 'Ranger noise, std dev [cm]');
end
