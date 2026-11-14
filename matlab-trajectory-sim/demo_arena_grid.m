%% demo_arena_grid.m - start and goal only, with obstacles: one arena map per scenario
%
%   Flies each scenario from ARENA_SCENARIOS (no waypoints) and draws the
%   maps side by side, with the path blue in GO_TO_GOAL and orange in AVOID.
%   The first row is the Isaac Sim arena (sim/build_scene.py) along its
%   center, lower and upper lanes; the second adds a long wall, a U-shaped
%   cup and a diagonal through a row of pillars. Scenarios are defined in
%   src/env/arena_scenarios.m.

clear; clc; close all;
setup_paths();
p = default_params();
p.safety.enabled = false;   % as in the Isaac Sim comparison runs; true = supervised
p.ranger.noise   = 0;       % Gaussian ranger noise, std dev [m]
p.t_max          = 120;     % per run [s]

%% Scenarios: all of them, or by name, e.g. arena_scenarios({'center', 'lower', 'upper'})
scenarios = arena_scenarios();

%% Fly them
for k = numel(scenarios):-1:1
    sc = scenarios(k);
    pk = p;
    pk.map.xlim = sc.xlim;
    pk.map.ylim = sc.ylim;
    env = make_environment(pk, sc.obstacles);
    sim_log = run_simulation(sc.start, [], sc.goal, env, pk);
    results(k) = struct('scenario', sc, 'env', env, 'p', pk, 'log', sim_log, ...
                        'm', compute_metrics(sim_log, env, pk));
end

m = [results.m];
disp(table({scenarios.name}', [m.arrived]', [m.collided]', [m.t_finish]', ...
           [m.min_clearance]', [m.avoid_time]', [m.sweeps]', ...
           'VariableNames', {'scenario', 'arrived', 'collided', 't_finish_s', ...
                             'min_clearance_m', 'avoid_s', 'sweeps'}));

%% Arena maps
fig = plot_scenario_grid(results);
% exportgraphics(fig, 'arena_grid.png', 'Resolution', 150);   % save it

%% One run in detail (sensors, speed, clearance, playback)
% r = results(1);
% plot_results(r.log, r.env, r.p, r.m);
% animate_run(r.log, r.env, r.p);
