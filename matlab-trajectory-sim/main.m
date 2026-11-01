%% main.m - Crazyflie + Multi-ranger: reactive navigation through your waypoints
%
%   start -> waypoints -> goal   (define them in the Mission section)
%   each tick: read the 4 rangers -> GO_TO_GOAL / AVOID state machine
%   (straight line to each target, wall-following around obstacles)
%   -> velocity + yaw-rate command -> kinematic drone.
%   Then metrics, plots and playback.
%
% Vehicle, sensor and navigation settings are fields of p; see
% src/config/default_params.m for the full list.

clear; clc; close all;
setup_paths();
p = default_params();
p.ranger.noise = 0.1;   % Gaussian ranger noise, std dev [m] (default 0)

%% Mission (metres, inside the p.map.xlim x p.map.ylim room)
% Each leg has one obstacle in the way:
%   start -> 1: box    1 -> 2: long wall    2 -> 3: pillar    3 -> goal: bench
start     = [0.8 0.8];
waypoints = [3.4 4.2
             6.6 2.0
             9.4 3.8];   % [] for no waypoints: fly straight to the goal
goal      = [6.4 6.4];

% Obstacles: squares (obstacle_square), rectangles (obstacle_rect) and
% circles (obstacle_circle). The drone only knows about them through its
% rangers. Use {} for an empty room.
obstacles = {obstacle_square([2.5 3.0], 1.0)        % box
             obstacle_rect([5.5 4.0], [0.2 3.4])    % long wall
             obstacle_circle([8.0 2.9], 0.3)        % pillar
             obstacle_rect([7.6 5.3], [1.6 0.5])};  % bench

%% Fly it
env     = make_environment(p, obstacles);
sim_log = run_simulation(start, waypoints, goal, env, p);
m       = compute_metrics(sim_log, env, p);
print_metrics(m);

%% Plots (Figure 1) and playback (Figure 2: use its Replay button to watch again)
plot_results(sim_log, env, p, m);
animate_run(sim_log, env, p);
% To also save a video of the first playback:
% animate_run(sim_log, env, p, struct('video_file', 'drone_run.mp4'));

%% Ranger noise study (optional, a few minutes): this mission flown 10 times
% at each of several Gaussian noise levels, to see how well it copes.
% results = noise_study(start, waypoints, goal, obstacles, p);
