%% main.m - Crazyflie + Multi-ranger: WedgeBug round one box to a goal
%
%   The simplest WedgeBug run: one box between the start and the goal.
%     GO_TO_GOAL  turn to face the goal and fly straight at it
%     AVOID       the box blocks the way: stop 1 m (d_sweep) short of it and
%                 sweep the rangers +/-45 deg; find where the box ends on
%                 each side; pick the shorter way round; fly to a pass
%                 point just outside that end
%     GO_TO_GOAL  past the box: fly straight to the goal
%   Each tick: read the 4 rangers -> navigator -> velocity + yaw-rate
%   command -> kinematic drone. Then metrics, plots and playback.
%
% Vehicle, sensor and navigation settings are fields of p; see
% src/config/default_params.m for the full list.

clear; clc; close all;
setup_paths();
p = default_params();
p.ranger.noise = 0;     % Gaussian ranger noise, std dev [m]; try 0.1 to see how it copes
% p.safety.enabled = false; % reproduce the original navigator without supervision

%% Mission (metres, inside the p.map.xlim x p.map.ylim room)
% One box, set 0.25 m above the straight line, so going round below it
% (round the right) is the shorter way.
start     = [1.5 4.0];
waypoints = [];          % none: fly straight to the goal
goal      = [8.5 4.0];

% Obstacles: squares (obstacle_square), rectangles (obstacle_rect) and
% circles (obstacle_circle). The drone only knows about them through its
% rangers. Use {} for an empty room.
obstacles = {obstacle_square([5.0 4.25], 1.0)};      % box

% The full mission instead (three waypoints, one obstacle on each leg; the
% Isaac Sim mission matlab_main in sim/navigation/missions.py):
% start     = [0.8 0.8];
% waypoints = [3.4 4.2; 6.6 2.0; 9.4 3.8];
% goal      = [6.4 6.4];
% obstacles = {obstacle_square([2.5 3.0], 1.0), ...      % box
%              obstacle_rect([5.5 4.0], [0.2 3.4]), ...  % long wall
%              obstacle_circle([8.0 2.9], 0.3), ...      % pillar
%              obstacle_rect([7.6 5.3], [1.6 0.5])};     % bench
% p.ranger.noise = 0.1;

%% Fly it
env     = make_environment(p, obstacles);
sim_log = run_simulation(start, waypoints, goal, env, p);
m       = compute_metrics(sim_log, env, p);
print_metrics(m);

%% Plots (Figure 1) and playback (Figure 2: use its Replay button to watch again)
plot_results(sim_log, env, p, m);
animate_run(sim_log, env, p);
if p.safety.enabled
    plot_safety(sim_log,p);  % candidate/selected commands, margins, sensor age
end
% To also save a video of the first playback:
% animate_run(sim_log, env, p, struct('video_file', 'drone_run.mp4'));

%% Ranger noise study (optional, a few minutes): this mission flown 10 times
% at each of several Gaussian noise levels, to see how well it copes.
% results = noise_study(start, waypoints, goal, obstacles, p);
