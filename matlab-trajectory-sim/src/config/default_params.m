function p = default_params()
%DEFAULT_PARAMS  Every simulation setting in one struct.
%   p = DEFAULT_PARAMS() returns the baseline configuration: a Crazyflie
%   2.x with a Multi-ranger deck in a 10 x 8 m room whose obstacles are
%   mostly lined up with the walls. Change fields after
%   calling it rather than editing the defaults, e.g.
%       p = default_params();  p.nav.v_cruise = 0.3;
%   so a run is fully described by p plus the mission in main.m.

% Simulation
p.dt     = 0.02;      % integration step [s]
p.t_max  = 300;       % hard cap on simulated time [s]
p.seed   = 1;         % RNG seed (wind gusts, sensor noise)
p.stop_on_collision = true;  % end the run when the body touches an obstacle or wall

% Vehicle: Crazyflie as a 2-D kinematic point that can also yaw; the four
% rangers turn with the body.
p.radius = 0.07;      % body radius including propellers [m] (about 13 cm tip to tip)
p.v_max  = 1.0;       % speed limit [m/s]
p.a_max  = 2.0;       % acceleration limit [m/s^2]
p.tau    = 0.25;      % velocity time constant [s]
p.yaw_rate_max = deg2rad(180);  % [rad/s]
p.psi0   = 0;         % initial yaw [rad], 0 = forward along +x

% Disturbance
p.wind      = [0; 0]; % constant wind drift [m/s]
p.wind_gust = 0;      % std dev of random gusts [m/s], 0 = off

% Map: a closed room. Its walls are obstacles the rangers can see.
p.map.xlim = [0 10];  % [m]
p.map.ylim = [0 8];   % [m]

% Multi-ranger deck: VL53L1x time-of-flight sensors facing front, left, back, right
p.ranger.range_max = 4.0;          % beyond this: no return (Inf) [m]
p.ranger.fov       = deg2rad(27);  % cone width; 0 = a single centre ray [rad]
p.ranger.n_rays    = 9;            % rays cast across the cone; the nearest hit is reported
p.ranger.period    = 0.04;         % update period [s]
p.ranger.noise     = 0;            % Gaussian range noise: std dev [m]...
p.ranger.noise_rel = 0;            % ...plus this fraction of the range [-]

% Navigation: straight-line GO_TO_GOAL; WedgeBug AVOID (sweep, then round
% the shorter side, wall-following when the edge is out of view)
p.nav.v_cruise      = 0.5;   % GO_TO_GOAL speed [m/s]
p.nav.v_follow      = 0.3;   % wall-following speed [m/s]
p.nav.k_stop        = 2.0;   % speed = k_stop * distance to the next stop point [1/s]
p.nav.k_yaw         = 4.0;   % yaw rate = k_yaw * heading error [1/s]
p.nav.yaw_tol       = deg2rad(10);  % heading error above which the drone stops and turns on the spot
p.nav.yaw_free_radius = 0.2; % within this of a target, slide onto it without turning [m]
p.nav.d_sweep       = 1.0;   % stop this far from an obstacle ahead and sweep [m]
p.nav.sweep_half    = deg2rad(45);  % sweep +/- this about the blocked heading
p.nav.sweep_rate    = deg2rad(60);  % yaw rate while sweeping [rad/s]
p.nav.edge_jump     = 0.5;   % a range jump this big marks the end of an obstacle's outline [m]
p.nav.pass_margin   = 0.4;   % fly at least this far outside a seen edge [m]
p.nav.tangent_clearance = 0.5; % ...and far enough out that the leg on to the target keeps this from the obstacle [m]
p.nav.pass_max      = 1.5;   % but never more than this far out [m]
p.nav.max_sweeps    = 12;    % sweeps on one obstacle before the target counts as unreachable
p.nav.max_winding   = 2.5*pi;% circling the target this far without getting closer: unreachable [rad]
p.nav.d_side        = 0.25;  % side reading that switches GO_TO_GOAL to wall-following [m]
p.nav.d_detect      = 0.6;   % wall-following: a way counts as clear beyond this [m]
p.nav.d_wall        = 0.3;   % stop this far from whatever is ahead [m]
p.nav.d_open        = 0.6;   % wall-side reading above this means the wall has ended [m]
p.nav.d_close       = 0.22;  % wall-side reading below this: too close, turn away [m]
p.nav.corner_margin = 0.3;   % extra travel past a wall's end before turning back toward it [m]
p.nav.v_turn        = 0.05;  % brake below this speed before changing direction [m/s]
p.nav.wp_tol        = 0.10;  % a waypoint counts as reached within this distance [m]
p.nav.leave_margin  = 0.10;  % leave the wall only this much closer to the target than the hit point [m]
p.nav.loop_tol      = 0.15;  % back within this of where wall-following began...  [m]
p.nav.loop_min_travel = 1.0; % ...after following at least this far = full loop, unreachable [m]

% End of run and metrics
p.goal_vtol = 0.05;   % the run ends once the goal is reached and speed is below this [m/s]
p.near_miss = 0.10;   % a body-to-obstacle gap below this counts as a near miss [m]
end
