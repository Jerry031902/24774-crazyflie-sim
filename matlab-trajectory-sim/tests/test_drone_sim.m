function tests = test_drone_sim
%TEST_DRONE_SIM  Unit and closed-loop tests. Run with  runtests('tests')
tests = functiontests(localfunctions);
end

function setupOnce(tc)
addpath(fileparts(fileparts(mfilename('fullpath'))));
setup_paths();
tc.TestData.p = default_params();
end

% --- Motion model ---------------------------------------------------------

function testSpeedAndAccelerationLimits(tc)
p = tc.TestData.p;
x = zeros(4, 1);
for k = 1:200
    v_prev = x(3:4);
    x = dynamics_step(x, 0, [100; 50], 0, [0; 0], p);   % far beyond both limits
    verifyLessThanOrEqual(tc, norm(x(3:4)), p.v_max + 1e-9);
    verifyLessThanOrEqual(tc, norm(x(3:4) - v_prev) / p.dt, p.a_max + 1e-6);
end
verifyEqual(tc, norm(x(3:4)), p.v_max, 'AbsTol', 1e-3);   % the lag approaches it exponentially
[~, psi] = dynamics_step(x, 0, [0; 0], 100, [0; 0], p);    % yaw rate far beyond its limit
verifyEqual(tc, psi, p.yaw_rate_max * p.dt, 'AbsTol', 1e-12);
end

function testFirstOrderLag(tc)
% Below the acceleration limit, velocity reaches 63% of a step command after tau.
p = tc.TestData.p;
p.a_max = 1e3;
x = zeros(4, 1);
for k = 1:round(p.tau / p.dt)
    x = dynamics_step(x, 0, [1; 0], 0, [0; 0], p);
end
verifyEqual(tc, x(3), 1 - exp(-1), 'AbsTol', 0.03);
end

function testBodyFrameCommands(tc)
p = tc.TestData.p;
p.tau = p.dt;        % respond in one step
p.a_max = 1e3;
x = dynamics_step(zeros(4,1), pi/2, [1; 0], 0, [0; 0], p);   % forward with the nose along +y
verifyEqual(tc, x(3:4), [0; 1], 'AbsTol', 1e-12);
x = dynamics_step(zeros(4,1), 0, [0; 1], 0, [0; 0], p);      % left with the nose along +x
verifyEqual(tc, x(3:4), [0; 1], 'AbsTol', 1e-12);
end

% --- Environment and sensors ----------------------------------------------

function testObstacleDistance(tc)
p = tc.TestData.p;
env = make_environment(p, {obstacle_rect([5.5 4.5], [1 1])});
d = obstacle_distance(env, [7 4.5; 5.5 4.5; 5.5 7; 0.5 6]);
verifyEqual(tc, d, [1; -0.5; 1; 0.5], 'AbsTol', 1e-12);   % last point: 0.5 m from the room wall
end

function testRangersSeeWallsAndObstacles(tc)
p = tc.TestData.p;
p.ranger.fov = 0;   % single centre ray: exact distances
env = make_environment(p, {obstacle_rect([5 4], [1 1])});
r = read_rangers([3; 4], 0, env, p);
verifyEqual(tc, r, [1.5; 4; 3; 4], 'AbsTol', 1e-12);       % box ahead, room walls elsewhere
r = read_rangers([3; 4], pi/2, env, p);                    % yawed 90 deg: front now faces +y
verifyEqual(tc, r, [4; 3; 4; 1.5], 'AbsTol', 1e-12);
end

function testRangerMaxRangeAndCone(tc)
p = tc.TestData.p;
env = make_environment(p, {obstacle_rect([5 4.2], [0.2 0.2])});   % small block just off the centre line
p.ranger.fov = 0;
r = read_rangers([2; 4], 0, env, p);
verifyTrue(tc, isinf(r(1)));               % centre ray misses the block; the far wall is beyond 4 m
verifyEqual(tc, r(3), 2, 'AbsTol', 1e-12);
p.ranger.range_max = 1.5;
r = read_rangers([2; 4], 0, env, p);
verifyTrue(tc, isinf(r(3)));               % the back wall at 2 m is now out of range
p = tc.TestData.p;                         % the default 27 deg cone does see the block
r = read_rangers([2; 4], 0, env, p);
verifyEqual(tc, r(1), 2.9, 'AbsTol', 0.05);
end

function testRangerNoise(tc)
% Gaussian noise: unbiased, with the std dev asked for; no return stays no return.
p = tc.TestData.p;
p.ranger.fov = 0;
p.ranger.noise = 0.05;
env = make_environment(p, {obstacle_rect([5 4], [1 1])});
rng(3);
R = zeros(4, 2000);
for k = 1:2000
    R(:,k) = read_rangers([3; 4], 0, env, p);
end
verifyEqual(tc, mean(R(1,:)), 1.5, 'AbsTol', 0.01);
verifyEqual(tc, std(R(1,:)), 0.05, 'RelTol', 0.1);
p.ranger.range_max = 2;                     % the room walls (3-4 m away) are now out of range
verifyTrue(tc, all(isinf(read_rangers([3; 4], 0, env, p)) == [false; true; true; true]));
end

function testFliesWithRangerNoise(tc)
% 5 cm of ranger noise: still round a square without touching it.
p = tc.TestData.p;
p.ranger.noise = 0.05;
env = make_environment(p, {obstacle_square([5 4], 1.5)});
m = compute_metrics(run_simulation([1 4], [], [9 4.3], env, p), env, p);
verifyTrue(tc, m.arrived);
verifyFalse(tc, m.collided);
end

% --- Navigation -------------------------------------------------------------

function testOpenRoomFliesStraightLine(tc)
p = tc.TestData.p;
env = make_environment(p);
a = [1 1];  b = [8.5 6.5];
sim_log = run_simulation(a, [], b, env, p);
verifyTrue(tc, sim_log.arrived);
verifyEqual(tc, sim_log.x(end, 1:2), b, 'AbsTol', p.nav.wp_tol + 0.05);
n = (b - a) / norm(b - a);
off_line = abs((sim_log.x(:,1) - a(1)) * n(2) - (sim_log.x(:,2) - a(2)) * n(1));
verifyLessThan(tc, max(off_line), 0.02);                 % on the straight line throughout
verifyEqual(tc, sim_log.psi(end), atan2(n(2), n(1)), 'AbsTol', deg2rad(2));   % nose first
verifyEqual(tc, nnz(sim_log.mode == 2), 0);              % nothing to avoid
end

function testNoWaypointsFliesToGoal(tc)
p = tc.TestData.p;
env = make_environment(p, {obstacle_rect([5 4], [1.5 1.5])});
for none = {[], {}, zeros(0, 2)}   % any empty value means "no waypoints"
    sim_log = run_simulation([1 4], none{1}, [9 4], env, p);
    m = compute_metrics(sim_log, env, p);
    verifyTrue(tc, m.arrived);
    verifyFalse(tc, m.collided);
    verifyEqual(tc, sim_log.target_names, {'goal'});
end
close(plot_results(sim_log, env, p, m));   % plots handle the empty waypoint list
close(animate_run(sim_log, env, p, struct('speed', 16)));
verifyError(tc, @() run_simulation([1 4], [2 3 4], [9 4], env, p), 'run_simulation:waypoints');
end

function testShapes(tc)
% Squares, rectangles and circles, alone and combined into an L and a U
% that opens toward the drone: each must be passed without a collision,
% without looping, and with every move along a sensed direction (straight
% flights are nose first, wall-following is along body axes).
p = tc.TestData.p;
shapes = {
    'square',       {obstacle_square([5 4], 1.5)},                              [1 4], [9 4.3]
    'long wall',    {obstacle_rect([5 4.2], [0.2 6.4])},                        [2 4], [8 3.5]
    'rotated rect', {obstacle_rect([5 4], [2.5 0.5], 30)},                      [1 3], [9 5]
    'big circle',   {obstacle_circle([5 4], 1.0)},                              [1 4], [9 4.2]
    'pillar',       {obstacle_circle([5 4], 0.2)},                              [1 4], [9 4]
    'L of rects',   {obstacle_rect([5 2.3], [2 0.6]), obstacle_rect([5.7 4], [0.6 4])}, [1 1], [8 5]
    'U of rects',   {obstacle_rect([5.5 2.3], [3 0.6]), obstacle_rect([5.5 5.7], [3 0.6]), ...
                     obstacle_rect([6.7 4], [0.6 4])},                          [1 4], [8.5 4]};
for i = 1:size(shapes, 1)
    env = make_environment(p, shapes{i, 2});
    sim_log = run_simulation(shapes{i, 3}, [], shapes{i, 4}, env, p);
    m = compute_metrics(sim_log, env, p);
    verifyTrue(tc, m.arrived, shapes{i, 1});
    verifyFalse(tc, m.collided, shapes{i, 1});
    verifyGreaterThanOrEqual(tc, m.avoid_episodes, 1, shapes{i, 1});
    verifyGreaterThan(tc, m.min_clearance, 0.1, shapes{i, 1});
    verifyLessThanOrEqual(tc, max(sideways_speed(sim_log)), p.nav.v_turn + 0.01, shapes{i, 1});
end
end

function testSweepCoversPlusMinus45(tc)
% Stopped at d_sweep from an obstacle, the drone yaws its front ranger
% across +/-45 deg; with the side rangers that scans +/-135 deg.
p = tc.TestData.p;
env = make_environment(p, {obstacle_rect([5 3.4], [0.3 2.4])});
sim_log = run_simulation([1 4], [], [9 4], env, p);
s = sim_log.sweeps(1);
rel = rad2deg(wrap_to_pi(s.ang - s.center));
front = rel(1:3:end);                       % front, left, right logged in turn
verifyEqual(tc, [min(front) max(front)], [-45 45], 'AbsTol', 2);
verifyEqual(tc, [min(rel) max(rel)], [-135 135], 'AbsTol', 2);
gap = 5 - 0.15 - s.pos(1);                  % distance from the obstacle's face when sweeping
verifyEqual(tc, gap, p.nav.d_sweep, 'AbsTol', 0.05);
end

function testGoesRoundTheShorterSide(tc)
% A wall across the line with its end 0.6 m off the line on one side and
% 1.8 m on the other: the drone must go round the near end.
p = tc.TestData.p;
cases = {'up', [5 3.4], +1;    % wall y 2.2..4.6: top end is near, go left (north)
         'down', [5 4.6], -1}; % wall y 3.4..5.8: bottom end is near, go right (south)
for i = 1:2
    env = make_environment(p, {obstacle_rect(cases{i, 2}, [0.3 2.4])});
    sim_log = run_simulation([1 4], [], [9 4], env, p);
    m = compute_metrics(sim_log, env, p);
    verifyTrue(tc, m.arrived, cases{i, 1});
    verifyFalse(tc, m.collided, cases{i, 1});
    verifyEqual(tc, sim_log.sweeps(1).side, cases{i, 3}, cases{i, 1});
    beside = abs(sim_log.x(:,1) - 5) < 0.2;          % passing the wall
    y = sim_log.x(beside, 2);
    if cases{i, 3} > 0
        verifyGreaterThan(tc, min(y), 4.6, cases{i, 1});   % over the top end
    else
        verifyLessThan(tc, max(y), 3.4, cases{i, 1});      % under the bottom end
    end
    verifyLessThan(tc, m.path_ratio, 1.2, cases{i, 1});
end
end

function v = sideways_speed(sim_log)
% Speed across the body axes (the smaller of |forward| and |left| body
% velocity): near zero means every move is along a sensor's line of sight.
% The final slide onto a target (dir 0) is excluded.
n = numel(sim_log.t);
v = zeros(n, 1);
for k = 1:n
    vb = rot2d(sim_log.psi(k))' * sim_log.x(k, 3:4)';
    v(k) = min(abs(vb));
end
v(sim_log.dir == 0) = 0;
end

function testEnclosedWaypointIsReportedAndSkipped(tc)
p = tc.TestData.p;
ring = {obstacle_rect([7 2.1], [2.2 0.2]), obstacle_rect([7 5.9], [2.2 0.2]), ...
        obstacle_rect([6 4], [0.2 4]),     obstacle_rect([8 4], [0.2 4])};
env = make_environment(p, ring);
sim_log = verifyWarning(tc, @() run_simulation([1 4], [7 4], [9 7], env, p), ...
    'run_simulation:unreachable');
verifyEqual(tc, sim_log.unreachable, 1);
verifyTrue(tc, sim_log.arrived);            % the goal after it is still reached
verifyFalse(tc, sim_log.collided);
end

function testWaypointInsideObstacleIsRejected(tc)
p = tc.TestData.p;
env = make_environment(p, {obstacle_rect([5 4], [2 2])});
verifyError(tc, @() run_simulation([1 1], [5 4], [9 7], env, p), 'validate_waypoints:unreachable');
end

function testBlindDroneCollides(tc)
% With the rangers effectively off, the collision check must stop the run.
p = tc.TestData.p;
p.ranger.range_max = 0.01;
env = make_environment(p, {obstacle_rect([5 4], [1 1])});
sim_log = run_simulation([1 4], [], [9 4], env, p);
verifyTrue(tc, sim_log.collided);
verifyFalse(tc, sim_log.arrived);
verifyLessThan(tc, sim_log.x(end, 1), 4.5);   % stopped at the block's near face
end
