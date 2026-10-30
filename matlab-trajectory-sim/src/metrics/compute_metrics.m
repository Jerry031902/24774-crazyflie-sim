function m = compute_metrics(sim_log, env, p)
%COMPUTE_METRICS  Summary numbers for one run.
%   m = COMPUTE_METRICS(sim_log, env, p) returns a scalar struct, so the
%   metrics from several runs (a parameter sweep, say) stack into a
%   struct array and convert with STRUCT2TABLE.
%     arrived, t_finish   reached the goal and stopped, and when [s]
%     collided            body touched an obstacle or wall
%     targets_reached     waypoints and goal reached, out of n_targets
%     unreachable         names of targets given up on ('' if none)
%     path_length         distance flown [m]
%     direct_length       straight lines start -> waypoints -> goal [m]
%     path_ratio          path_length / direct_length
%     avoid_episodes      obstacles met (AVOID episodes)
%     sweeps              +/-45 deg sweeps made
%     avoid_time          time spent in AVOID [s]
%     min_clearance       smallest gap between body and obstacle or wall [m]
%     near_misses         separate times that gap dropped below p.near_miss
%                         (collisions included)
%     max_speed           [m/s]

pos   = sim_log.x(:, 1:2);
gap   = obstacle_distance(env, pos) - p.radius;
route = [sim_log.start; sim_log.targets];
types = {sim_log.events.type};

m.arrived         = sim_log.arrived;
m.t_finish        = sim_log.t_arrive;
m.collided        = sim_log.collided || any(gap < 0);
m.targets_reached = nnz(strcmp(types, 'reached'));
m.n_targets       = size(sim_log.targets, 1);
m.unreachable     = strjoin(sim_log.target_names(sim_log.unreachable), ', ');
m.path_length     = sum(vecnorm(diff(pos), 2, 2));
m.direct_length   = sum(vecnorm(diff(route), 2, 2));
m.path_ratio      = m.path_length / m.direct_length;
m.avoid_episodes  = nnz(strcmp(types, 'hit'));
m.sweeps          = numel(sim_log.sweeps);
m.avoid_time      = nnz(sim_log.mode == 2) * p.dt;
m.min_clearance   = min(gap);
m.near_misses     = sum(diff([false; gap < p.near_miss]) == 1);
m.max_speed       = max(vecnorm(sim_log.x(:, 3:4), 2, 2));
end
