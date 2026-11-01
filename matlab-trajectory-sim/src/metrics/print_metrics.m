function print_metrics(m)
%PRINT_METRICS  Print metrics from COMPUTE_METRICS.
%   One run prints as a labelled list; a struct array prints as a table.
if numel(m) > 1
    disp(struct2table(m));
    return
end

if m.collided
    outcome = 'COLLIDED';
elseif m.arrived
    outcome = sprintf('reached the goal at %.1f s', m.t_finish);
else
    outcome = 'did not reach the goal';
end
unreachable = m.unreachable;
if isempty(unreachable)
    unreachable = 'none';
end

fprintf('\n  Run metrics\n');
fprintf('    %-18s %s\n', 'Outcome', outcome);
fprintf('    %-18s %d of %d reached; unreachable: %s\n', 'Targets', m.targets_reached, m.n_targets, unreachable);
fprintf('    %-18s %.1f m flown, %.1f m in straight lines (x%.2f)\n', 'Path length', ...
    m.path_length, m.direct_length, m.path_ratio);
fprintf('    %-18s %d episodes, %d sweeps, %.1f s\n', 'AVOID', m.avoid_episodes, m.sweeps, m.avoid_time);
fprintf('    %-18s %.2f m minimum, %d near misses\n', 'Clearance', m.min_clearance, m.near_misses);
fprintf('    %-18s %.2f m/s\n\n', 'Peak speed', m.max_speed);
end
