function validate_waypoints(start, targets, env, p)
%VALIDATE_WAYPOINTS  Error if the start or any target cannot be reached.
%   VALIDATE_WAYPOINTS(start, targets, env, p) checks the start point and
%   every target (waypoints, then the goal). The drone stops p.nav.d_wall
%   short of anything ahead, so a point inside an obstacle, or nearer than
%   that to an obstacle or a room wall, can never be reached.

points = [start; targets];
names  = [{'start'}, target_names(size(targets, 1))];
d   = obstacle_distance(env, points);
bad = find(d < p.nav.d_wall);
if isempty(bad)
    return
end

lines = cell(numel(bad), 1);
for i = 1:numel(bad)
    b = bad(i);
    if d(b) < 0
        where = 'is inside an obstacle';
    else
        where = sprintf('is only %.2f m from an obstacle or wall', d(b));
    end
    lines{i} = sprintf('  %s (%.2f, %.2f) %s', names{b}, points(b,1), points(b,2), where);
end
error('validate_waypoints:unreachable', ...
    'These points cannot be reached:\n%s\nKeep every point at least p.nav.d_wall = %.2f m from obstacles and walls.', ...
    strjoin(lines, newline), p.nav.d_wall);
end
