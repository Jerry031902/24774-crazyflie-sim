function env = schedule_obstacle(env, obstacle, appear_time)
%SCHEDULE_OBSTACLE  Add a static obstacle that begins to exist at a given time.
% Geometry is fixed in world coordinates, persists after activation, and is
% visible only through the ordinary delayed rangers. The schedule is not a
% navigator/monitor input. Non-grid times activate at the first following tick.
validateattributes(appear_time,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(obstacle.vertices,{'numeric'},{'2d','ncols',2,'finite','real'});
assert(size(obstacle.vertices,1) >= 3,'schedule_obstacle:vertices', ...
    'An obstacle must contain at least three vertices.');
if ~isfield(env,'scheduled')
    env.scheduled = struct('time',{},'obstacle',{});
end
env.scheduled(end+1) = struct('time',appear_time,'obstacle',obstacle);
end
