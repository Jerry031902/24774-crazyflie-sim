function env = make_environment(p, obstacles)
%MAKE_ENVIRONMENT  Room bounds plus an optional list of obstacles.
%   env = MAKE_ENVIRONMENT(p) is an empty room with walls at p.map.xlim
%   and p.map.ylim. env = MAKE_ENVIRONMENT(p, obstacles) adds obstacles, a
%   cell array of OBSTACLE_SQUARE / OBSTACLE_RECT / OBSTACLE_CIRCLE
%   results, e.g.
%       env = make_environment(p, {obstacle_square([2.5 3], 1), ...
%                                  obstacle_rect([5.5 4], [0.2 3.4]), ...
%                                  obstacle_circle([7.4 2.6], 0.6)});
%
%   env.xlim, env.ylim   room bounds [m]
%   env.obstacles        struct array with .type and .vertices (Kx2 polygon)
%   env.edges            every wall segment, room walls included, one per
%                        row as [ax ay bx by]; used by the rangers and the
%                        collision check

if nargin < 2
    obstacles = {};
end

env.xlim = p.map.xlim;
env.ylim = p.map.ylim;
if isempty(obstacles)
    env.obstacles = struct('type', {}, 'vertices', {});
else
    env.obstacles = vertcat(obstacles{:});
end

room = [env.xlim(1) env.ylim(1); env.xlim(2) env.ylim(1);
        env.xlim(2) env.ylim(2); env.xlim(1) env.ylim(2)];
env.edges = [room, circshift(room, -1, 1)];
for i = 1:numel(env.obstacles)
    V = env.obstacles(i).vertices;
    env.edges = [env.edges; V, circshift(V, -1, 1)];
end
end
