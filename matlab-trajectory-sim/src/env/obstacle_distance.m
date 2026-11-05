function d = obstacle_distance(env, P, time)
%OBSTACLE_DISTANCE  Signed distance from points to the nearest obstacle or map edge.
%   d = OBSTACLE_DISTANCE(env, P) for points P (Nx2) returns d (Nx1): the
%   distance to the closest obstacle edge, negative inside an obstacle.
%   The map boundary counts as a wall, so d is never larger than the
%   distance to the edge of the map, and is negative outside the map.
%   For scheduled obstacles, provide one time per point (or a scalar time).
%   Future geometry must not affect earlier clearance/collision metrics.

d = min([P(:,1) - env.xlim(1), env.xlim(2) - P(:,1), ...
         P(:,2) - env.ylim(1), env.ylim(2) - P(:,2)], [], 2);

for k = 1:numel(env.obstacles)
    d = min(d,polygon_distance(P,env.obstacles(k).vertices));
end
if isfield(env,'scheduled') && ~isempty(env.scheduled)
    if nargin < 3
        error('obstacle_distance:timeRequired','Scheduled obstacles require point timestamps.');
    end
    if isscalar(time), time = repmat(time,size(P,1),1); end
    assert(numel(time)==size(P,1),'obstacle_distance:timeSize','One time is required per point.');
    for event = env.scheduled
        active = time(:) >= event.time-1e-9;
        d(active) = min(d(active),polygon_distance(P(active,:),event.obstacle.vertices));
    end
end
end

function dk = polygon_distance(P,V)
    W  = circshift(V, -1, 1);          % edge e runs from V(e,:) to W(e,:)
    dk = inf(size(P, 1), 1);
    for e = 1:size(V, 1)
        ab = W(e,:) - V(e,:);
        ap = P - V(e,:);
        s  = min(max((ap * ab') / (ab * ab'), 0), 1);   % closest point on the edge
        dk = min(dk, vecnorm(ap - s .* ab, 2, 2));
    end
    inside = inpolygon(P(:,1), P(:,2), V(:,1), V(:,2));
    dk(inside) = -dk(inside);
end
