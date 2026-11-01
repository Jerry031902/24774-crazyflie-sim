function d = obstacle_distance(env, P)
%OBSTACLE_DISTANCE  Signed distance from points to the nearest obstacle or map edge.
%   d = OBSTACLE_DISTANCE(env, P) for points P (Nx2) returns d (Nx1): the
%   distance to the closest obstacle edge, negative inside an obstacle.
%   The map boundary counts as a wall, so d is never larger than the
%   distance to the edge of the map, and is negative outside the map.

d = min([P(:,1) - env.xlim(1), env.xlim(2) - P(:,1), ...
         P(:,2) - env.ylim(1), env.ylim(2) - P(:,2)], [], 2);

for k = 1:numel(env.obstacles)
    V  = env.obstacles(k).vertices;
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
    d = min(d, dk);
end
end
