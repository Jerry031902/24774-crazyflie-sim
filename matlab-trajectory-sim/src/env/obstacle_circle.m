function obs = obstacle_circle(center, r, n)
%OBSTACLE_CIRCLE  Circular obstacle, stored as an n-sided polygon (default 48).
%   obs = OBSTACLE_CIRCLE([x y], r). The polygon circumscribes the circle,
%   so distances to it are never larger than distances to the true circle.
if nargin < 3, n = 48; end

th = (0:n-1)' * (2*pi / n);
V  = center(:)' + (r / cos(pi/n)) * [cos(th) sin(th)];
obs = struct('type', 'circle', 'vertices', V);
end
