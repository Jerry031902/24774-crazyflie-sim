function obs = obstacle_rect(center, sz, angle_deg)
%OBSTACLE_RECT  Rectangular obstacle.
%   obs = OBSTACLE_RECT(center, [w h]) is a w-by-h rectangle centred at
%   center = [x y]. OBSTACLE_RECT(center, [w h], angle_deg) rotates it
%   counter-clockwise about its centre.
if nargin < 3, angle_deg = 0; end

hw = sz(1) / 2;
hh = sz(2) / 2;
V  = [-hw -hh; hw -hh; hw hh; -hw hh];
V  = V * rot2d(deg2rad(angle_deg))' + center(:)';
obs = struct('type', 'rect', 'vertices', V);
end
