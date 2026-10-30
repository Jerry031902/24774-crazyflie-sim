function obs = obstacle_square(center, side, angle_deg)
%OBSTACLE_SQUARE  Square obstacle.
%   obs = OBSTACLE_SQUARE([x y], side) is a square of the given side length
%   centred at [x y]. OBSTACLE_SQUARE([x y], side, angle_deg) rotates it
%   counter-clockwise about its centre.
if nargin < 3, angle_deg = 0; end
obs = obstacle_rect(center, [side side], angle_deg);
obs.type = 'square';
end
