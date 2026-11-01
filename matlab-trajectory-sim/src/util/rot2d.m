function R = rot2d(a)
%ROT2D  2-D rotation matrix for a counter-clockwise angle a [rad].
R = [cos(a) -sin(a); sin(a) cos(a)];
end
