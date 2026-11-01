function [x, y] = mode_path(pos, mode, which)
%MODE_PATH  Path coordinates for one navigator mode, NaN elsewhere.
%   [x, y] = MODE_PATH(pos, mode, which) keeps the samples where mode
%   equals which, plus the first sample after each stretch so the
%   differently coloured pieces join without gaps.
keep = mode(:) == which;
keep = keep | [false; keep(1:end-1)];
x = pos(:,1);
y = pos(:,2);
x(~keep) = NaN;
y(~keep) = NaN;
end
