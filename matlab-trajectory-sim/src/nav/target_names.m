function names = target_names(n)
%TARGET_NAMES  Display names for n targets: 'waypoint 1', ..., then 'goal'.
names = [arrayfun(@(i) sprintf('waypoint %d', i), 1:n-1, 'UniformOutput', false), {'goal'}];
end
