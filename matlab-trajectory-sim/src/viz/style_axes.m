function style_axes(ax)
%STYLE_AXES  Quiet axes: light grid, grey axis lines, outward ticks.
c = viz_colors();
set(ax, 'Color', c.surface, 'XColor', c.ink2, 'YColor', c.ink2, ...
    'GridColor', c.grid, 'GridAlpha', 1, 'Box', 'off', 'TickDir', 'out', ...
    'FontSize', 9);
grid(ax, 'on');
hold(ax, 'on');
end
