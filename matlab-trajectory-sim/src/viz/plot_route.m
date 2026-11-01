function h = plot_route(ax, sim_log)
%PLOT_ROUTE  Draw the route: start, numbered waypoints, goal, unreachable flags.
%   h = PLOT_ROUTE(ax, sim_log) draws a thin dashed line start ->
%   waypoints -> goal (just start -> goal when there are no waypoints) and
%   returns handles [route, waypoint markers] for a legend.
c   = viz_colors();
tg  = sim_log.targets;
wp  = tg(1:end-1, :);
off = 0.012 * diff(xlim(ax));

route = [sim_log.start; tg];
h = plot(ax, route(:,1), route(:,2), '--', 'Color', c.muted, 'LineWidth', 0.8, ...
    'DisplayName', 'Route');
if ~isempty(wp)
    h(2) = plot(ax, wp(:,1), wp(:,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.2, ...
        'MarkerEdgeColor', c.ink, 'MarkerFaceColor', c.surface, 'DisplayName', 'Waypoints');
    for i = 1:size(wp, 1)
        text(ax, wp(i,1) + off, wp(i,2) + off, sprintf('%d', i), 'Color', c.ink2, ...
            'FontSize', 9, 'VerticalAlignment', 'bottom');
    end
end

s = sim_log.start;
plot(ax, s(1), s(2), 's', 'MarkerSize', 9, 'MarkerFaceColor', c.ink, ...
    'MarkerEdgeColor', c.ink, 'HandleVisibility', 'off');
plot(ax, tg(end,1), tg(end,2), 'p', 'MarkerSize', 14, 'MarkerFaceColor', c.ink, ...
    'MarkerEdgeColor', c.ink, 'HandleVisibility', 'off');
text(ax, s(1) + 2*off, s(2), 'Start', 'Color', c.ink, 'FontSize', 9);
text(ax, tg(end,1) + 2*off, tg(end,2), 'Goal', 'Color', c.ink, 'FontSize', 9);

for i = sim_log.unreachable
    plot(ax, tg(i,1), tg(i,2), 'x', 'Color', c.critical, 'MarkerSize', 14, ...
        'LineWidth', 2, 'HandleVisibility', 'off');
    text(ax, tg(i,1) + 2*off, tg(i,2) - 2*off, 'unreachable', 'Color', c.critical, ...
        'FontSize', 9, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
end
end
