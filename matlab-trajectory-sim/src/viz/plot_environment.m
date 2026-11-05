function plot_environment(ax, env)
%PLOT_ENVIRONMENT  Draw the map boundary and obstacles into axes ax.
c = viz_colors();
style_axes(ax);
for i = 1:numel(env.obstacles)
    V = env.obstacles(i).vertices;
    patch(ax, 'XData', V(:,1), 'YData', V(:,2), 'FaceColor', c.obstacle, ...
        'EdgeColor', c.muted, 'HandleVisibility', 'off');
end
if isfield(env,'scheduled')
    for event = env.scheduled
        V = event.obstacle.vertices;
        patch(ax,'XData',V(:,1),'YData',V(:,2),'FaceColor',c.critical, ...
            'FaceAlpha',0.10,'EdgeColor',c.critical,'LineStyle','--','HandleVisibility','off');
        text(ax,mean(V(:,1)),max(V(:,2)),sprintf('  appears at %.2f s',event.time), ...
            'Color',c.critical,'FontSize',8,'VerticalAlignment','bottom');
    end
end
rectangle(ax, 'Position', [env.xlim(1) env.ylim(1) diff(env.xlim) diff(env.ylim)], ...
    'EdgeColor', c.ink2, 'LineWidth', 1.5);

axis(ax, 'equal');
pad = 0.02 * max(diff(env.xlim), diff(env.ylim));
xlim(ax, env.xlim + [-pad pad]);
ylim(ax, env.ylim + [-pad pad]);
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
end
