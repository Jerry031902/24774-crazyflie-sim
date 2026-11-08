function fig = plot_popup(L, env, p)
%PLOT_POPUP  Appearance, delayed perception, braking and outcome of one popup.
r = popup_summary(L,env,p); e = L.obstacle_events(1); c = viz_colors();
fig = figure('Name','Sudden obstacle encounter','Color',c.surface,'Position',[80 40 1250 900]);
tl = tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl,1,[4 1]); plot_environment(ax,env);
plot(ax,L.x(:,1),L.x(:,2),'Color',c.go,'LineWidth',1.5);
plot(ax,e.pos(1),e.pos(2),'o','MarkerFaceColor',c.go,'MarkerEdgeColor',c.go);
text(ax,e.pos(1),e.pos(2)-0.08,'appearance position','FontSize',8,'VerticalAlignment','top');
if L.collided
    plot(ax,L.x(end,1),L.x(end,2),'x','Color',c.critical,'MarkerSize',12,'LineWidth',2);
end
xlim(ax,[e.pos(1)-0.6,e.pos(1)+1.6]); ylim(ax,[2.7 5.3]);
title(ax,sprintf('%s | Safety %s\nInitial body gap %.3f m, speed %.2f m/s', ...
    strrep(r.outcome,'_',' '),on_off(L.safety_enabled),e.gap,e.speed),'FontWeight','normal');
window = L.t >= e.t-0.4 & L.t <= e.t+2;
tw = L.t(window); axes_list = gobjects(1,4);
for j = 1:4, axes_list(j) = nexttile(tl,2*j); style_axes(axes_list(j)); end
% Instantaneous geometry is diagnostic only. Noise is disabled for this trace.
q = p; q.ranger.noise = 0; q.ranger.noise_rel = 0;
instant = zeros(numel(tw),1); indices = find(window);
for j = 1:numel(indices)
    k = indices(j);
    ranges = read_rangers(L.x(k,1:2)',L.psi(k),environment_at(env,L.t(k)),q);
    instant(j) = min(ranges(1),p.ranger.range_max);
end
ax = axes_list(1);
plot(ax,tw,instant,'Color',c.front,'DisplayName','instantaneous geometric front');
stairs(ax,tw,min(L.ranges(window,1),p.ranger.range_max),'Color',c.ink,'DisplayName','delivered front');
ylabel(ax,'Sensor range [m]'); title(ax,'No return plotted at range limit');
legend(ax,'Location','best','FontSize',8);
ax = axes_list(2);
plot(ax,tw,vecnorm(L.ground_velocity(window,:),2,2),'Color',c.ink,'DisplayName','ground speed');
plot(ax,tw,vecnorm(L.u_body(window,:),2,2),'--','Color',c.go,'DisplayName','selected command');
ylim(ax,[0,max(0.1,1.15*max([vecnorm(L.ground_velocity(window,:),2,2);vecnorm(L.u_body(window,:),2,2)]))]);
ylabel(ax,'Speed [m/s]'); title(ax,'Same velocity dynamics in both modes'); legend(ax,'Location','best','FontSize',8);
ax = axes_list(3);
gap = obstacle_distance(env,L.x(:,1:2),L.t)-p.radius;
gap(L.t < e.t-1e-9) = NaN;
plot(ax,tw,gap(window),'Color',c.ink);
yline(ax,0,'--','body contact','Color',c.critical,'HandleVisibility','off');
ylabel(ax,'True body gap [m]'); title(ax,'Scoring uses active geometry at each time');
ax = axes_list(4);
stairs(ax,tw,L.safety_state(window),'Color',c.critical);
yticks(ax,0:4); yticklabels(ax,[{'DISABLED'},L.safety_names]); ylim(ax,[-0.2 4.2]);
xlabel(ax,'Time [s]'); title(ax,'Safety state');
for ax = axes_list
    xline(ax,e.t,':','Color',c.serious,'HandleVisibility','off');
    if isfinite(r.first_fresh_delivery_s)
        xline(ax,r.first_fresh_delivery_s,'--','Color',c.front,'HandleVisibility','off');
    end
    if isfinite(r.first_zero_command_s)
        xline(ax,r.first_zero_command_s,'-.','Color',c.go,'HandleVisibility','off');
    end
end
linkaxes(axes_list,'x'); xlim(axes_list(1),[e.t-0.4,max(tw(end),e.t+p.dt)]);
title(tl,sprintf('Appearance %.2f s | fresh delivery %s | zero command %s | stop %s', ...
    e.t,time_label(r.first_fresh_delivery_s,'not received'), ...
    time_label(r.first_zero_command_s,'not issued'),time_label(r.first_stopped_s,'not reached')), ...
    'FontWeight','normal');
end

function s = on_off(enabled)
if enabled, s = 'on'; else, s = 'off'; end
end

function s = time_label(t,missing)
if isfinite(t), s = sprintf('%.2f s',t); else, s = missing; end
end
