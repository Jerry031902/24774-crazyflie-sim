function fig = plot_safety(L, p)
%PLOT_SAFETY  Candidate/selected commands, stopping margins and sensor health.
c = viz_colors(); t = L.t;
fig = figure('Name','Safety diagnostics','Color',c.surface,'Position',[100 60 1200 850]);
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
axes_list = gobjects(1,6);
for k = 1:6
    axes_list(k) = nexttile(tl); style_axes(axes_list(k));
end
ax = axes_list(1);
plot(ax,t,vecnorm(L.nominal_u_body,2,2),':','Color',c.muted,'DisplayName','nominal');
plot(ax,t,vecnorm(L.candidate_u_body,2,2),'--','Color',c.go,'DisplayName','assessed candidate');
plot(ax,t,vecnorm(L.u_body,2,2),'Color',c.critical,'DisplayName','selected command');
plot(ax,t,vecnorm(L.ground_velocity,2,2),'Color',c.ink,'DisplayName','ground speed');
ylabel(ax,'Speed [m/s]'); title(ax,'Command arbitration'); legend(ax,'Location','best');
ax = axes_list(2);
plot(ax,t,L.predicted_stop_distance,'Color',c.critical);
ylabel(ax,'Distance [m]'); title(ax,'Predicted zero-target stopping distance');
ax = axes_list(3);
a = L.primary_margin; b = L.backup_margin;
a(~isfinite(a)) = NaN; b(~isfinite(b)) = NaN;
plot(ax,t,a,'Color',c.go,'DisplayName','primary');
plot(ax,t,b,'Color',c.serious,'DisplayName','backup');
yline(ax,0,'--','intervention','Color',c.critical,'HandleVisibility','off');
ylabel(ax,'Available - required [m]'); title(ax,'Observed-direction stopping margins');
legend(ax,'Location','best');
ax = axes_list(4);
plot(ax,t,L.ranger_age,'Color',c.ink);
yline(ax,p.safety.max_data_age,'--','timeout','Color',c.critical);
ylabel(ax,'Measurement age [s]'); title(ax,'Age includes sensor latency');
ax = axes_list(5);
stairs(ax,t,L.safety_state,'Color',c.critical,'LineWidth',1.3);
yticks(ax,0:4); yticklabels(ax,[{'DISABLED'},L.safety_names]); ylim(ax,[-0.2 4.2]);
title(ax,'Safety state (independent of navigation)'); xlabel(ax,'Time [s]');
ax = axes_list(6);
stairs(ax,t,double(L.primary_trip),'Color',c.go,'DisplayName','primary trigger');
stairs(ax,t,2*double(L.backup_trip),'Color',c.serious,'DisplayName','backup trigger');
yticks(ax,[0 1 2]); yticklabels(ax,{'clear','primary','backup'}); ylim(ax,[-0.2 2.2]);
title(ax,'Parallel monitor triggers'); xlabel(ax,'Time [s]'); legend(ax,'Location','best');
linkaxes(axes_list,'x'); xlim(axes_list(1),[0 max(t(end),p.dt)]);
end
