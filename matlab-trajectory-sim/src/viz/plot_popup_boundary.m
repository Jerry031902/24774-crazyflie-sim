function fig = plot_popup_boundary(report, phases)
%PLOT_POPUP_BOUNDARY  On/off clearance comparison with tolerant timestamp matching.
if ismember('suite',report.Properties.VariableNames)
    report = report(strcmp(report.suite,'distance_sweep'),:);
end
if nargin < 2, phases = unique(report.appear_s)'; end
fig = figure('Name','Popup distance boundary','Color','white','Position',[80 80 1350 430]);
tl = tiledlayout(fig,1,numel(phases),'TileSpacing','compact','Padding','compact');
for j = 1:numel(phases)
    ax = nexttile(tl); style_axes(ax);
    selected = report(abs(report.appear_s-phases(j)) < 1e-9,:);
    assert(~isempty(selected),'plot_popup_boundary:noPhase','No encounters match this phase.');
    for enabled = [false true]
        a = selected(selected.enabled == enabled,:);
        if isempty(a), continue; end
        if enabled, label = 'Safety on'; else, label = 'Safety off'; end
        plot(ax,a.requested_gap_m,a.min_gap_after_m,'-o','DisplayName',label,'LineWidth',1.2);
    end
    xline(ax,mean(selected.zero_target_required_gap_m),'--','zero-target limit', ...
        'HandleVisibility','off','FontSize',8);
    yline(ax,0,':','contact','HandleVisibility','off');
    xlabel(ax,'Body gap at appearance [m]'); ylabel(ax,'Minimum subsequent body gap [m]');
    title(ax,sprintf('Appearance %.2f s',phases(j))); legend(ax,'Location','best');
end
end
