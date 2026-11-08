function report = popup_validation(output_dir, seeds)
%POPUP_VALIDATION  Matched on/off encounters across distance, sample phase, noise.
% 66 noiseless cases (11 gaps x 3 phases x 2 modes), plus 60 noise cases
% by default (3 gaps x 2 noises x 5 seeds x 2 modes). Collisions in insufficient
% distance cases are measured outcomes, not assertions that a test failed.
if nargin < 1 || isempty(output_dir)
    root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    output_dir = fullfile(root,'build','popups');
end
if nargin < 2, seeds = 1:5; end
validateattributes(seeds,{'numeric'},{'vector','integer','positive','nonempty','finite'});
if ~exist(output_dir,'dir'), mkdir(output_dir); end
base = default_params(); base.t_max = 90;
gaps = [0.05 0.10 0.14 0.16 0.18 0.20 0.22 0.24 0.28 0.35 0.55];
phases = [4 4.02 4.08]; rows = struct([]);
for appear = phases
    for gap = gaps
        for enabled = [false true]
            p = base; p.safety.enabled = enabled;
            [row,L,scene] = one_run(gap,appear,p,'distance_sweep');
            if isempty(rows), rows = row; else, rows(end+1) = row; end %#ok<AGROW>
            if appear == 4.02 && enabled && ismember(gap,[0.05 0.18 0.24 0.55])
                name = sprintf('gap_%03d_on',round(1000*gap));
                env = scene.env; m = compute_metrics(L,env,p); %#ok<NASGU>
                save(fullfile(output_dir,[name '.mat']),'L','env','p','m','row','scene');
            end
        end
    end
end
for noise = [0.05 0.1]
    for gap = [0.05 0.22 0.55]
        for seed = seeds(:)'
            for enabled = [false true]
                p = base; p.safety.enabled = enabled; p.seed = seed; p.ranger.noise = noise;
                row = one_run(gap,4.02,p,'noise_stress');
                rows(end+1) = row; %#ok<AGROW>
            end
        end
    end
end
report = struct2table(rows);
writetable(report,fullfile(output_dir,'encounters.csv'));
distance_sweep = report(strcmp(report.suite,'distance_sweep'),:);
noise_stress = report(strcmp(report.suite,'noise_stress'),:);
writetable(distance_sweep,fullfile(output_dir,'distance_sweep.csv'));
writetable(noise_stress,fullfile(output_dir,'noise_stress.csv'));
summary = groupsummary(report,{'suite','noise_m','requested_gap_m','enabled'}, ...
    'mean',{'collided','arrived'});
writetable(summary,fullfile(output_dir,'summary.csv')); disp(summary);
save(fullfile(output_dir,'validation.mat'),'report','summary','base','gaps','phases','seeds');

old = get(groot,'defaultFigureVisible'); set(groot,'defaultFigureVisible','off');
cleanup = onCleanup(@() set(groot,'defaultFigureVisible',old)); %#ok<NASGU>
for gap = [0.05 0.18 0.24 0.55]
    name = sprintf('gap_%03d_on',round(1000*gap));
    S = load(fullfile(output_dir,[name '.mat']));
    fig = plot_popup(S.L,S.env,S.p);
    exportgraphics(fig,fullfile(output_dir,[name '.png']),'Resolution',140); close(fig);
end
fig = plot_popup_boundary(distance_sweep,phases);
exportgraphics(fig,fullfile(output_dir,'distance_boundary.png'),'Resolution',140); close(fig);
end

function [row,L,scene] = one_run(gap,appear,p,suite)
scene = popup_scenario(gap,appear,p);
L = run_simulation(scene.start,scene.waypoints,scene.goal,scene.env,p);
row = popup_summary(L,scene.env,p);
row.suite = suite; row.requested_gap_m = gap;
end
