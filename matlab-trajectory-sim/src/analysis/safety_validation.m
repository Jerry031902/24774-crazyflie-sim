function [comparison, faults_report] = safety_validation(output_dir, seeds)
%SAFETY_VALIDATION  Reproducible on/off mission study and fault-injection runs.
% Writes CSVs, parameters/logs and rendered figures under build/safety by default.
if nargin < 1 || isempty(output_dir)
    root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    output_dir = fullfile(root,'build','safety');
end
if nargin < 2, seeds = 1:10; end
if ~exist(output_dir,'dir'), mkdir(output_dir); end
base = default_params();
start = [0.8 0.8]; waypoints = [3.4 4.2;6.6 2;9.4 3.8]; goal = [6.4 6.4];
obstacles = {obstacle_square([2.5 3],1),obstacle_rect([5.5 4],[0.2 3.4]), ...
    obstacle_circle([8 2.9],0.3),obstacle_rect([7.6 5.3],[1.6 0.5])};
env = make_environment(base,obstacles);
rows = struct([]);
for noise = [0 0.05 0.1]
    for enabled = [false true]
        for seed = seeds
            p = base; p.seed = seed; p.ranger.noise = noise; p.safety.enabled = enabled;
            L = run_simulation(start,waypoints,goal,env,p);
            m = compute_metrics(L,env,p);
            row = struct('enabled',enabled,'noise_m',noise,'seed',seed, ...
                'success',m.arrived && ~m.collided && m.targets_reached == 4, ...
                'collided',m.collided,'targets',m.targets_reached,'finish_s',m.t_finish, ...
                'min_gap_m',m.min_clearance,'brakes',m.safety_brakes, ...
                'backup_brakes',m.backup_brakes,'override_s',m.safety_time);
            if isempty(rows), rows = row; else, rows(end+1) = row; end %#ok<AGROW>
            if enabled && noise == 0.1 && seed == seeds(1)
                example = L; example_p = p; %#ok<NASGU>
                save(fullfile(output_dir,'example_enabled.mat'),'L','p','env','m');
            end
        end
    end
end
comparison = struct2table(rows);
writetable(comparison,fullfile(output_dir,'comparison.csv'));
disp(groupsummary(comparison,{'enabled','noise_m'},'mean',{'success','collided','brakes','override_s'}));

p = base; p.t_max = 35; empty = make_environment(p,{});
cases = {'ranger_dropout','primary_timeout','navigator_timeout'};
rows = struct([]);
for k = 1:numel(cases)
    inject = struct(); inject.(cases{k}) = [3 4];
    L = run_simulation([1 4],[],[9 4],empty,p,inject);
    m = compute_metrics(L,empty,p);
    row = struct('fault',cases{k},'arrived',m.arrived,'collided',m.collided, ...
        'brakes',m.safety_brakes,'backup_brakes',m.backup_brakes, ...
        'max_brake_s',m.max_brake_time,'max_brake_m',m.max_brake_distance);
    if isempty(rows), rows = row; else, rows(end+1) = row; end %#ok<AGROW>
    if k == 2, fault_example = L; end
end
faults_report = struct2table(rows);
writetable(faults_report,fullfile(output_dir,'faults.csv')); disp(faults_report);

old = get(groot,'defaultFigureVisible'); set(groot,'defaultFigureVisible','off');
cleanup = onCleanup(@() set(groot,'defaultFigureVisible',old)); %#ok<NASGU>
fig = plot_results(example,env,example_p,compute_metrics(example,env,example_p));
exportgraphics(fig,fullfile(output_dir,'mission_enabled.png'),'Resolution',140); close(fig);
fig = plot_safety(example,example_p);
exportgraphics(fig,fullfile(output_dir,'mission_safety.png'),'Resolution',140); close(fig);
fig = plot_safety(fault_example,p);
exportgraphics(fig,fullfile(output_dir,'primary_timeout.png'),'Resolution',140); close(fig);
save(fullfile(output_dir,'validation.mat'),'comparison','faults_report','base','seeds');
end
