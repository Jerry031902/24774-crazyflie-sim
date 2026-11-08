%% Sudden static obstacle, observed only through the delayed Multi-ranger.
clear; clc; close all; setup_paths();
p = default_params();
p.t_max = 90;
gap = 0.24;           % body-to-box clearance at appearance [m]
appear_time = 4.02;   % just after a ranger sample: next result arrives at 4.20 s
% Try gap = 0.55 (ample), 0.20 (near current zero-target limit), 0.05 (too close).
% p.safety.enabled = false;  % matched encounter using the original navigator
% p.ranger.noise = 0.1;      % optional stress test
scene = popup_scenario(gap,appear_time,p);
env = scene.env;
sim_log = run_simulation(scene.start,scene.waypoints,scene.goal,env,p);
m = compute_metrics(sim_log,env,p); print_metrics(m);
disp(struct2table(popup_summary(sim_log,env,p)));
plot_popup(sim_log,env,p);
plot_results(sim_log,env,p,m);
animate_run(sim_log,env,p,struct('speed',1,'start_time',appear_time-0.4));
