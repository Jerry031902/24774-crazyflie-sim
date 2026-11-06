% HONGFEI LIU - HORIZONTAL LQR WITH LINEAR DRAG
% Run in base MATLAB. No toolbox, hardware connection, or output files.
% All pending team data are explicitly marked as estimates/placeholders.
% The model, feedback noise, lag and waypoints below are simulated only.
%
% State = [x; y; vx; vy], world frame, m and m/s; yaw is fixed at zero.
% Control = [roll_ref; pitch_ref], rad; Crazyflie reported Euler signs.
% ax = -g*pitch - beta_x*vx; ay = -g*roll - beta_y*vy.
% MODEL: Greiff (2017), Appendix A.8, printed p.123, Eqs. A.61-A.63.
% https://lup.lub.lu.se/student-papers/record/8905295/file/8905299.pdf#page=135
% Keep -D/m drag and adapt the 12-state model to a four-state outer loop.
% https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/
% Pitch conversion: theta_G = -pitch_CF. Fast inner-loop tracking is an
% extra DESIGN assumption; simulation may add an estimated first-order lag.
% DARE / gain equations: https://www.mathworks.com/help/control/ref/lti.dlqr.html
% Model provenance and team handoff notes are in README.md.

clearvars;
clc;

%% 1. Replace pending values here when team data become available.
cfg.g = 9.81;
cfg.Ts = 0.01;                          % Proposed 100 Hz, not measured timing.
cfg.beta_design = [0.5; 0.5];          % Kelvin: estimated d_x/m, d_y/m (1/s).
cfg.beta_plant = [0.5; 0.5];           % Kelvin: simulated plant drag (1/s).
cfg.attitude_tau = 0.08;               % Estimated lag (s); 0 = ideal tracking.
cfg.angle_limit_deg = 10;              % Provisional per-axis tilt limit (deg).
cfg.Q = diag([20 20 4 4]);             % Initial tuning: [x y vx vy] errors.
cfg.R = diag([300 300]);               % Initial tuning: [roll pitch] in rad.
cfg.noise_std = [0.005; 0.005; 0.01; 0.01]; % Synthetic feedback noise, m / m/s.
cfg.seed = 7;                         % Repeatable simulation, not recorded data.
cfg.initial_state = zeros(4,1);
cfg.initial_angles = zeros(2,1);       % Actual roll, pitch (rad) for lag model.
cfg.waypoint_times = [0 1 6 11 16];   % Jerry placeholder: activation times (s).
cfg.waypoint_xy = [0 0; 1 0; 1 1; 0 1; 0 0]; % Each row is a hold position (m).
cfg.duration = 22;
cfg.show_open_loop = false;           % Optional second figure: old drag check.
cfg.parameter_status = 'ASSUMED PARAMETERS / SYNTHETIC FEEDBACK';

validate_config(cfg);

%% 2. Four-state design model and exact zero-order-hold discretization.
[A, B] = horizontal_model(cfg.g, cfg.beta_design);
[Ad, Bd] = exact_zoh(A, B, cfg.Ts);
controllability_rank = rank([Bd Ad*Bd Ad^2*Bd Ad^3*Bd]);
assert(controllability_rank == 4, 'The design model must be controllable.');

%% 3. Solve the discrete algebraic Riccati equation (no toolbox required).
[K, P, riccati_iterations, riccati_residual] = solve_discrete_lqr(Ad, Bd, cfg.Q, cfg.R);
closed_loop_poles = eig(Ad-Bd*K);
assert(max(abs(closed_loop_poles)) < 1, 'Nominal LQR is not Schur stable.');

%% 4. Separate simulated plant; optional lag is a placeholder for inner loops.
[A_plant, B_plant] = horizontal_model(cfg.g, cfg.beta_plant);
if cfg.attitude_tau > 0
    % Plant state [x;y;vx;vy;actual_roll;actual_pitch]. This is a linear lag
    % approximation, not the stock firmware attitude/rate PID implementation.
    A_plant = [A_plant B_plant; zeros(2,4) -eye(2)/cfg.attitude_tau];
    B_plant = [zeros(4,2); eye(2)/cfg.attitude_tau];
    plant_initial = [cfg.initial_state; cfg.initial_angles];
else
    plant_initial = cfg.initial_state;
end
[Ap, Bp] = exact_zoh(A_plant, B_plant, cfg.Ts);
C_feedback = [eye(4) zeros(4,size(Ap,1)-4)];
plant_closed_loop_poles = eig(Ap-Bp*K*C_feedback);
assert(max(abs(plant_closed_loop_poles)) < 1, ...
    'Unsaturated loop is unstable with these plant/lag estimates; retune Q/R.');

%% 5. Closed-loop simulation: reference -> error -> LQR -> limits -> plant.
rng(cfg.seed, 'twister');
t = 0:cfg.Ts:cfg.duration;
N = numel(t);
plant_state = zeros(size(Ap,1),N);
plant_state(:,1) = plant_initial;
feedback_state = zeros(4,N);
reference_state = zeros(4,N);
command = zeros(2,N);
unlimited_command = zeros(2,N);
actual_angles = zeros(2,N);
angle_limit = cfg.angle_limit_deg*pi/180;
feedback_noise = bsxfun(@times, cfg.noise_std, randn(4,N));

for k = 1:N
    % Jiaming handoff: replace this synthetic feedback with estimated state
    % [x;y;vx;vy], in the same world frame, with synchronized timestamps.
    feedback_state(:,k) = C_feedback*plant_state(:,k) + feedback_noise(:,k);
    % Jerry handoff: replace reference_at() with agreed waypoint/trajectory data.
    [reference_state(:,k), reference_acceleration] = reference_at(t(k), cfg);
    error_state = feedback_state(:,k)-reference_state(:,k);
    % Feedforward is zero for the current position-hold references. For a
    % smooth trajectory, supply consistent velocity AND acceleration refs.
    accel_ff = reference_acceleration + cfg.beta_design.*reference_state(3:4,k);
    u_ff = [-accel_ff(2); -accel_ff(1)]/cfg.g;
    unlimited_command(:,k) = u_ff-K*error_state;
    command(:,k) = max(-angle_limit, min(angle_limit, unlimited_command(:,k)));
    if cfg.attitude_tau > 0
        actual_angles(:,k) = plant_state(5:6,k);
    else
        actual_angles(:,k) = command(:,k);
    end
    if k < N
        plant_state(:,k+1) = Ap*plant_state(:,k) + Bp*command(:,k);
    end
end

%% 6. Metrics use simulated true state, not the noisy controller feedback.
state = C_feedback*plant_state;
position_error = state(1:2,:)-reference_state(1:2,:);
metrics.rmse_xy = sqrt(mean(position_error.^2,2));
metrics.rmse_2d = sqrt(mean(sum(position_error.^2,1)));
metrics.final_position_error = norm(position_error(:,end));
metrics.final_speed = norm(state(3:4,end));
metrics.peak_command_deg = max(abs(command),[],2)*180/pi;
metrics.saturation_percent = 100*mean(any(abs(unlimited_command(:,1:end-1)) > angle_limit+1e-12,1));
metrics.control_effort = cfg.Ts*sum(sum(command(:,1:end-1).^2)); % rad^2*s
metrics.waypoint_end_error = zeros(numel(cfg.waypoint_times),1);
metrics.settling_time = nan(numel(cfg.waypoint_times),1);
for j = 1:numel(cfg.waypoint_times)
    if j < numel(cfg.waypoint_times)
        idx = find(t >= cfg.waypoint_times(j) & t < cfg.waypoint_times(j+1));
    else
        idx = find(t >= cfg.waypoint_times(j));
    end
    e = state(1:2,idx)-cfg.waypoint_xy(j,:).';
    good = sqrt(sum(e.^2,1)) <= 0.02 & sqrt(sum(state(3:4,idx).^2,1)) <= 0.05;
    last_bad = find(~good,1,'last');
    if isempty(last_bad)
        metrics.settling_time(j) = t(idx(1))-cfg.waypoint_times(j);
    elseif last_bad < numel(idx)
        metrics.settling_time(j) = t(idx(last_bad+1))-cfg.waypoint_times(j);
    end
    metrics.waypoint_end_error(j) = norm(e(:,end));
end
assert(all(isfinite(state(:))) && all(isfinite(command(:))), 'Non-finite response.');
assert(max(abs(command(:))) <= angle_limit+1e-12, 'Command limit violated.');

fprintf('Horizontal LQR | %s\n', cfg.parameter_status);
fprintf('Design drag [%.3g %.3g] 1/s; plant drag [%.3g %.3g] 1/s; lag %.3g s\n', ...
    cfg.beta_design, cfg.beta_plant, cfg.attitude_tau);
fprintf('Controllability rank: %d / 4\n', controllability_rank);
disp('K (rows: roll, pitch; columns: x, y, vx, vy) ='); disp(K);
fprintf('Riccati iterations: %d; relative residual: %.3e\n', riccati_iterations, riccati_residual);
fprintf('Maximum pole magnitude: design %.6f, plant with lag %.6f\n', ...
    max(abs(closed_loop_poles)), max(abs(plant_closed_loop_poles)));
fprintf('Position RMSE: x %.4f m, y %.4f m, 2D %.4f m (includes waypoint steps)\n', ...
    metrics.rmse_xy, metrics.rmse_2d);
fprintf('Final position error: %.6f m; final speed: %.6f m/s\n', ...
    metrics.final_position_error, metrics.final_speed);
fprintf('Peak roll/pitch command: %.3f / %.3f deg; saturated samples: %.2f %%\n', ...
    metrics.peak_command_deg, metrics.saturation_percent);
fprintf('Waypoint end errors (m):'); fprintf(' %.5f', metrics.waypoint_end_error); fprintf('\n');
fprintf('Settling times (s; 2 cm / 0.05 m/s bands; NaN = not settled):');
fprintf(' %.2f', metrics.settling_time); fprintf('\n');
fprintf('Model/solver checks passed. These are synthetic simulations, not flight validation.\n');

% Keep useful outputs together for review and future integration.
results = struct('time',t,'state',state,'feedback',feedback_state, ...
    'reference',reference_state,'command',command,'actual_angles',actual_angles, ...
    'K',K,'P',P,'Ad',Ad,'Bd',Bd,'metrics',metrics,'config',cfg);

%% 7. One figure: route, position tracking, velocity and attitude commands.
model_figure = figure('Name','Horizontal LQR - assumed parameters', ...
    'NumberTitle','off','Color','w','Position',[80 60 1150 760]);
subplot(2,2,1);
plot(cfg.waypoint_xy(:,1),cfg.waypoint_xy(:,2),'k--o','LineWidth',1.2); hold on;
plot(state(1,:),state(2,:),'b-','LineWidth',1.5);
plot(state(1,1),state(2,1),'go','MarkerFaceColor','g');
grid on; axis equal; xlabel('x (m)'); ylabel('y (m)');
title('Waypoint sequence and simulated path'); legend('Waypoint order','LQR path','Start','Location','best');
subplot(2,2,2);
plot(t,state(1,:),'b-',t,reference_state(1,:),'b--', ...
    t,state(2,:),'r-',t,reference_state(2,:),'r--','LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Position (m)');
title('Position tracking'); legend('x','x reference','y','y reference','Location','best');
subplot(2,2,3);
plot(t,state(3,:),'b-',t,state(4,:),'r-','LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity (hold references: zero)'); legend('vx','vy','Location','best');
subplot(2,2,4);
plot(t,command(1,:)*180/pi,'b-',t,command(2,:)*180/pi,'r-', ...
    t,actual_angles(1,:)*180/pi,'b--',t,actual_angles(2,:)*180/pi,'r--','LineWidth',1.0);
hold on;
plot([t(1) t(end)],[1 1]*cfg.angle_limit_deg,'k:','HandleVisibility','off');
plot([t(1) t(end)],-[1 1]*cfg.angle_limit_deg,'k:','HandleVisibility','off');
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Commands, limits and assumed attitude response');
legend('Roll cmd','Pitch cmd','Roll actual (sim)','Pitch actual (sim)','Location','best');
sgtitle({'LQR position tracking with linear drag',cfg.parameter_status},'Interpreter','none');

if cfg.show_open_loop
    plot_open_loop(Ad,Bd,cfg);
end

function validate_config(c)
    validateattributes(c.g,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(c.Ts,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(c.duration,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(c.beta_design,{'numeric'},{'real','finite','size',[2 1],'nonnegative'});
    validateattributes(c.beta_plant,{'numeric'},{'real','finite','size',[2 1],'nonnegative'});
    validateattributes(c.attitude_tau,{'numeric'},{'real','finite','scalar','nonnegative'});
    validateattributes(c.angle_limit_deg,{'numeric'},{'real','finite','scalar','positive'});
    validateattributes(c.noise_std,{'numeric'},{'real','finite','size',[4 1],'nonnegative'});
    validateattributes(c.initial_state,{'numeric'},{'real','finite','size',[4 1]});
    validateattributes(c.initial_angles,{'numeric'},{'real','finite','size',[2 1]});
    validateattributes(c.Q,{'numeric'},{'real','finite','size',[4 4]});
    validateattributes(c.R,{'numeric'},{'real','finite','size',[2 2]});
    assert(norm(c.Q-c.Q.','fro')<1e-12 && min(eig(c.Q))>=0,'Q must be symmetric PSD.');
    assert(norm(c.R-c.R.','fro')<1e-12 && min(eig(c.R))>0,'R must be symmetric PD.');
    validateattributes(c.waypoint_times,{'numeric'},{'real','finite','vector','nonnegative'});
    validateattributes(c.waypoint_xy,{'numeric'},{'real','finite','size',[numel(c.waypoint_times) 2]});
    assert(c.waypoint_times(1)==0 && all(diff(c.waypoint_times)>0),'Waypoints must start at 0 and have increasing times.');
    assert(c.waypoint_times(end)<c.duration,'Simulation must continue after final waypoint.');
end

function [A,B] = horizontal_model(g,beta)
    A = [zeros(2) eye(2); zeros(2) -diag(beta)];
    B = [0 0; 0 0; 0 -g; -g 0];
end

function [Ad,Bd] = exact_zoh(A,B,Ts)
    n = size(A,1); m = size(B,2);
    E = expm([A B; zeros(m,n+m)]*Ts);
    Ad = E(1:n,1:n); Bd = E(1:n,n+1:n+m);
end

function [K,P,iterations,residual] = solve_discrete_lqr(A,B,Q,R)
% Finite-horizon Riccati iteration to the stabilizing infinite-horizon limit.
% It is checked for convergence, DARE residual and closed-loop stability.
    P = Q;
    converged = false;
    for iterations = 1:20000
        K_step = (R+B.'*P*B)\(B.'*P*A);
        P_next = Q+A.'*P*A-A.'*P*B*K_step;
        P_next = (P_next+P_next.')/2;
        change = norm(P_next-P,'fro')/max(1,norm(P_next,'fro'));
        P = P_next;
        if change < 1e-12
            converged = true; break;
        end
    end
    assert(converged,'Riccati iteration did not converge; check model and Q/R.');
    K = (R+B.'*P*B)\(B.'*P*A);
    residual = norm(Q+A.'*P*A-P-A.'*P*B*K,'fro')/max(1,norm(P,'fro'));
    assert(residual < 1e-9 && min(eig(P)) >= -1e-9,'Invalid Riccati solution.');
end

function [reference,acceleration] = reference_at(time,c)
% Jerry placeholder: piecewise-constant position goals, zero velocity/accel.
% This is waypoint position hold, not a smooth reference trajectory.
    idx = find(time >= c.waypoint_times,1,'last');
    reference = [c.waypoint_xy(idx,:).'; 0; 0];
    acceleration = zeros(2,1);
end

function plot_open_loop(Ad,Bd,c)
% Retain the old +/- drag comparison as an optional secondary figure.
    t = 0:c.Ts:1; a = 2*pi/180;
    pitch = zeros(4,numel(t)); roll = pitch;
    for k=1:numel(t)-1
        pitch(:,k+1)=Ad*pitch(:,k)+Bd*[0;a];
        roll(:,k+1)=Ad*roll(:,k)+Bd*[a;0];
    end
    p0=-0.5*c.g*a*t.^2; v0=-c.g*a*t;
    figure('Name','Open-loop drag comparison','NumberTitle','off','Color','w');
    subplot(2,2,1); plot(t,pitch(1,:),t,p0,'--'); grid on; title('Pitch: x'); ylabel('m');
    subplot(2,2,2); plot(t,roll(2,:),t,p0,'--'); grid on; title('Roll: y'); ylabel('m');
    subplot(2,2,3); plot(t,pitch(3,:),t,v0,'--'); grid on; title('Pitch: vx'); ylabel('m/s'); xlabel('s');
    subplot(2,2,4); plot(t,roll(4,:),t,v0,'--'); grid on; title('Roll: vy'); ylabel('m/s'); xlabel('s');
    legend('With drag','No drag');
end