function tests = test_safety
%TEST_SAFETY  Monitor, arbitration, recovery and sensor/compute fault tests.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
addpath(fileparts(fileparts(mfilename('fullpath')))); setup_paths();
tc.TestData.p = default_params();
tc.TestData.p.ranger.noise = 0;  % written for ideal ranging; bench-measured noise is now the default
end

function testStoppingDistanceMatchesPlant(tc)
p = tc.TestData.p;
for speed = [0 0.03 0.3 0.5 0.53 1]
    x = [0;0;speed;0];
    for k = 1:400
        x = dynamics_step(x,0,[0;0],0,[0;0],p);
    end
    verifyEqual(tc,stopping_distance(speed,p),x(1),'AbsTol',1e-10);
end
end

function testNoReturnAndMissingAreDifferent(tc)
p = tc.TestData.p; x = [2;2;0;0];
health = struct('primary_time',0.1,'navigator_time',0.1,'navigator_required',true);
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,0);
r = backup_monitor(f,x,0,[0;0],0.1,health,p);
verifyTrue(tc,r.trip); verifyEqual(tc,r.reason,'waiting_for_ranges');
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,1);
r = backup_monitor(f,x,0,[0;0],0.1,health,p);
verifyFalse(tc,r.trip); verifyEqual(tc,f.status,uint8(2*ones(4,1)));
f.ranges(1) = NaN;
r = backup_monitor(f,x,0,[0;0],0.1,health,p);
verifyTrue(tc,r.fault); verifyEqual(tc,r.reason,'invalid_raw_frame');
end

function testRepeatedFramesDoNotAdvanceFilterOrRecovery(tc)
p = tc.TestData.p; x = [2;2;0;0];
f = ranger_frame(ones(4,1),0,0.1,x(1:2),0,1);
[a,h] = range_processing(f,[],0.1,p);
[b,h] = range_processing(f,h,0.12,p);
verifyTrue(tc,a.new_frame); verifyFalse(tc,b.new_frame);
verifyEqual(tc,numel(h.frames),1);
s = safety_init(x,0);
primary = struct('trip',false); backup = struct('trip',false);
for k = 1:20
    [s,accept] = command_arbiter(s,primary,backup,x,[0;0],k*p.dt,1,p);
    verifyFalse(tc,accept);
end
verifyEqual(tc,s.state,'HOLD'); verifyEqual(tc,s.clear_count,1);
end

function testCandidateAccelerationIsChecked(tc)
p = tc.TestData.p; x = [2;2;0;0];
f = ranger_frame([0.10;Inf;Inf;Inf],0,0.1,x(1:2),0,1);
[data,~] = range_processing(f,[],0.1,p);
still = primary_monitor(data,x,0,[0;0],0,[0;0],0.1,p);
move = primary_monitor(data,x,0,[0.5;0],0,[0;0],0.1,p);
verifyFalse(tc,still.trip); verifyTrue(tc,move.trip);
verifyEqual(tc,move.reason,'stopping_distance');
end

function testSideRearAndUnseenCommands(tc)
p = tc.TestData.p; x = [2;2;0;0];
for k = [2 3 4]
    ranges = inf(4,1); ranges(k) = 0.10;
    D = [1 0;0 1;-1 0;0 -1];
    f = ranger_frame(ranges,0,0.1,x(1:2),0,1);
    [data,~] = range_processing(f,[],0.1,p);
    r = primary_monitor(data,x,0,0.5*D(k,:)',0,[0;0],0.1,p);
    verifyTrue(tc,r.trip);
end
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,1);
[data,~] = range_processing(f,[],0.1,p);
r = primary_monitor(data,x,0,[0.3;0.3],0,[0;0],0.1,p);
verifyEqual(tc,r.reason,'unobserved_direction');
end

function testBackupBypassesCorruptedPrimaryData(tc)
p = tc.TestData.p; x = [2;2;0.5;0];
f = ranger_frame([0.15;Inf;Inf;Inf],0,0.1,x(1:2),0,1);
[data,~] = range_processing(f,[],0.1,p); data.ranges(:) = Inf;
a = primary_monitor(data,x,0,[0.5;0],0,x(3:4),0.1,p);
health = struct('primary_time',0.1,'navigator_time',0.1,'navigator_required',true);
b = backup_monitor(f,x,0,x(3:4),0.1,health,p);
verifyFalse(tc,a.trip); verifyTrue(tc,b.trip);
verifyEqual(tc,b.reason,'backup_distance');
end

function testBackupCannotBeClearedByPrimary(tc)
p = tc.TestData.p; x = [2;2;0.5;0];
s = safety_init(x,0); s.state = 'NORMAL';
a = struct('trip',false);
b = struct('trip',true,'fault',true,'reason','primary_timeout');
[s,accept] = command_arbiter(s,a,b,x,x(3:4),0.1,1,p);
verifyFalse(tc,accept); verifyEqual(tc,s.state,'BRAKE');
verifyEqual(tc,s.source,'backup'); verifyEqual(tc,s.reason,'primary_timeout');
[u,r] = fallback_step(s,x,0,x(3:4),p);
verifyEqual(tc,u,[0;0]); verifyEqual(tc,r,0);
end

function testRawDataAgeAndHeartbeat(tc)
p = tc.TestData.p; x = [2;2;0;0];
health = struct('primary_time',1,'navigator_time',1,'navigator_required',true);
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,1);
r = backup_monitor(f,x,0,[0;0],0.4,health,p);
verifyEqual(tc,r.reason,'ranger_timeout');
f = ranger_frame(inf(4,1),0.9,1,x(1:2),0,2);
health.primary_time = 0.8;
r = backup_monitor(f,x,0,[0;0],1,health,p);
verifyEqual(tc,r.reason,'primary_timeout');
health.primary_time = 1; health.navigator_time = 0.8;
r = backup_monitor(f,x,0,[0;0],1,health,p);
verifyEqual(tc,r.reason,'navigator_timeout');
end

function testUnsafeCandidateDoesNotCommitNavigation(tc)
p = tc.TestData.p; x = [2;2;0.1;0];
p.nav.d_sweep = 0.03; p.nav.d_wall = 0.02;
nav = navigator_init(x(1:2)',[2.15 2]);
s = safety_init(x,0); s.state = 'NORMAL';
s.last_pos = x(1:2)-0.1*x(3:4); % consistent observed ground velocity
f = ranger_frame([0.12;Inf;Inf;Inf],0,0.1,x(1:2),0,1);
[~,~,after,s,d] = safety_step(nav,s,x,0,f,0.1,p);
verifyFalse(tc,d.accepted); verifyEqual(tc,after,nav);
verifyEqual(tc,s.state,'BRAKE');
end

function testResumePreservesMissionAndEpisode(tc)
p = tc.TestData.p; %#ok<NASGU>
x = [2;2;0;0]; nav = navigator_init([1 2],[5 2;8 2]);
nav.episode = true; nav.side = -1; nav.hit_dist = 4;
nav.hit_point = [1;2]; nav.sweep_spots = [1 2 0];
nav.ep_travel = 1; nav.scan.rng = 1; nav.subgoal = [3;2];
after = navigator_resume(nav,x,0,1);
verifyEqual(tc,after.wp,nav.wp); verifyEqual(tc,after.targets,nav.targets);
verifyEqual(tc,after.side,-1); verifyEqual(tc,after.sweep_spots,nav.sweep_spots);
verifyEqual(tc,after.last_pos,x(1:2)); verifyTrue(tc,isempty(after.scan.rng));
verifyTrue(tc,all(isnan(after.subgoal))); verifyEqual(tc,after.phase,'sweep');
end

function testNominalClosedLoopShapes(tc)
p = tc.TestData.p;
shapes = {{obstacle_square([5 4],1.5)}, ...
    {obstacle_rect([5 4.2],[0.2 6.4])}, ...
    {obstacle_rect([5 4],[2.5 0.5],30)}, ...
    {obstacle_circle([5 4],1)}, ...
    {obstacle_circle([5 4],0.2)}, ...
    {obstacle_rect([5 2.3],[2 0.6]),obstacle_rect([5.7 4],[0.6 4])}, ...
    {obstacle_rect([5.5 2.3],[3 0.6]),obstacle_rect([5.5 5.7],[3 0.6]),obstacle_rect([6.7 4],[0.6 4])}};
starts = [1 4;2 4;1 3;1 4;1 4;1 1;1 4];
goals = [9 4.3;8 3.5;9 5;9 4.2;9 4;8 5;8.5 4];
for k = 1:numel(shapes)
    env = make_environment(p,shapes{k});
    L = run_simulation(starts(k,:),[],goals(k,:),env,p);
    verifyTrue(tc,L.arrived,sprintf('shape %d',k)); verifyFalse(tc,L.collided);
    verifyEqual(tc,L.safety_last_error,'');
    verifyTrue(tc,all(vecnorm(L.u_body(~L.command_accepted,:),2,2) <= p.safety.hold_speed+1e-9));
end
end

function testMainMissionWithNoise(tc)
p = tc.TestData.p; p.ranger.noise = 0.1;
obstacles = {obstacle_square([2.5 3],1),obstacle_rect([5.5 4],[0.2 3.4]), ...
    obstacle_circle([8 2.9],0.3),obstacle_rect([7.6 5.3],[1.6 0.5])};
env = make_environment(p,obstacles);
for seed = 1:3
    p.seed = seed;
    L = run_simulation([0.8 0.8],[3.4 4.2;6.6 2;9.4 3.8],[6.4 6.4],env,p);
    verifyTrue(tc,L.arrived,sprintf('seed %d',seed)); verifyFalse(tc,L.collided);
    verifyEmpty(tc,L.unreachable); verifyEqual(tc,L.safety_last_error,'');
end
end

function testFaultsOverrideAndRecover(tc)
p = tc.TestData.p; p.t_max = 35; env = make_environment(p,{});
fields = {'ranger_dropout','primary_timeout','navigator_timeout'};
reasons = {'ranger_timeout','primary_timeout','navigator_timeout'};
for k = 1:numel(fields)
    faults = struct(); faults.(fields{k}) = [3 4];
    L = run_simulation([1 4],[],[9 4],env,p,faults);
    tripped = strcmp(L.safety_reason,reasons{k});
    verifyTrue(tc,any(tripped),fields{k});
    verifyTrue(tc,all(~L.command_accepted(tripped)));
    verifyTrue(tc,any(strcmp({L.safety_events.type},'stopped')));
    verifyTrue(tc,L.arrived,fields{k}); verifyFalse(tc,L.collided);
    % Recovery must use new frames, not repeated 50 Hz reads of one sample.
    e = L.safety_events(strcmp({L.safety_events.type},'resume'));
    verifyTrue(tc,any([e.t] > 4));
end
end

function testRawBackupStopsWithBiasedFilter(tc)
p = tc.TestData.p; p.t_max = 15;
env = make_environment(p,{obstacle_rect([5 4],[0.2 4])});
faults = struct('processed_bias',[0 15 10]);
L = run_simulation([1 4],[],[9 4],env,p,faults);
verifyTrue(tc,any(strcmp(L.safety_reason,'backup_distance')));
verifyFalse(tc,L.collided); verifyFalse(tc,L.arrived);
verifyLessThan(tc,max(L.x(:,1)),4.9-p.radius);
end

function testHoldCompensatesObservedWind(tc)
p = tc.TestData.p; p.t_max = 8; p.wind = [0.08;0];
env = make_environment(p,{});
L = run_simulation([2 4],[],[9 4],env,p,struct('primary_timeout',[3 8]));
late = L.t > 6;
verifyLessThan(tc,max(vecnorm(L.ground_velocity(late,:),2,2)),p.safety.stop_speed);
verifyLessThan(tc,max(vecnorm(L.u_body(late,:),2,2)),0.12);
verifyFalse(tc,L.collided);
end

function testSequenceRollbackIsRejected(tc)
p = tc.TestData.p; x = [2;2;0;0];
health = struct('primary_time',0.2,'navigator_time',0.2,'navigator_required',true, ...
    'last_seq',2,'last_sample_time',0.1);
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,1);
r = backup_monitor(f,x,0,[0;0],0.2,health,p);
verifyEqual(tc,r.reason,'nonmonotonic_frame'); verifyTrue(tc,r.fault);
end

function testObservedTravelProtectsDelayedDeceleration(tc)
p = tc.TestData.p; x = [2;2;0.03;0];
f = ranger_frame([0.2;Inf;Inf;Inf],0,0.1,[1.9;2],0,1);
[data,~] = range_processing(f,[],0.2,p);
r = primary_monitor(data,x,0,[0;0],0,x(3:4),0.2,p);
verifyGreaterThanOrEqual(tc,r.required(1),0.1+p.safety.margin);
verifyTrue(tc,r.trip);
end

function testInvalidProcessedRangeFailsClosed(tc)
p = tc.TestData.p; x = [2;2;0;0];
f = ranger_frame(inf(4,1),0,0.1,x(1:2),0,1);
[data,~] = range_processing(f,[],0.1,p);
for value = [NaN -1 -Inf]
    data.ranges(1) = value;
    r = primary_monitor(data,x,0,[0.3;0],0,[0;0],0.1,p);
    verifyTrue(tc,r.trip); verifyEqual(tc,r.reason,'invalid_processed_ranges');
end
end

function testStationaryScanSurvivesShortOverride(tc)
p = tc.TestData.p; x = [2;2;0;0];
s = safety_init(x,0); s.state = 'NORMAL';
a = struct('trip',false); b = struct('trip',true,'reason','hard_distance');
[s,accept] = command_arbiter(s,a,b,x,[0;0],0.1,1,p);
verifyFalse(tc,accept); verifyFalse(tc,s.need_replan);
b.trip = false;
for k = 2:4
    [s,~,replan] = command_arbiter(s,a,b,x,[0;0],k*0.1,k,p);
end
verifyEqual(tc,s.state,'RECOVER'); verifyFalse(tc,replan);
end

function testStartupAndPermanentDropoutStayStopped(tc)
p = tc.TestData.p; p.t_max = 2;
env = make_environment(p,{});
L = run_simulation([2 4],[],[9 4],env,p,struct('ranger_dropout',[0 Inf]));
verifyEqual(tc,L.x(:,1:2),repmat([2 4],numel(L.t),1),'AbsTol',1e-12);
verifyFalse(tc,any(L.command_accepted)); verifyFalse(tc,L.arrived);
end

function testTimestampedSweepAndVisualizations(tc)
p = tc.TestData.p;
env = make_environment(p,{obstacle_square([5 4],1.5)});
L = run_simulation([1 4],[],[9 4],env,p);
for sweep = L.sweeps
    verifyTrue(tc,all(diff(sweep.t(1:3:end)) > 0));
    rel = rad2deg(wrap_to_pi(sweep.ang-sweep.center));
    verifyEqual(tc,[min(rel) max(rel)],[-135 135],'AbsTol',2);
end
old = get(groot,'defaultFigureVisible'); set(groot,'defaultFigureVisible','off');
cleanup = onCleanup(@() set(groot,'defaultFigureVisible',old)); %#ok<NASGU>
f1 = plot_results(L,env,p,compute_metrics(L,env,p));
f2 = plot_safety(L,p);
f3 = animate_run(L,env,p,struct('autoplay',false));
verifyTrue(tc,all(isgraphics([f1 f2 f3]))); close([f1 f2 f3]);
end
