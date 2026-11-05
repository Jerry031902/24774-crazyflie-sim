function tests = test_popup
tests = functiontests(localfunctions);
end

function setupOnce(tc)
addpath(fileparts(fileparts(mfilename('fullpath')))); setup_paths();
tc.TestData.p = default_params();
end

function testGeometryAndClearanceRespectAppearance(tc)
p = tc.TestData.p; env = make_environment(p,{});
env = schedule_obstacle(env,obstacle_square([5 4],1),2);
before = environment_at(env,1.99); after = environment_at(env,2);
verifyEmpty(tc,before.obstacles); verifyEqual(tc,numel(after.obstacles),1);
r0 = read_rangers([3;4],0,before,p); r1 = read_rangers([3;4],0,after,p);
verifyTrue(tc,isinf(r0(1))); verifyLessThan(tc,r1(1),1.5);
d = obstacle_distance(env,[5 4;5 4],[1;2]);
verifyGreaterThan(tc,d(1),0); verifyLessThan(tc,d(2),0);
verifyError(tc,@() obstacle_distance(env,[5 4]),'obstacle_distance:timeRequired');
end

function testUnsortedEventsAndGridActivation(tc)
p = tc.TestData.p; p.t_max = 0.4;
env = schedule_obstacle(make_environment(p,{}),obstacle_square([7 6],0.2),0.211);
env = schedule_obstacle(env,obstacle_square([6 6],0.2),0.1);
L = run_simulation([1 4],[],[9 4],env,p);
verifyEqual(tc,[L.obstacle_events.id],[2 1]);
verifyEqual(tc,[L.obstacle_events.t],[0.1 0.22],'AbsTol',1e-12);
end

function testQueuedMeasurementsDoNotSeeFutureGeometry(tc)
p = tc.TestData.p; p.t_max = 4.3;
scene = popup_scenario(0.55,4.02,p);
L = run_simulation(scene.start,[],scene.goal,scene.env,p);
old = L.t < 4.2-1e-9;
verifyTrue(tc,all(isinf(L.ranges(old,1))));
i = find(L.t >= 4.1-1e-9,1);
verifyEqual(tc,L.ranger_sample_time(i),4,'AbsTol',1e-12);
verifyTrue(tc,isinf(L.ranges(i,1))); % 4.00-s sample delivered after appearance
i = find(L.t >= 4.2-1e-9,1);
verifyEqual(tc,L.ranger_sample_time(i),4.1,'AbsTol',1e-12);
verifyTrue(tc,isfinite(L.ranges(i,1)));
verifyFalse(tc,any(L.backup_trip(L.t > 3 & L.t < 4.2-1e-9)));
end

function testNoScheduleLeakBeforeFirstFreshMeasurement(tc)
p = tc.TestData.p; p.t_max = 4.3;
scene = popup_scenario(0.55,4.02,p);
reference = run_simulation(scene.start,[],scene.goal,make_environment(p,{}),p);
L = run_simulation(scene.start,[],scene.goal,scene.env,p);
before_fresh = L.t < 4.2-1e-9;
verifyEqual(tc,L.x(before_fresh,:),reference.x(before_fresh,:),'AbsTol',1e-12);
verifyEqual(tc,L.u_body(before_fresh,:),reference.u_body(before_fresh,:),'AbsTol',1e-12);
verifyEqual(tc,L.safety_state(before_fresh),reference.safety_state(before_fresh));
end

function testMatchedOnOffEncounters(tc)
p = tc.TestData.p; p.t_max = 4.1;
for enabled = [false true]
    p.safety.enabled = enabled;
    scene = popup_scenario(0.24,4.02,p);
    L = run_simulation(scene.start,[],scene.goal,scene.env,p);
    e = L.obstacle_events(1);
    verifyEqual(tc,e.gap,0.24,'AbsTol',1e-10);
    verifyEqual(tc,e.pos,scene.expected_pos,'AbsTol',1e-12);
    verifyEqual(tc,e.speed,0.5,'AbsTol',0.002);
end
end

function testAmpleDistanceStopsAndCompletes(tc)
p = tc.TestData.p; p.t_max = 90;
scene = popup_scenario(0.55,4.02,p);
L = run_simulation(scene.start,[],scene.goal,scene.env,p);
r = popup_summary(L,scene.env,p);
verifyFalse(tc,L.collided); verifyTrue(tc,L.arrived);
verifyTrue(tc,isfinite(r.first_stopped_s)); verifyGreaterThan(tc,r.gap_at_stop_m,0);
verifyGreaterThanOrEqual(tc,r.first_zero_command_s,r.first_fresh_delivery_s-1e-9);
end

function testCriticalBoundaryOfExistingZeroTargetFallback(tc)
p = tc.TestData.p; p.t_max = 15;
for gap = [0.18 0.24]
    scene = popup_scenario(gap,4.02,p);
    L = run_simulation(scene.start,[],scene.goal,scene.env,p);
    r = popup_summary(L,scene.env,p);
    verifyEqual(tc,r.first_override_s,4.2,'AbsTol',1e-10);
    verifyEqual(tc,r.first_zero_command_s,4.2,'AbsTol',1e-10);
    verifyEqual(tc,L.collided,gap < r.zero_target_required_gap_m);
    verifyEqual(tc,r.zero_target_stop_m,0.115,'AbsTol',0.001);
end
end

function testTooCloseCanCollideBeforeAnyFreshFrame(tc)
p = tc.TestData.p; p.t_max = 10;
scene = popup_scenario(0.05,4.02,p);
L = run_simulation(scene.start,[],scene.goal,scene.env,p);
r = popup_summary(L,scene.env,p);
verifyTrue(tc,L.collided); verifyLessThan(tc,L.t_collide,r.expected_first_delivery_s);
verifyTrue(tc,isnan(r.first_fresh_delivery_s)); verifyTrue(tc,isnan(r.first_override_s));
verifyGreaterThan(tc,L.obstacle_events(1).gap,0); % no body overlap at creation
end

function testAppearanceEnclosingBodyIsImmediateCollision(tc)
p = tc.TestData.p; p.t_max = 2;
env = schedule_obstacle(make_environment(p,{}),obstacle_rect([1 4],[1 2]),0.1);
L = run_simulation([1 4],[],[9 4],env,p);
verifyTrue(tc,L.collided); verifyEqual(tc,L.t_collide,0.1,'AbsTol',1e-12);
end

function testMetricsIgnoreFutureObstacle(tc)
p = tc.TestData.p; p.t_max = 0.4;
env = schedule_obstacle(make_environment(p,{}),obstacle_square([1 4],1),10);
L = run_simulation([1 4],[],[9 4],env,p);
m = compute_metrics(L,env,p);
verifyFalse(tc,m.collided); verifyGreaterThan(tc,m.min_clearance,0.8);
end

function testPlaybackVisibilityAndPopupPlot(tc)
p = tc.TestData.p; p.t_max = 4.4;
scene = popup_scenario(0.55,4.02,p);
L = run_simulation(scene.start,[],scene.goal,scene.env,p);
old = get(groot,'defaultFigureVisible'); set(groot,'defaultFigureVisible','off');
cleanup = onCleanup(@() set(groot,'defaultFigureVisible',old)); %#ok<NASGU>
f0 = animate_run(L,scene.env,p,struct('autoplay',false,'start_time',4));
h = findall(f0,'Tag','scheduled_obstacle'); verifyEqual(tc,char(h.Visible),'off');
f1 = animate_run(L,scene.env,p,struct('autoplay',false,'start_time',4.02));
h = findall(f1,'Tag','scheduled_obstacle'); verifyEqual(tc,char(h.Visible),'on');
f2 = plot_popup(L,scene.env,p);
row = popup_summary(L,scene.env,p); row.requested_gap_m = 0.55;
f3 = plot_popup_boundary(struct2table(row),4.02);
lines = findall(f3,'Type','Line');
verifyNotEmpty(tc,lines); % guards against an empty panel from float equality
verifyTrue(tc,isgraphics(f2)); close([f0 f1 f2 f3]);
end
