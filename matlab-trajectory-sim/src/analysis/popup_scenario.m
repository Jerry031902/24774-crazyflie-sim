function scene = popup_scenario(gap, appear_time, p)
%POPUP_SCENARIO  Matched cruising encounter with a time-triggered static box.
% gap is body-to-obstacle clearance when the box appears, not sensor range.
% An obstacle-free reference locates the box OFFLINE. Its world coordinates
% are then fixed; neither the navigator nor safety sees the spawn schedule.
% Each safety mode is calibrated separately to match encounter gap/speed,
% since startup waiting/recovery gives the modes different pre-spawn positions.
if nargin < 3, p = default_params(); end
if nargin < 2, appear_time = 4.02; end
validateattributes(gap,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(appear_time,{'numeric'},{'scalar','real','finite','nonnegative'});
assert(all(p.wind==0) && p.wind_gust==0,'popup_scenario:wind', ...
    'The matched scalar stopping benchmark assumes no wind.');
scene.start = [1 4]; scene.waypoints = []; scene.goal = [9 4];
scene.p = p;
scene.appear_time = appear_time;
scene.effective_time = ceil((appear_time-1e-9)/p.dt)*p.dt;
empty = make_environment(p,{});
reference_p = p; reference_p.t_max = max(scene.effective_time,p.dt);
reference = run_simulation(scene.start,[],scene.goal,empty,reference_p);
i = find(reference.t >= scene.effective_time-1e-9,1);
assert(~isempty(i),'popup_scenario:finished','Choose a time before the reference mission finishes.');
scene.expected_pos = reference.x(i,1:2);
scene.expected_speed = norm(reference.x(i,3:4));
thickness = 0.12;
front_x = scene.expected_pos(1)+p.radius+gap;
scene.obstacle = obstacle_rect([front_x+thickness/2,4],[thickness,1.2]);
scene.env = schedule_obstacle(empty,scene.obstacle,appear_time);
scene.requested_gap = gap;
end
