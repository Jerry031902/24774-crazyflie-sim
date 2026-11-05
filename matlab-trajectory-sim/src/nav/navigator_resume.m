function nav = navigator_resume(nav, x, psi, t)
%NAVIGATOR_RESUME  Replan after an override; retain target and episode history.
nav.last_pos = x(1:2);
nav.facing = false;
if strcmp(nav.mode,'DONE'), return; end
if ~nav.episode
    nav.episode = true;
    nav.hit_point = x(1:2);
    nav.hit_dist = norm(nav.targets(nav.wp,:)'-x(1:2));
    nav.side = 0;
    nav.ep_travel = 0;
    nav.sweep_spots = zeros(0,3);
    d = x(1:2)-nav.targets(nav.wp,:)';
    nav.winding = 0; nav.last_bearing = atan2(d(2),d(1));
    nav.events(end+1) = struct('t',t,'type','hit','pos',x(1:2)','wp',nav.wp);
else
    d = x(1:2)-nav.targets(nav.wp,:)';
    nav.last_bearing = atan2(d(2),d(1));
end
nav.mode = 'AVOID'; nav.phase = 'sweep'; nav.sweep_stage = 'stop';
nav.sweep_center = psi;
nav.scan = struct('t',[],'ang',[],'rng',[]);
nav.subgoal = [NaN;NaN];
nav.following = false; nav.wall_seen = false;
nav.since_turn = 0; nav.overshoot_left = 0;
nav.scan_seq = 0;
end
