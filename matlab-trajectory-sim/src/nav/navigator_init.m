function nav = navigator_init(start, targets)
%NAVIGATOR_INIT  Initial navigator state for NAVIGATOR_STEP.
%   nav = NAVIGATOR_INIT(start, targets) flies from start ([x y]) through
%   the rows of targets (waypoints, then the goal) in order.

nav.targets = targets;
nav.wp      = 1;              % index of the current target
nav.mode    = 'GO_TO_GOAL';   % 'GO_TO_GOAL' | 'AVOID' | 'DONE'
nav.phase   = '';             % AVOID: 'sweep' | 'pass' | 'align' | 'follow' | 'overshoot'

% Motion. Directions are 1 forward, 2 left, 3 back, 4 right (body frame);
% 0 means "straight at the point", used only for the last few cm.
nav.facing     = false;       % lined up with the point being flown to
nav.exec_dir   = 1;           % direction being flown
nav.last_pos   = start(:);

% Avoidance episode: from first contact with an obstacle until the drone
% is closer to the target than where that contact happened.
nav.episode     = false;
nav.hit_point   = [NaN; NaN]; % where the episode started
nav.hit_dist    = Inf;        % distance from there to the target
nav.side        = 0;          % way round chosen by the first sweep: +1 left, -1 right
nav.ep_travel   = 0;          % distance flown during the episode
nav.sweep_spots = zeros(0, 3);% [x y ep_travel] at each sweep of the episode
nav.winding     = 0;          % angle the drone has circled round the target [rad]
nav.last_bearing = 0;         % bearing from the target to the drone last tick [rad]

% Sweep (the wedge scan)
nav.sweep_stage  = '';        % 'stop' | 'to_start' | 'scan'
nav.sweep_center = 0;         % heading the sweep is centred on [rad]
nav.scan         = struct('t', [], 'ang', [], 'rng', []);
nav.subgoal      = [NaN; NaN];% point just outside the chosen edge

% Wall-following (when the chosen edge lies outside the wedge)
nav.hand           = 1;          % +1 wall on the left, -1 wall on the right
nav.travel_dir     = 1;
nav.psi_hold       = 0;          % yaw held while following, on a room axis
nav.following      = false;      % true once the first turn has put the wall beside the drone
nav.loop_point     = [NaN; NaN]; % position of that first turn
nav.travel         = 0;          % distance flown since then
nav.since_turn     = 0;          % distance flown since the last turn
nav.wall_seen      = false;      % the wall-side ranger has seen the wall since the last turn
nav.overshoot_left = 0;          % remaining travel past a wall's end
nav.d_followed     = inf;        % closest the followed wall has come to the target

nav.unreachable = zeros(1, 0);
nav.events = struct('t', {}, 'type', {}, 'pos', {}, 'wp', {});
nav.sweeps = struct('t', {}, 'ang', {}, 'rng', {}, 'pos', {}, 'center', {}, ...
                    'edge', {}, 'seen', {}, 'cost', {}, 'side', {}, 'pass', {});
end
