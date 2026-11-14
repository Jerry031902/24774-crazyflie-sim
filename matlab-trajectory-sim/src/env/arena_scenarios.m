function scenarios = arena_scenarios(names)
%ARENA_SCENARIOS  Start-and-goal scenarios with obstacles, for DEMO_ARENA_GRID.
%   scenarios = ARENA_SCENARIOS() returns every scenario below as a struct
%   array. scenarios = ARENA_SCENARIOS({'center', 'wall'}) returns only
%   those, in that order. Each scenario has
%     name        short label, used as the plot title
%     arena       name of the obstacle layout
%     xlim, ylim  room walls [m]; the walls are obstacles the rangers see
%     start, goal [x y] [m]; there are no waypoints
%     obstacles   cell array of OBSTACLE_SQUARE / OBSTACLE_RECT / OBSTACLE_CIRCLE
%     labels      a name for each obstacle ('' for none), drawn on the plot
%
%   The first three are the Isaac Sim arena (sim/build_scene.py): a 6 x 6 m
%   room, inner wall faces at +/-2.95 m, and four 0.30 m pillars, flown in
%   +x from x = -2.4 to x = 2.4 along three lanes:
%     center  y = 0
%     lower   y = -0.45 (Box01's row)
%     upper   y = 0.55  (Box02's row)
%   The rest use the same room with other obstacles, to show the other
%   ways AVOID gets round:
%     wall    a long wall across the way; the sweep sees its end and the
%             drone flies to a pass point round it
%     cup     a U open toward the start; inside it the obstacle runs past
%             the sweep on both sides, so the drone follows the wall out
%     slalom  a diagonal across the room through a row of pillars
%
%   To add a scenario, append another scenario(...) line below.

room = [-2.95 2.95];               % sim/build_scene.py: 6 m outside, 0.05 m walls
box  = 0.30;                       % pillar side [m]
arena_boxes  = {obstacle_square([-1.5 -0.45], box), obstacle_square([-0.5  0.55], box), ...
                obstacle_square([ 0.5 -0.60], box), obstacle_square([ 1.5  0.40], box)};
arena_labels = {'Box01', 'Box02', 'Box03', 'Box04'};

s = [ ...
    scenario('center', 'Original arena', room, [-2.4  0.00], [2.4  0.00], arena_boxes, arena_labels)
    scenario('lower',  'Original arena', room, [-2.4 -0.45], [2.4 -0.45], arena_boxes, arena_labels)
    scenario('upper',  'Original arena', room, [-2.4  0.55], [2.4  0.55], arena_boxes, arena_labels)
    scenario('wall',   'Long wall', room, [-2.4 0], [2.4 0], ...
             {obstacle_rect([0 -0.4], [0.2 3.0])}, {'Wall'})
    scenario('cup',    'U-shaped cup', room, [-2.4 0], [2.4 0], ...
             {obstacle_rect([1.0 0], [0.2 2.2]), ...        % back
              obstacle_rect([0 1.0], [2.2 0.2]), ...        % top arm
              obstacle_rect([0 -1.0], [2.2 0.2])}, ...      % bottom arm
             {'Cup', '', ''})
    scenario('slalom', 'Pillar row', room, [-2.4 -2.4], [2.4 2.4], ...
             {obstacle_square([-1.2 -1.2], 0.5), obstacle_circle([0 0], 0.35), ...
              obstacle_square([1.2 1.2], 0.5, 45)}, {'P1', 'P2', 'P3'})
    ];

if nargin < 1 || isempty(names)
    scenarios = s;
    return
end
names = cellstr(names);
[found, idx] = ismember(names, {s.name});
if ~all(found)
    error('arena_scenarios:unknown', 'Unknown scenario(s): %s. Known: %s.', ...
        strjoin(names(~found), ', '), strjoin({s.name}, ', '));
end
scenarios = s(idx);
end

function sc = scenario(name, arena, room, start, goal, obstacles, labels)
sc = struct('name', name, 'arena', arena, 'xlim', room, 'ylim', room, ...
            'start', start, 'goal', goal, 'obstacles', {obstacles}, 'labels', {labels});
end
