function fig = animate_run(sim_log, env, p, opts)
%ANIMATE_RUN  Play back a finished run with its sensor rays; replay on demand.
%   fig = ANIMATE_RUN(sim_log, env, p) opens the playback figure and plays
%   the run once. Click Replay to watch it again (the button reads Stop
%   while playing) and pick the speed from the menu next to it.
%
%   Each frame shows the four ranger rays (solid to the reading, dotted
%   to full range when there is no return), the path so far coloured by
%   mode and the current target. During an avoidance episode it also shows
%   the hit point, the points of the latest sweep as they are scanned, the
%   pass point the drone is heading for, and the leave zone: the drone
%   leaves a wall only inside this circle, closer to the target than the
%   hit point.
%
%   ANIMATE_RUN(..., opts) takes any of these fields:
%     speed       starting playback speed, multiple of real time (default 4)
%     fps         frame rate (default 30)
%     video_file  record the first playback to this MPEG-4 file (default '')
%     icon_scale  drone icon size relative to its body radius (default 2.5)

defaults = struct('speed', 4, 'fps', 30, 'video_file', '', 'icon_scale', 2.5);
if nargin < 4, opts = struct(); end
for f = fieldnames(defaults)'
    if ~isfield(opts, f{1}), opts.(f{1}) = defaults.(f{1}); end
end

c      = viz_colors();
pos    = sim_log.x(:, 1:2);
K      = numel(sim_log.t);
modes  = {'GO_TO_GOAL', 'AVOID', 'DONE'};
speeds = [1 2 4 8 16];
[~, speed_idx] = min(abs(speeds - opts.speed));

fig = figure('Name', 'Drone playback', 'Color', c.surface, 'Position', [120 80 940 720]);
ax  = axes(fig, 'Position', [0.08 0.17 0.88 0.76]);
plot_environment(ax, env);
plot_route(ax, sim_log);

h_go    = plot(ax, NaN, NaN, '-', 'Color', c.go, 'LineWidth', 2, 'DisplayName', 'GO_TO_GOAL');
h_av    = plot(ax, NaN, NaN, '-', 'Color', c.avoid, 'LineWidth', 2, 'DisplayName', 'AVOID');
h_zone  = plot(ax, NaN, NaN, ':', 'Color', c.avoid, 'LineWidth', 1.2, 'DisplayName', 'leave zone');
ring    = [cos(linspace(0, 2*pi, 90)); sin(linspace(0, 2*pi, 90))];
h_hit   = plot(ax, NaN, NaN, 'v', 'MarkerSize', 7, 'MarkerFaceColor', c.avoid, ...
    'MarkerEdgeColor', c.surface, 'HandleVisibility', 'off');
h_scan  = plot(ax, NaN, NaN, '.', 'Color', c.muted, 'MarkerSize', 7, 'DisplayName', 'sweep scan');
h_pass  = plot(ax, NaN, NaN, 'd', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', c.avoid, ...
    'MarkerFaceColor', c.surface, 'DisplayName', 'pass point');
h_tgt   = plot(ax, NaN, NaN, 'o', 'MarkerSize', 18, 'Color', c.ink2, 'LineWidth', 1, ...
    'HandleVisibility', 'off');
h_rays  = gobjects(1, 4);
for j = 1:4
    h_rays(j) = plot(ax, NaN, NaN, '-', 'Color', c.muted, 'LineWidth', 1, 'HandleVisibility', 'off');
end
h_ends  = plot(ax, NaN, NaN, '.', 'Color', c.ink, 'MarkerSize', 14, 'HandleVisibility', 'off');
h_drone = hgtransform(ax);
draw_drone(h_drone, opts.icon_scale * p.radius, c);
h_title = title(ax, '', 'FontWeight', 'normal', 'Color', c.ink, 'Interpreter', 'none');
legend(ax, [h_go h_av h_scan h_pass h_zone], 'Location', 'southoutside', 'Orientation', 'horizontal', ...
    'Box', 'off', 'Interpreter', 'none', 'AutoUpdate', 'off');

btn = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Replay', 'FontSize', 10, ...
    'Units', 'normalized', 'Position', [0.02 0.015 0.10 0.045], 'Callback', @on_button);
pop = uicontrol(fig, 'Style', 'popupmenu', 'FontSize', 10, ...
    'String', arrayfun(@(s) sprintf('%gx speed', s), speeds, 'UniformOutput', false), ...
    'Value', speed_idx, 'Units', 'normalized', 'Position', [0.13 0.015 0.12 0.045]);

playing  = false;
stop_req = false;
vw = [];

if isempty(opts.video_file)
    play(false);
else
    set([btn pop], 'Visible', 'off');   % keep the controls out of the video
    vw = VideoWriter(opts.video_file, 'MPEG-4');
    vw.FrameRate = opts.fps;
    open(vw);
    play(true);
    close(vw);
    if isgraphics(fig)
        set([btn pop], 'Visible', 'on');
    end
end

    % ---------------------------------------------------------------------
    function on_button(~, ~)
        if playing
            stop_req = true;
        else
            play(false);
        end
    end

    function play(record)
        playing  = true;
        stop_req = false;
        btn.String = 'Stop';
        speed  = speeds(pop.Value);
        step   = max(1, round(speed / (opts.fps * p.dt)));
        frames = unique([1:step:K, K]);
        t0 = tic;
        for k = frames
            if ~isgraphics(fig) || stop_req
                break
            end
            draw_frame(k);
            if record
                drawnow;
                writeVideo(vw, getframe(fig));
            else
                drawnow limitrate;
                wait = sim_log.t(k) / speed - toc(t0);
                if wait > 0
                    pause(wait);
                end
            end
        end
        playing = false;
        if isgraphics(btn)
            btn.String = 'Replay';
        end
    end

    function draw_frame(k)
        [gx, gy] = mode_path(pos(1:k,:), sim_log.mode(1:k), 1);
        [ax_, ay_] = mode_path(pos(1:k,:), sim_log.mode(1:k), 2);
        set(h_go, 'XData', gx, 'YData', gy);
        set(h_av, 'XData', ax_, 'YData', ay_);

        % Ranger rays: front, left, back, right
        ends = nan(4, 2);
        for jj = 1:4
            th = sim_log.psi(k) + (jj - 1) * pi/2;
            r  = sim_log.ranges(k, jj);
            if isinf(r)
                len = p.ranger.range_max;
                style = ':';
            else
                len = r;
                style = '-';
                ends(jj,:) = pos(k,:) + r * [cos(th) sin(th)];
            end
            set(h_rays(jj), 'XData', pos(k,1) + [0 len*cos(th)], ...
                'YData', pos(k,2) + [0 len*sin(th)], 'LineStyle', style);
        end
        set(h_ends, 'XData', ends(:,1), 'YData', ends(:,2));

        tg = sim_log.targets(sim_log.wp(k), :);
        set(h_tgt, 'XData', tg(1), 'YData', tg(2));
        hp = sim_log.hit_point(k, :);
        if any(isnan(hp))
            set([h_zone h_hit], 'XData', NaN, 'YData', NaN);
        else
            rz = max(norm(tg - hp) - p.nav.leave_margin, 0);
            set(h_zone, 'XData', tg(1) + rz * ring(1,:), 'YData', tg(2) + rz * ring(2,:));
            set(h_hit, 'XData', hp(1), 'YData', hp(2));
        end

        % Latest sweep of this episode, filled in as it is scanned
        P = [NaN NaN];
        if ~any(isnan(hp)) && ~isempty(sim_log.sweeps)
            started = arrayfun(@(s) s.t(1), sim_log.sweeps) <= sim_log.t(k);
            if any(started)
                s = sim_log.sweeps(find(started, 1, 'last'));
                ok = isfinite(s.rng) & s.t <= sim_log.t(k);
                P = s.pos + s.rng(ok)' .* [cos(s.ang(ok))' sin(s.ang(ok))'];
            end
        end
        set(h_scan, 'XData', P(:,1), 'YData', P(:,2));
        set(h_pass, 'XData', sim_log.subgoal(k,1), 'YData', sim_log.subgoal(k,2));

        h_drone.Matrix = makehgtform('translate', [pos(k,:) 0], 'zrotate', sim_log.psi(k));
        status = modes{sim_log.mode(k)};
        if sim_log.phase(k) > 0
            status = [status ': ' sim_log.phase_names{sim_log.phase(k)}];
        end
        if k == K && sim_log.collided
            status = 'COLLIDED';
        end
        h_title.String = sprintf('t = %5.1f s     %-18s     target: %s     speed %.2f m/s', ...
            sim_log.t(k), status, sim_log.target_names{sim_log.wp(k)}, norm(sim_log.x(k, 3:4)));
    end
end

function draw_drone(parent, r, c)
% Quad-X icon in the body frame, nose along +x: arms, four rotors, nose.
a  = r / sqrt(2);
rr = 0.45 * r;
th = linspace(0, 2*pi, 25)';
line('Parent', parent, 'XData', [-a a NaN -a a], 'YData', [-a a NaN a -a], ...
    'Color', c.ink, 'LineWidth', 2);
for s = [1 1; 1 -1; -1 1; -1 -1]'
    patch('Parent', parent, 'XData', a*s(1) + rr*cos(th), 'YData', a*s(2) + rr*sin(th), ...
        'FaceColor', c.go, 'FaceAlpha', 0.25, 'EdgeColor', c.go, 'LineWidth', 1.2);
end
patch('Parent', parent, 'XData', r * [1.5 0.8 0.8], 'YData', r * [0 0.35 -0.35], ...
    'FaceColor', c.ink, 'EdgeColor', 'none');
end
