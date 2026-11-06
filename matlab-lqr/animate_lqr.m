% HONGFEI LIU - HORIZONTAL LQR ANIMATION
% Run this script from MATLAB. It runs the existing week1_model.m first,
% then saves Hongfei_LQR_demo.mp4 beside this file.
% This is a synthetic simulation with estimated parameters, not flight data.

animation_folder = fileparts(mfilename('fullpath'));
run(fullfile(animation_folder, 'week1_model.m'));
if exist('model_figure','var') && isgraphics(model_figure)
    close(model_figure);
end

animation_folder = fileparts(mfilename('fullpath'));
video_path = fullfile(animation_folder, 'Hongfei_LQR_demo.mp4');
t_anim = results.time;
s_anim = results.state;
r_anim = results.reference;
u_anim = results.command * (180/pi);
angles_anim = results.actual_angles * (180/pi);
cfg_anim = results.config;
err_anim = sqrt(sum((s_anim(1:2,:) - r_anim(1:2,:)).^2, 1));
speed_anim = sqrt(sum(s_anim(3:4,:).^2, 1));

fig_anim = figure('Name','Hongfei horizontal LQR animation', ...
    'NumberTitle','off','Color','w','Position',[100 70 1200 680]);
ax_map = axes('Parent',fig_anim,'Position',[0.07 0.16 0.48 0.72]);
hold(ax_map,'on'); grid(ax_map,'on'); axis(ax_map,'equal');
xlim(ax_map,[-0.22 1.30]); ylim(ax_map,[-0.23 1.25]);
xlabel(ax_map,'x position (m)'); ylabel(ax_map,'y position (m)');
title(ax_map,'Top view: active waypoint and simulated flight');
plot(ax_map,cfg_anim.waypoint_xy(:,1),cfg_anim.waypoint_xy(:,2), ...
    '--','Color',[0.68 0.68 0.68],'LineWidth',1.2,'HandleVisibility','off');
plot(ax_map,cfg_anim.waypoint_xy(:,1),cfg_anim.waypoint_xy(:,2), ...
    'ko','MarkerFaceColor','w','MarkerSize',6,'HandleVisibility','off');
h_path = plot(ax_map,s_anim(1,1),s_anim(2,1),'-', ...
    'Color',[0.02 0.39 0.84],'LineWidth',2,'DisplayName','Simulated path');
h_goal = plot(ax_map,r_anim(1,1),r_anim(2,1),'p', ...
    'Color',[0.86 0.18 0.11],'MarkerFaceColor',[0.98 0.35 0.18], ...
    'MarkerSize',16,'DisplayName','Current target');
h_drone = plot(ax_map,s_anim(1,1),s_anim(2,1),'o', ...
    'Color',[0.0 0.27 0.65],'MarkerFaceColor',[0.08 0.55 1], ...
    'MarkerSize',16,'LineWidth',1.5,'DisplayName','Drone (simulated)');
h_arm_x = plot(ax_map,nan,nan,'-','Color',[0.0 0.27 0.65], ...
    'LineWidth',2.1,'HandleVisibility','off');
h_arm_y = plot(ax_map,nan,nan,'-','Color',[0.0 0.27 0.65], ...
    'LineWidth',2.1,'HandleVisibility','off');
h_tilt = quiver(ax_map,0,0,0,0,0,'Color',[0.94 0.42 0.04], ...
    'LineWidth',2,'MaxHeadSize',1.0,'DisplayName','Commanded tilt direction');
h_velocity = quiver(ax_map,0,0,0,0,0,'Color',[0.06 0.64 0.66], ...
    'LineWidth',2,'MaxHeadSize',1.0,'DisplayName','Velocity direction');
legend(ax_map,'Location','southoutside','NumColumns',2,'Box','off');

ax_error = axes('Parent',fig_anim,'Position',[0.64 0.58 0.32 0.27]);
hold(ax_error,'on'); grid(ax_error,'on');
h_error = plot(ax_error,nan,nan,'Color',[0.10 0.48 0.82],'LineWidth',1.8);
plot(ax_error,[t_anim(1) t_anim(end)],[0.02 0.02],':', ...
    'Color',[0.2 0.55 0.2],'LineWidth',1.0);
h_err_cursor = plot(ax_error,[0 0],[0 1.6],':k');
xlim(ax_error,[0 t_anim(end)]); ylim(ax_error,[0 1.6]);
ylabel(ax_error,'2D error (m)');
title(ax_error,'Error drops after each waypoint change');

ax_cmd = axes('Parent',fig_anim,'Position',[0.64 0.16 0.32 0.28]);
hold(ax_cmd,'on'); grid(ax_cmd,'on');
h_roll = plot(ax_cmd,nan,nan,'Color',[0.04 0.39 0.85], ...
    'LineWidth',1.5,'DisplayName','Roll ref');
h_pitch = plot(ax_cmd,nan,nan,'Color',[0.87 0.18 0.13], ...
    'LineWidth',1.5,'DisplayName','Pitch ref');
h_roll_actual = plot(ax_cmd,nan,nan,'--','Color',[0.35 0.63 0.97], ...
    'LineWidth',1.0,'DisplayName','Roll actual (sim)');
h_pitch_actual = plot(ax_cmd,nan,nan,'--','Color',[0.98 0.54 0.43], ...
    'LineWidth',1.0,'DisplayName','Pitch actual (sim)');
plot(ax_cmd,[t_anim(1) t_anim(end)],[10 10],':k','HandleVisibility','off');
plot(ax_cmd,[t_anim(1) t_anim(end)],[-10 -10],':k','HandleVisibility','off');
h_cmd_cursor = plot(ax_cmd,[0 0],[-11 11],':k','HandleVisibility','off');
xlim(ax_cmd,[0 t_anim(end)]); ylim(ax_cmd,[-11 11]);
xlabel(ax_cmd,'Simulation time (s)'); ylabel(ax_cmd,'Command angle (deg)');
title(ax_cmd,'LQR commands and assumed attitude lag');
legend(ax_cmd,'Location','southoutside','NumColumns',2,'Box','off');

status_box = annotation(fig_anim,'textbox',[0.06 0.91 0.90 0.055], ...
    'String','ASSUMED PARAMETERS / SYNTHETIC FEEDBACK - NOT FLIGHT DATA', ...
    'Color',[0.45 0.18 0.10],'FontWeight','bold','FontSize',14, ...
    'HorizontalAlignment','center','EdgeColor','none');
readout_box = annotation(fig_anim,'textbox',[0.56 0.87 0.40 0.055], ...
    'String','','FontSize',10,'HorizontalAlignment','right','EdgeColor','none');
summary_box = annotation(fig_anim,'textbox',[0.06 0.02 0.89 0.075], ...
    'String','Blue dot: simulated position. Red star: current target. Orange: tilt command direction. Cyan: velocity.', ...
    'FontSize',10,'HorizontalAlignment','center','EdgeColor','none');

writer = VideoWriter(video_path,'MPEG-4');
writer.FrameRate = 10;
writer.Quality = 85;
open(writer);
frame_step = max(1,round(0.2/cfg_anim.Ts)); % 2x playback: 0.2 sim s / 0.1 video s.
frame_indices = unique([1:frame_step:numel(t_anim), numel(t_anim)]);
tmp_png = fullfile(tempdir,'hongfei_lqr_animation_frame.png');
try
    for frame_id = 1:numel(frame_indices)
        k = frame_indices(frame_id);
        x_now = s_anim(1,k); y_now = s_anim(2,k);
        set(h_path,'XData',s_anim(1,1:k),'YData',s_anim(2,1:k));
        set(h_goal,'XData',r_anim(1,k),'YData',r_anim(2,k));
        set(h_drone,'XData',x_now,'YData',y_now);
        set(h_arm_x,'XData',[x_now-0.035 x_now+0.035],'YData',[y_now y_now]);
        set(h_arm_y,'XData',[x_now x_now],'YData',[y_now-0.035 y_now+0.035]);
        set(h_tilt,'XData',x_now,'YData',y_now, ...
            'UData',-u_anim(2,k)*0.023,'VData',-u_anim(1,k)*0.023);
        set(h_velocity,'XData',x_now,'YData',y_now, ...
            'UData',s_anim(3,k)*0.30,'VData',s_anim(4,k)*0.30);
        set(h_error,'XData',t_anim(1:k),'YData',err_anim(1:k));
        set(h_err_cursor,'XData',[t_anim(k) t_anim(k)]);
        set(h_roll,'XData',t_anim(1:k),'YData',u_anim(1,1:k));
        set(h_pitch,'XData',t_anim(1:k),'YData',u_anim(2,1:k));
        set(h_roll_actual,'XData',t_anim(1:k),'YData',angles_anim(1,1:k));
        set(h_pitch_actual,'XData',t_anim(1:k),'YData',angles_anim(2,1:k));
        set(h_cmd_cursor,'XData',[t_anim(k) t_anim(k)]);
        readout_box.String = sprintf('t = %.1f s | error = %.3f m | speed = %.2f m/s', ...
            t_anim(k),err_anim(k),speed_anim(k));
        drawnow;
        try
            frame = getframe(fig_anim);
            writeVideo(writer,frame);
        catch frame_error
            % Some batch sessions do not expose a screen for getframe.
            exportgraphics(fig_anim,tmp_png,'Resolution',100);
            writeVideo(writer,imread(tmp_png));
        end
    end
    close(writer);
catch animation_error
    close(writer);
    if exist(tmp_png,'file'), delete(tmp_png); end
    rethrow(animation_error);
end
if exist(tmp_png,'file'), delete(tmp_png); end
fprintf('Saved %d animation frames: %s\n',numel(frame_indices),video_path);