function run_lqr_tuned_demo(make_gif, show_window)
% RUN_LQR_DRAG_DEMO  Live LQR position tracking and synchronized plots.
%   run_lqr_tuned_demo         opens the animation and exports a GIF.
%   run_lqr_tuned_demo(false)  opens the animation without writing the GIF.
%   run_lqr_tuned_demo(true,false) runs invisibly, useful for batch validation.
%
% This is a self-contained 2-D MATLAB model, not Crazyflie firmware, an EKF,
% Isaac Sim, or a flight result. Altitude is assumed fixed; yaw is zero.
% State [x;y;vx;vy], input [roll_ref;pitch_ref], angles in radians.
% A first-order attitude response and linear velocity drag are ASSUMED.

if nargin < 1, make_gif=true; end
if nargin < 2, show_window=true; end
root=fileparts(mfilename('fullpath'));out=fullfile(root,'tuned_results');
if ~exist(out,'dir'),mkdir(out);end
p.g=9.81; p.beta_design=.5; p.beta_plant=.5; % assumed, 1/s
p.attitude_tau=.08; p.dt=.01; p.substeps=5;
p.angle_limit=10*pi/180; p.duration=6; p.target=[.5;0];

% Design once; exact zero-order-hold discretization, toolbox-free DARE.
A=[0 0 1 0;0 0 0 1;0 0 -p.beta_design 0;0 0 0 -p.beta_design];
B=[0 0;0 0;0 -p.g;-p.g 0];
E=expm([A B;zeros(2,6)]*p.dt);Ad=E(1:4,1:4);Bd=E(1:4,5:6);
Q=diag([800 800 80 80]);R=diag([300 300]);P=Q;
for iter=1:200000
 Pnew=Q+Ad'*P*Ad-Ad'*P*Bd*((R+Bd'*P*Bd)\(Bd'*P*Ad));
 if norm(Pnew-P,'fro')<1e-10,P=Pnew;break;end
 P=Pnew;
end
assert(iter<200000,'Riccati equation did not converge.');
K=(R+Bd'*P*Bd)\(Bd'*P*Ad);
assert(max(abs(eig(Ad-Bd*K)))<1,'Unstable discrete LQR design.');

n=round(p.duration/p.dt)+1;t=(0:n-1)'*p.dt;
S=zeros(n,4);U=zeros(n,2);ATT=zeros(n,2);x=zeros(4,1);att=zeros(2,1);
ref=[p.target;0;0];
if show_window,vis='on';else,vis='off';end
f=figure('Name','Tuned LQR + linear drag: live simulation','Visible',vis,...
 'Color','w','Position',[50 70 1200 650]);
left=axes(f,'Position',[.065 .17 .39 .68]);hold(left,'on');grid(left,'on');box(left,'on');
xlim(left,[-.12 .72]);ylim(left,[-.30 .30]);axis(left,'equal');
xlabel(left,'x (m)');ylabel(left,'y (m)');title(left,'Horizontal motion | no obstacles');
plot(left,0,0,'ks','MarkerFaceColor','k','MarkerSize',7);text(left,.01,.025,'Start');
plot(left,p.target(1),p.target(2),'p','MarkerSize',16,'LineWidth',1.5,'Color',[.77 .07 .19]);
text(left,p.target(1)+.015,.025,'Target');
route=animatedline(left,'Color',[.02 .48 .72],'LineWidth',2.1);
arm=plot(left,NaN,NaN,'-','Color',[.1 .2 .3],'LineWidth',3);
rotors=plot(left,NaN,NaN,'o','Color',[.02 .48 .72],'MarkerSize',9,'LineWidth',1.4);
center=plot(left,NaN,NaN,'o','MarkerFaceColor',[.1 .2 .3],'MarkerEdgeColor',[.1 .2 .3]);
status=text(left,-.1,.25,'','FontSize',10,'Color',[.2 .3 .38]);

ax1=axes(f,'Position',[.55 .58 .41 .27]);hold(ax1,'on');grid(ax1,'on');box(ax1,'on');
plot(ax1,[0 p.duration],[p.target(1) p.target(1)],'--','Color',[.48 .54 .58],'LineWidth',1.3);
position=animatedline(ax1,'Color',[.77 .07 .19],'LineWidth',2.2);
xlim(ax1,[0 p.duration]);ylim(ax1,[-.04 .65]);ylabel(ax1,'Position x (m)');title(ax1,'LQR position tracking');
legend(ax1,{'Target','Position'},'Location','southeast');
ax2=axes(f,'Position',[.55 .17 .41 .27]);hold(ax2,'on');grid(ax2,'on');box(ax2,'on');
pitch=animatedline(ax2,'Color',[.77 .07 .19],'LineWidth',2.2);
xlim(ax2,[0 p.duration]);ylim(ax2,[-11 11]);ylabel(ax2,'Pitch command (deg)');xlabel(ax2,'Time (s)');
title(ax2,'Outer-loop command | +/-10 deg limit');
annotation(f,'textbox',[.06 .02 .9 .06],'String',...
 'MATLAB 2-D assumed model | linear drag beta = 0.5 /s | attitude lag = 0.08 s | no EKF / hardware',...
 'EdgeColor','none','Color',[.32 .37 .4],'FontSize',10);
gif=fullfile(out,'LQR_Tuned_Drag_Live.gif');

for k=1:n
 % Position mode: all four state-feedback terms are active.
 cmd=-K*(x-ref);cmd=min(max(cmd,-p.angle_limit),p.angle_limit);
 S(k,:)=x';U(k,:)=cmd';ATT(k,:)=att';
 addpoints(route,x(1),x(2));addpoints(position,t(k),x(1));
 addpoints(pitch,t(k),cmd(2)*180/pi);
 % Simple top-view icon; its size is decorative, not to physical scale.
 d=.035;coords=[-d -d;d d;NaN NaN;-d d;d -d]+x(1:2)';
 set(arm,'XData',coords(:,1),'YData',coords(:,2));
 set(rotors,'XData',coords([1 2 4 5],1),'YData',coords([1 2 4 5],2));
 set(center,'XData',x(1),'YData',x(2));
 set(status,'String',sprintf('t = %.2f s   speed = %.2f m/s',t(k),norm(x(3:4))));
 % Render 25 fps during integration, not after the simulation.
 if mod(k-1,4)==0 || k==n
  drawnow;
  if make_gif
   frame=getframe(f);[im,map]=rgb2ind(frame.cdata,256);
   if k==1,imwrite(im,map,gif,'gif','LoopCount',Inf,'DelayTime',.04);
   else,imwrite(im,map,gif,'gif','WriteMode','append','DelayTime',.04);end
  end
 end
 if k<n
  h=p.dt/p.substeps;
  for j=1:p.substeps
   att=att+(cmd-att)*(1-exp(-h/p.attitude_tau));
   accel=-p.g*[tan(att(2));tan(att(1))]-p.beta_plant*x(3:4);
   x(1:2)=x(1:2)+x(3:4)*h+.5*accel*h^2;
   x(3:4)=x(3:4)+accel*h;
  end
 end
end
T=array2table([t S U*180/pi ATT*180/pi], ...
 'VariableNames',{'time_s','x_m','y_m','vx_m_s','vy_m_s','roll_ref_deg','pitch_ref_deg','roll_actual_deg','pitch_actual_deg'});
writetable(T,fullfile(out,'lqr_drag_log.csv'));
valid=abs(S(:,1)-p.target(1))<=.03 & vecnorm(S(:,3:4),2,2)<=.05;
lastbad=find(~valid,1,'last');if isempty(lastbad),settle=0;elseif lastbad<n,settle=t(lastbad+1);else,settle=NaN;end
metrics=struct('target_m',p.target(1),'overshoot_percent',max(0,(max(S(:,1))-p.target(1))/p.target(1)*100),...
 'settling_s',settle,'rmse_m',sqrt(mean((S(:,1)-p.target(1)).^2)),...
 'beta_design_per_s',p.beta_design,'beta_plant_per_s',p.beta_plant,...
 'attitude_tau_s',p.attitude_tau,'angle_limit_deg',p.angle_limit*180/pi,'duration_s',p.duration);
fid=fopen(fullfile(out,'metrics.json'),'w');fprintf(fid,'%s',jsonencode(metrics));fclose(fid);
exportgraphics(f,fullfile(out,'final_frame.png'),'Resolution',150);
fprintf('LQR + drag finished. Overshoot %.2f%%, settling %.2f s, RMSE %.3f m over %.0f s.\n',...
 metrics.overshoot_percent,metrics.settling_s,metrics.rmse_m,p.duration);
fprintf('Results: %s\n',out);
end
