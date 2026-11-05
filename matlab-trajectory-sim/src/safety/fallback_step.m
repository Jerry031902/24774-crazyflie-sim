function [u_body, yaw_rate] = fallback_step(s, x, psi, ground_velocity, p)
%FALLBACK_STEP  Brake ground motion; then hold the stopped position.
% Drift is inferred from observed displacement, never read from p.wind/truth.
u_body = [0;0]; yaw_rate = 0;
if any(~isfinite([x(:); psi; ground_velocity(:)])), return; end
target = -(ground_velocity-x(3:4));
if strcmp(s.state,'HOLD')
    target = target + sat_norm(p.safety.hold_gain*(s.hold_pos-x(1:2)),p.safety.hold_speed);
end
u_body = rot2d(psi)' * sat_norm(target,p.v_max);
end
