function [x, psi] = dynamics_step(x, psi, u_body, yaw_rate, wind, p)
%DYNAMICS_STEP  Advance the 2-D kinematic drone by one time step.
%   [x, psi] = DYNAMICS_STEP(x, psi, u_body, yaw_rate, wind, p)
%     x         state [px; py; vx; vy] in the world frame [m, m/s]
%     psi       yaw [rad], 0 = nose along +x, counter-clockwise positive
%     u_body    commanded velocity [forward; left] in the body frame [m/s]
%     yaw_rate  commanded yaw rate [rad/s]
%     wind      drift velocity added to the motion [2x1, m/s]
%   This is the Crazyflie hover setpoint (vx, vy, yaw rate) without height.
%
%   Model (no attitude or rotor dynamics):
%     v_cmd = R(psi) * u_body        limited to |v_cmd| <= v_max
%     dv/dt = (v_cmd - v) / tau      limited to |dv/dt| <= a_max
%     dp/dt = v + wind
%     dpsi/dt = yaw_rate             limited to |yaw_rate| <= yaw_rate_max
%   integrated with semi-implicit Euler (the new velocity moves the position).

v_cmd = sat_norm(rot2d(psi) * u_body, p.v_max);

vel = x(3:4);
acc = sat_norm((v_cmd - vel) / max(p.tau, p.dt), p.a_max);  % tau < dt: reach the command in one step
vel = sat_norm(vel + acc * p.dt, p.v_max);
pos = x(1:2) + (vel + wind) * p.dt;
x   = [pos; vel];

psi = wrap_to_pi(psi + min(max(yaw_rate, -p.yaw_rate_max), p.yaw_rate_max) * p.dt);
end
