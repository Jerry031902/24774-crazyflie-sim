function r = read_rangers(pos, psi, env, p)
%READ_RANGERS  Multi-ranger deck: distances seen front, left, back and right.
%   r = READ_RANGERS(pos, psi, env, p) returns [front; left; back; right]
%   in metres for a drone at pos = [x; y] with yaw psi. Each sensor casts
%   p.ranger.n_rays rays spread over its p.ranger.fov cone against every
%   wall segment in env.edges and reports the nearest hit, as a
%   time-of-flight sensor reports the nearest target in its field of view.
%   Inf means no return within p.ranger.range_max.
%
%   Each reading that hits something gets zero-mean Gaussian noise with
%   standard deviation p.ranger.noise + p.ranger.noise_rel * range, drawn
%   independently per sensor and per update.

A = env.edges(:, 1:2);
E = env.edges(:, 3:4) - A;
W = A - pos(:)';                        % from the drone to each edge start

if p.ranger.n_rays > 1 && p.ranger.fov > 0
    spread = linspace(-p.ranger.fov/2, p.ranger.fov/2, p.ranger.n_rays);
else
    spread = 0;
end

r = inf(4, 1);
for j = 1:4
    th = psi + (j - 1) * pi/2 + spread;
    U  = [cos(th); sin(th)];            % 2 x rays
    % Ray pos + t*u meets edge a + s*e where t = (w x e)/(u x e), s = (w x u)/(u x e)
    den = E(:,2) * U(1,:) - E(:,1) * U(2,:);
    t = (W(:,1) .* E(:,2) - W(:,2) .* E(:,1)) ./ den;
    s = (W(:,1) * U(2,:) - W(:,2) * U(1,:)) ./ den;
    t(abs(den) < 1e-12 | t < 0 | s < 0 | s > 1) = inf;
    r(j) = min(t(:));
end

if p.ranger.noise > 0 || p.ranger.noise_rel > 0
    hit = isfinite(r);
    sigma = p.ranger.noise + p.ranger.noise_rel * r(hit);
    r(hit) = r(hit) + sigma .* randn(nnz(hit), 1);
end
r(r > p.ranger.range_max) = inf;
r = max(r, 0);
end
