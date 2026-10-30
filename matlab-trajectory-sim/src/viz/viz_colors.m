function c = viz_colors()
%VIZ_COLORS  Shared plot colours as hex strings.
%   Blue and orange mark the two navigator modes; neutral inks carry the
%   map, rays and labels; status red/orange are kept for collisions,
%   unreachable targets and the near-miss threshold.
c.surface  = '#fcfcfb';   % axes and figure background
c.ink      = '#0b0b0b';   % primary marks and labels
c.ink2     = '#52514e';   % secondary text, threshold lines
c.muted    = '#898781';   % sensor rays, obstacle outlines
c.grid     = '#e1e0d9';
c.obstacle = '#c3c2b7';
c.go       = '#2a78d6';   % GO_TO_GOAL
c.avoid    = '#eb6834';   % AVOID
c.front    = '#1baf7a';   % front-of-travel ranger
c.left     = '#4a3aa7';   % left-of-travel ranger (wall detector)
c.critical = '#d03b3b';   % collision, unreachable
c.serious  = '#ec835a';   % near-miss threshold
end
