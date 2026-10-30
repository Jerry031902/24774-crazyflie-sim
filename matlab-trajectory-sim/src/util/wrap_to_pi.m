function a = wrap_to_pi(a)
%WRAP_TO_PI  Wrap angles [rad] into [-pi, pi).
a = mod(a + pi, 2*pi) - pi;
end
