function snapshot = environment_at(env, time)
%ENVIRONMENT_AT  Active geometry for sensors/collisions/playback at one time.
snapshot = env;
if ~isfield(env,'scheduled'), return; end
snapshot = rmfield(snapshot,'scheduled');
for event = env.scheduled
    if time < event.time-1e-9, continue; end
    snapshot.obstacles(end+1) = event.obstacle;
    V = event.obstacle.vertices;
    snapshot.edges = [snapshot.edges; V,circshift(V,-1,1)]; %#ok<AGROW>
end
end
