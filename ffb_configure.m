% Set up wind:
uavsim.environment.ambientWind_NED_mps = 0*[-3 -3 0]';

% Define initial position and speed of each UAS:
b = uavsim.cularis.b;
% Virtual leader:
uavsim.formation(1).p0_NED = benchmarkTrajectory.ts_p.Data(1, :)';
uavsim.formation(1).v0_NED = benchmarkTrajectory.ts_v.Data(1, :)';
% Rotate separation vectors to e frame, assuming initial planar motion:
ux = fflib_normalize(uavsim.formation(1).v0_NED);
uz = [0 0 1]';
uy = cross(uz, ux);
DCM_eg = [ux uy uz];
for k=2:uavsim.nUAS_max
    uavsim.formation(k).dp_GF = [-b b 0]';
    % The first uav concides with the leader: 
    uavsim.formation(2).dp_GF = [0 0 0]';
    uavsim.formation(k).p0_NED = uavsim.formation(k-1).p0_NED + DCM_eg * uavsim.formation(k).dp_GF;
    uavsim.formation(k).v0_NED = uavsim.formation(k-1).v0_NED + [0.1 0.1 0.1]'; 
end

% Simulation time:
uavsim.simConfig.tsim = findclosemultiple(benchmarkTrajectory.ts_p.Time(end), uavsim.simConfig.tsample_model);
% Controller output sampling times:
uavsim.ap.tsampling_guidancelaws = findclosemultiple(1/30, uavsim.simConfig.tsample_model);
uavsim.ap.tsampling_attitudelaws = findclosemultiple(1/100, uavsim.simConfig.tsample_model);
