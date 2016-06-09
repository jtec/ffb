function traj = ffb_generatebenchmarkTrajectory(tcruise, uavsim)
% Builds a standard trajectory starting in mid-flight
tic;

% NOTE: Trajectory functions use the NED system.
traj = trajectory_build([0;0;-50]);
traj.news = 'verbose';
% Fly straight north
traj = trajectory_concatenateWaypoint([15*tcruise;0;0],traj);
% Climb:
% gammaClimb = deg2rad(5);
% hClimb = 10;
% xClimb = hClimb/tan(gammaClimb);
% traj = trajectory_concatenateWaypoint([xClimb;0;-hClimb],traj);
% Add straight segment:
%traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);
% Descend:
%traj = trajectory_concatenateWaypoint([xClimb;0;hClimb],traj);
% Add straight segment:
%traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);
% Add horizontal turn:
%traj = trajectory_concatenateWaypoint([0;lStraightsegments;0],traj);
%traj = trajectory_concatenateWaypoint([lStraightsegments;0;0],traj);
% Fly loiter helix:
lStraightsegments = 15*5;
gammaClimb = deg2rad(8);
rHelix =  2*lStraightsegments;
hClimb = 2*rHelix*pi * tan(gammaClimb);
traj = trajectory_concatenateHelicalPattern(rHelix, traj, -1, 1, hClimb);
traj = trajectory_concatenateHelicalPattern(rHelix, traj, 1, 1, -1.01*hClimb);

% Smooth:
% Define parameters that determine the minimal curvature of the trajectory:
envelope.v_max = 15;
envelope.CL_max = 1.47;
envelope.g = 9.81;
envelope.rho_min = 1.225;
envelope.S_min = uavsim.cularis.S;
envelope.m_max = uavsim.cularis.mass;
traj = trajectory_smooth(traj, envelope, 0.01);

% Rotate trajectory a little about the NED z axis to avoid hiding bugs e.g.
% in the guidance laws that do not occur when flying straight north:
traj = trajectory_rotate([0 0 0]', angle2quat(deg2rad(30), 0, 0, 'ZYX'), traj);
% Prepare for simulation:
traj = trajectory_getAirborne(traj);
traj = trajectory_generateTimeseries(traj, 0.01, 15);
toc;
% Display:
plotoptions.controlpointsvisible = false;
plotoptions.waypointsvisible = false;
plotoptions.tangentvisible = false;
plotoptions.curvaturevectorvisible = false;
plotoptions.discretepointsvisible = false;
plotoptions.plotcurvatureseparately = true;
plotoptions.plotosculatingcircle = false;
plotoptions.sCircle = 35;
plotoptions.plotkappaseparately = true;
% trajectory_display(traj, plotoptions);
end