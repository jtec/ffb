% Builds a standard trajectory starting in mid-flight
tic;
% NOTE: Trajectory functions use an NED system.
traj = trajectory_build([0;0;-50]);
traj = trajectory_concatenateWaypoint([0; dl ;0],traj);
traj = trajectory_concatenateWaypoint([0; dl ;-dl],traj);

% Smooth:
% Define parameters that determine the minimal curvature of the trajectory:
envelope.v_max = 15;
envelope.CL_max = 1.47;
envelope.g = 9.81;
envelope.rho_min = 1.225;
envelope.S_min = uavsim.zagi.S;
envelope.m_max = uavsim.zagi.mass;

traj = trajectory_smooth(traj, envelope, 0.004);
% Rotate trajectory a little about the NED z axis to reveal bugs e.g. in the
% guidance laws that do not occur otherwise:
traj = trajectory_rotate([0 0 0]', angle2quat(deg2rad(30), 0, 0, 'ZYX'), traj);
% Prepare for simulation:
% traj = trajectory_discretize(traj, 1.0);
traj = trajectory_getAirborne(traj);
toc;
plotoptions.controlpointsvisible = false;
plotoptions.waypointsvisible = true;
plotoptions.tangentvisible = false;
plotoptions.curvaturevectorvisible = true;
plotoptions.discretepointsvisible = false;
plotoptions.plotcurvatureseparately = true;
plotoptions.plotosculatingcircle = false;
plotoptions.sCircle = 35;
plotoptions.plotkappaseparately = true;
plotoptions.firstderivativevisible = true;
plotoptions.secondderivativevisible = true;

trajectory_display(traj, plotoptions);