function [Vi_bfrozen, omegai_bfrozen, Vi_NED, p0_filament_left, p0_filament_right, dcm_ew] = ffb_vortexdiagnostic(pNED, ...
    vNED, ...
    CL, ...
    qAttitude, ...
    alpha, ...
    beta, ...
    p, ...
    tsim, ...
    VortexSig)

% Tag indicating that this function is Embedded Matlab:
%#codegen

ID = p.id;
stack.vi_center_NED = [0 0 0]';
stack.vi_eff_b = [0 0 0]';
stack.omegai_eff_b = [0 0 0]';
stack.checkpoints.center = [0 0 0]';
stack.checkpoints.nose = zeros(3,1);
stack.checkpoints.rear = zeros(3,1);
stack.checkpoints.leftwingtip = zeros(3,1);
stack.checkpoints.rightwingtip = zeros(3,1);
stack.checkpoints.fintip = zeros(3,1);
stack.checkpoints.finbottom = zeros(3,1);
% Compute unit vectors pointing along the wing:
stack.leftwingvector = zeros(3,1);
stack.rightwingvector = zeros(3,1);

%test = linspace(2,3,randi(100));

% Inialize shared values structure:
stack.DCM_bw_leader = eye(3);
stack.DCM_eb_leader = eye(3);
stack.DCM_ew_leader = eye(3);
stack.DCM_bw_this = eye(3);
stack.DCM_eb_this = eye(3);
stack.DCM_ew_this = eye(3);

stack.DCM_be_follower = eye(3);
stack.DCM_windLeader2bodyFollower = eye(3);
stack.p0_rightFilament_wleader = zeros(3,1);
stack.p0_leftFilament_wleader = zeros(3,1);
stack.p0_rightFilament_bleader = zeros(3,1);
stack.p0_leftFilament_bleader = zeros(3,1);

% Compute necessary rotation matrices:
this = VortexSig(ID);
stack.DCM_bw_this = ffb_dcmbody2wind(this.alpha_rad, this.beta_rad)';
stack.DCM_eb_this = ffb_quat2dcm(this.qAttitude')';
stack.DCM_ew_this = stack.DCM_eb_this * stack.DCM_bw_this;

% Select VortexSig that have a relevant impact on this one:
[relevantUAS, n_relevantUAS] = selectDominantUASs(ID, VortexSig, stack);
if n_relevantUAS > 0
    relevantUAS = relevantUAS(1:n_relevantUAS);
    for j=1:length(relevantUAS)
        % UAS inducing flow:
        maximoleader = VortexSig(relevantUAS(j));
        if ~isempty(maximoleader)
            % Computation fails for two vehicles that are exactly in the same
            % place, as might be case at the very beginning of a simulation:
            % if norm(maximoleader.p_NED_m - VortexSig(ID).p_NED_m) > 1e-3
            % FIXME: Not reliable, just wait a little as workaround:
            if tsim > 0.1;
                % Compute rotation matrix from the leader's wind frame to the
                % follower's body frame:
                % DCM (wind leader) -> (body leader)
                stack.DCM_bw_leader = ffb_dcmbody2wind(maximoleader.alpha_rad, maximoleader.beta_rad)';
                % DCM (body leader) -> NED
                stack.DCM_eb_leader = ffb_quat2dcm(maximoleader.qAttitude')';
                % DCM (wind leader) -> NED
                stack.DCM_ew_leader = stack.DCM_eb_leader * stack.DCM_bw_leader;
                % DCM NED -> (body follower)
                stack.DCM_be_follower = ffb_quat2dcm(VortexSig(ID).qAttitude');
                stack.DCM_windLeader2bodyFollower = stack.DCM_be_follower * stack.DCM_ew_leader;
                % Compute source points of vortex filaments:
                % unit vector that points from the wind frame's origin to the right
                % wing tip:
                % TODO Handling of b not entirely correct.
                stack.p0_rightFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b_m * pi/4, 1);
                stack.p0_rightFilament_wleader = stack.DCM_bw_leader' * stack.p0_rightFilament_bleader;
                % The same procedure for the left wing:
                stack.p0_leftFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b_m * pi/4, -1);
                stack.p0_leftFilament_wleader = stack.DCM_bw_leader' * stack.p0_leftFilament_bleader;
                
                % Compute bounding points:
                if ~isfield(VortexSig(ID), 'checkpoints')
                    stack.checkpoints.center = [0 0 0]';
                    stack.checkpoints.nose = VortexSig(ID).pNose_b_m;
                    stack.checkpoints.rear = VortexSig(ID).pNose_b_m + [-VortexSig(ID).longiRefLength_m 0 0]';
                    stack.checkpoints.leftwingtip = getWingPoint(VortexSig(ID),  0.5 * maximoleader.b_m, -1);
                    stack.checkpoints.rightwingtip = getWingPoint(VortexSig(ID),  0.5 * maximoleader.b_m, 1);
                    stack.checkpoints.fintip = VortexSig(ID).posFinTip_b_m;
                    stack.checkpoints.finbottom = VortexSig(ID).posFinTip_b_m + [0 0 -VortexSig(ID).verticalRefLength_m]';
                    % Compute unit vectors pointing along the wing:
                    stack.leftwingvector = fflib_normalize(stack.checkpoints.leftwingtip);
                    stack.rightwingvector = fflib_normalize(stack.checkpoints.rightwingtip);
                end
                [test, stack.vi_center_NED]= getVi(stack.checkpoints.center, ...
                    VortexSig(ID), maximoleader, ...
                    stack);
                
                debugos = false;
                if debugos
                    computeInducedV(ID, VortexSig, stack, maximoleader);
                end
                [stack] = computeEffectiveVAndOmega(VortexSig, ID, maximoleader, stack);
            end
        end
    end
end

Vi_bfrozen = stack.vi_eff_b;
omegai_bfrozen = stack.omegai_eff_b;
Vi_NED = stack.vi_center_NED;

% Compute origin and center axis of vortices for this UAS:
% Compute vectors in NED:
p0_rightFilament_b = getWingPoint(VortexSig(ID),  0.5 * VortexSig(ID).b_m * pi/4, 1);
p0_rightFilament_NED = body2NED(p0_rightFilament_b, VortexSig(ID).p_NED_m, VortexSig(ID).qAttitude);
p0_leftFilament_b = getWingPoint(VortexSig(ID),  0.5 * VortexSig(ID).b_m * pi/4, -1);
p0_leftFilament_NED = body2NED(p0_leftFilament_b, VortexSig(ID).p_NED_m, VortexSig(ID).qAttitude);

p0_filament_right = p0_rightFilament_NED;
p0_filament_left = p0_leftFilament_NED;
dcm_ew = stack.DCM_ew_this;
if tsim > 1
    bp = 0;
end
end

% Computes the average induced velocity and índuced body rotation rates:
function stack_out = computeEffectiveVAndOmega(VortexSig, ID, maximoleader, stack_in)
vi_center2nose = average(stack_in.checkpoints.center, stack_in.checkpoints.nose, VortexSig, ID, maximoleader, stack_in);
vi_center2rear = average(stack_in.checkpoints.center, stack_in.checkpoints.rear, VortexSig, ID, maximoleader, stack_in);
vi_center2rightwingtip = average(stack_in.checkpoints.center, stack_in.checkpoints.rightwingtip, VortexSig, ID, maximoleader, stack_in);
vi_center2leftwingtip = average(stack_in.checkpoints.center, stack_in.checkpoints.leftwingtip, VortexSig, ID, maximoleader, stack_in);
vi_center2fintip = average(stack_in.checkpoints.center, stack_in.checkpoints.fintip, VortexSig, ID, maximoleader, stack_in);
vi_center2finbottom = average(stack_in.checkpoints.center, stack_in.checkpoints.finbottom, VortexSig, ID, maximoleader, stack_in);

stack_out = stack_in;
stack_out.vi_eff_b(1) = 1/4 * (vi_center2rightwingtip(1) ...
    + vi_center2leftwingtip(1) ...
    + vi_center2fintip(1) ...
    + vi_center2finbottom(1));

stack_out.vi_eff_b(2) = 1/4 * (vi_center2nose(2) ...
    + vi_center2rear(2) ...
    + vi_center2fintip(2) ...
    + vi_center2finbottom(2));

stack_out.vi_eff_b(3) = 1/4 * (vi_center2rear(3) ...
    + vi_center2nose(3) ...
    + vi_center2rightwingtip(3) ...
    + vi_center2leftwingtip(3));
end

% Computes the average induced velocity over one airframe segment
% by integrating between two checkpoints in the body frame.
function vi_av = average(p1, p2, VortexSig, ID, maximoleader, stack)
% Generate evenly spaced integration points:
ds = 0.1;
l = norm(p1-p2);
steps = zeros(10,1);
div = length(steps);
for k=2:div
    steps(k) = steps(k-1) + (l/div);
end
unit_p1top2 = fflib_normalize(p2-p1);
vi_av = zeros(3,1);
for i=1:length(steps);
    p = p1 + steps(i) * unit_p1top2;
    [vi_atPoint_b, ~] = getVi(p, VortexSig(ID), maximoleader, stack);
    vi_av = vi_av + vi_atPoint_b;
end
vi_av = vi_av ./ length(steps);
end

% Computes the induced velocity distribution at discrete points:
function computeInducedV(ID, VortexSig, stack, maximoleader)
b = VortexSig(ID).b_m;

vi_alongBodyX = distribution(VortexSig(ID), [0 0 0]', [1 0 0]', [-b, b], stack, maximoleader);
vi_alongBodyZ = distribution(VortexSig(ID),[0 0 0]', [0 0 1]', [-b, b]', stack, maximoleader);
vi_alongtheleftwing = distribution(VortexSig(ID),[0 0 0]', stack.leftwingvector, [0 b], stack, maximoleader);
vi_alongtherightwing = distribution(VortexSig(ID),[0 0 0]', - stack.rightwingvector, [0 -b], stack, maximoleader);
vi_alongthewing = [vi_alongtheleftwing vi_alongtherightwing];
%
% % Piotr:
subplot(3,2,1)
plot(vi_alongthewing(1,:), vi_alongthewing(2,:) );
title('Wx along the wing')
axis([-4,4,-0.4,0.3]);
subplot(3,2,2)
plot(vi_alongBodyZ(1,:), vi_alongBodyZ(2,:) );
title('Wx along body z')
axis([-2,2,-0.01,0.06]);

subplot(3,2,3)
plot(vi_alongBodyX(1,:), vi_alongBodyX(3,:) );
title('Wy along body x')
axis([-60,20,-0.2,0.3]);
subplot(3,2,4)
plot(vi_alongBodyZ(1,:), vi_alongBodyZ(3,:) );
title('Wy along body z')
axis([-2,2,-0.4,0.3]);

subplot(3,2,5)
plot(vi_alongBodyX(1,:), vi_alongBodyX(4,:) );
title('Wz along body x')
axis([-60,20,-0.4,0.1]);
subplot(3,2,6)
plot(vi_alongthewing(1,:), vi_alongthewing(4,:) );
title('Wz along the wing')
axis([-4,4,-1.5,2.5]);

drawnow;
end

% Computes the induced velocity distribution over one airframe
% segment defined by a point in the body frame, a unit vector and
% a range.
function vi_dist = distribution(inducee, p0, u, r, stack, inducer)
% Generate evenly spaced points:
div = 10;
steps = zeros(div+1, 1);
steps = ffb_linspace(r(1),r(2), div);
vi_dist = zeros(4,length(steps));
ps = zeros(3,length(steps));
for i=1:length(steps);
    p = p0 + steps(i) * u;
    [vi_dist(2:4, i), ~] = getVi(p, inducee, inducer, stack);
    ps(:, i) = p;
    vi_dist(1, i) = steps(i)/inducee.b_m;
end
end

% Generates an array of evenly spaced points between two given scalars a
% and b, for which b > a.
function lspace = ffb_linspace(a, b, n)
dl = (b-a)/n;
lspace = zeros(n+1, 1);
lspace(1) = a;
for k=2:length(lspace)
    lspace(k) = lspace(k-1) + dl;
end

end

% Computes the body frame coordinates of a point on the wing of the given
% UAS for the given distance from the body frame's origin. The third
% parameter indicates the halfplane the point is supposed to lie in
% (1=right wing, -1=left wing). Helpful e.g. to compute the positions of
% the wing tips for an aircraft featuring dihedral and sweep angle.
function p = getWingPoint(ac, r, hlfpl)
% Compute unit vector pointing from the origin to the wing tip:
wingvector = ffb_angle2dcm((hlfpl)*ac.sweep_rad, (hlfpl)*ac.dihedral_rad, 0, 'ZXY')' * [0 hlfpl 0]';
p = wingvector * r;
end

% Computes the induced velocity vector in the follower's body frame at a
% given point defined in the follower's body frame.
function [vi_b, vi_NED]= getVi(p_bfollower, follower, leader, stack)
% Find point coordinates in the leader's wind frame:
% First compute position of the follower's body frame in the
% leader's wind frame.
% relative position vector in the NED frame:
d_NED = body2NED(p_bfollower, follower.p_NED_m, follower.qAttitude) - leader.p_NED_m;
% rotate to the leader's wind frame:
d_wleader = stack.DCM_ew_leader' * d_NED;
% Transform point:
p_wleader = zeros(3,1);
p_wleader = fflib_frame2frame(p_bfollower, d_wleader, ffb_dcm2quat(stack.DCM_windLeader2bodyFollower)');
% Compute the velocities induced by the right and the left
% filament:
% Get induced velocities:
vi_wleader = computeVi(p_wleader, stack.p0_leftFilament_wleader, [1 0 0]', leader, d_wleader) ...
    + computeVi(p_wleader, stack.p0_rightFilament_wleader, [-1 0 0]', leader, d_wleader);
vi_b = stack.DCM_windLeader2bodyFollower * vi_wleader;
vi_NED = stack.DCM_be_follower' * vi_b;
end

% Computes the induced velocity vector for a given filament that
% starts at the given point in the leader's wind frame.
% The unit vector u_senseofrotation_wleader indicates the sense of
% rotation of the vortex (right hand rule).
function vi_wleader = computeVi(p_wleader, p0_filament_wleader, u_senseofrotation_wleader, leader, d_wleader)
% To compute the norm of the induced velocity, we need the
% longitudinal separation to compute the vortex age tau:
dx = d_wleader(1);
% Vortex age tau:
tau = norm(dx / norm(leader.v_NED_mps));
% Disable core:
% tau = 1e-6;
vortexStrength = 0.5 * leader.CL * norm(leader.v_NED_mps) * leader.cbar_m;
% TODO Have a closer look at the empirical value for nu used in eq. 32.
empiricalNu = 0.06*vortexStrength;
rc = 2.24 * sqrt(empiricalNu * tau);
% Compute the radial distance between the filament and the
% point, i.e. the distance in the wind frame's yz plane:
dfp = p_wleader - p0_filament_wleader;
rr = norm(dfp(2:3));
vi_wleader = (vortexStrength/(2*pi*rr)) * (1 - exp(-1.26 * (rr/rc)^2));
% Get unit vector pointing in the direction of the induced
% flow:
v_radius_wleader = [0; dfp(2:3, :)];
u_flow_wleader = fflib_normalize(cross(u_senseofrotation_wleader, v_radius_wleader));
vi_wleader = vi_wleader * u_flow_wleader;
end

% Transforms a given position from the body frame to the NED frame.
function p_NED = body2NED(p_b, pBodyframe_NED, qAttitude_NED)
p_NED = fflib_frame2frame(p_b, pBodyframe_NED, qAttitude_NED);
end

% Finds those UASs whose wake vortices have a relevant impact on this
% one.
function [indices, n] = selectDominantUASs(ID, VortexSig, stack)
% Figure out how many active UAS there are by finding zero fields in the
% shared state:
nUAS = length(VortexSig);
for k=1:length(VortexSig)
    if VortexSig(k).timeOfLastUpdate < eps
        nUAS = k-1;
        break;
    end
end
% Check for each other UAS is it is in front of this one:
n = 0;
indices = zeros(10,1);
if nUAS > 2
    % The index one is a virtual leader, ignore:
    for k=2:nUAS
        if k ~= ID
            dp_e = VortexSig(k).p_NED_m - VortexSig(ID).p_NED_m;
            if ID == 3 && VortexSig(ID).timeOfLastUpdate > 0.1;
                bp = 0;
            end
            % rotate to this UAS's wind frame:
            dp_w = stack.DCM_ew_this' * dp_e;
            if dp_w(3) > 0
                n = n + 1;
                indices(n) = k;
            end
        end
    end
end
n = 0;
end