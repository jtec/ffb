%% Analyse:
%% Plot approximated effective induced velocities and angular rates:
figure(3);
hold off;
xdata = dp_w_ts.Data(:, 2)/b;
ydata = ffb_vortexdiagnostic_log_Vi_NED_mps.Data(:,3);
plot(xdata,ydata);
grid on;
ylabel('$\frac{dz}{b} [-]$');

%% Plot induced velocity in y-z plane:
close('all');
y = dp_w_ts.Data(:, 2);
z = dp_w_ts.Data(:, 3);
v_e = ffb_vortexdiagnostic_log_Vi_NED_mps.Data;
v_w = zeros(size(v_e));
for k=1:length(v_e)
    v_w(k, :) = DCM_ew'*v_e(k, :)';
end
vy = v_w(:, 2);
vz = v_w(:, 3);
figure(1);
hold off;
quiver(y/b, z/b, vy, vz);
xlabel('$\frac{dy}{b} [-]$');
ylabel('$\frac{dz}{b} [-]$');
grid on;

%% Try isosurface plot:
% Generate coordinate arrays compatible with matlab's meshgrid():
V = zeros(size(dX));
for k=1:length(dp_w)
    V(k) = norm(v_w(k, :));
end
figure(2);
p = patch(isosurface(dX./b,dY./b,dZ./b,V,0.3));
isonormals(dX,dY,dZ,V,p)
p.FaceColor = 'red';
p.EdgeColor = 'none';
daspect([1,1,1])
view(3);
axis tight;
grid on;
camlight;
lighting gouraud;
return;

%% Generate vortex measurements:
ffb_vortexdiagnostic_par.S


b = uavsim.cularis.b;
logindex = ceil(10/uavsim.simConfig.tsample_model);
ffb_vortexdiagnostic_par.Cl = -(2*uavsim.formation(1).y_trim(24))/(1.225*15^2*uavsim.cularis.S);
ffb_vortexdiagnostic_par.alpha = uavsim.cularis.alpha_trim;
ffb_vortexdiagnostic_par.beta = 0;
ffb_vortexdiagnostic_par.qAttitude = angle2quat(uavsim.cularis.phithetapsi0(3), ...
    uavsim.cularis.phithetapsi0(2), ...
    uavsim.cularis.phithetapsi0(1));
%%
ffb_vortexdiagnostic_par.n = [1 100 1]';
dx = linspace(-b/1000, -b/1000, ffb_vortexdiagnostic_par.n(1));
dy = linspace(-b/100, b/100, ffb_vortexdiagnostic_par.n(2));
dz = linspace(-b/100, b/100, ffb_vortexdiagnostic_par.n(3));

[dX,dY,dZ] = meshgrid(dx,dy,dz);
ndps = size(dX, 1)*size(dX, 2)*size(dX, 3);
dp_w = zeros(ndps, 3);
dp_e = dp_w;
for k=1:length(dp_w)
    dp_w(k, :) = [dX(k) dY(k) dZ(k)];
    % Rotate to wind frame:
    DCM_bw = ffb_dcmbody2wind(ffb_vortexdiagnostic_par.alpha, ...
                              ffb_vortexdiagnostic_par.beta)';
    % DCM (body) -> NED
    DCM_eb = quat2dcm(ffb_vortexdiagnostic_par.qAttitude)';
    % DCM (wind) -> NED
    DCM_ew = DCM_eb * DCM_bw;
    dp_e(k, :) = DCM_ew * dp_w(k, :)';
end
ffb_vortexdiagnostic_par.v_NED = uavsim.formation(1).y_trim(14:16);


% Generate timeseries for simulation:
t = 0:uavsim.simConfig.tsample_model:(length(dp_w)-1)*uavsim.simConfig.tsample_model;
dp_e_ts = timeseries(dp_e, t);
dp_w_ts = timeseries(dp_w, t);
sim('ffb_vortexdiagnostic_sweep');

return;