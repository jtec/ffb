%% Analyse:
% Plot induced velocity in y-z plane:
close('all');
y = dp_ts.Data(:, 2);
z = dp_ts.Data(:, 3);
v = ffb_vortexdiagnostic_log_Vi_NED_mps.Data;
vy = v(:, 2);
vz = v(:, 3);
figure(1);
quiver(y/b, -z/b, vy, -vz);
xlabel('$\frac{dy}{b} [-]$');
ylabel('$\frac{dz}{b} [-]$');
grid on;



% Try isosurface plot:
% Generate coordinate arrays compatible with matlab's meshgrid():
V = zeros(size(dX));
for k=1:length(dp)
    V(k) = norm(v(k, :));
end
figure(2);
p = patch(isosurface(dX,dY,dZ,V,0.4));
isonormals(dX,dY,dZ,V,p)
p.FaceColor = 'red';
p.EdgeColor = 'none';
daspect([1,1,1])
view(3); 
axis tight;
camlight; 
lighting gouraud;

return;

%% Generate vortex measurements:
b = uavsim.cularis.b;
logindex = ceil(10/uavsim.simConfig.tsample_model);
ffb_vortexdiagnostic_par.Cl = ffb_vortexdiagnostic_log_CL.Data(logindex);
ffb_vortexdiagnostic_par.alpha = ffb_vortexdiagnostic_log_alpha.Data(logindex);
ffb_vortexdiagnostic_par.beta = ffb_vortexdiagnostic_log_beta.Data(logindex);
ffb_vortexdiagnostic_par.qAttitude = ffb_vortexdiagnostic_log_qAttitude.Data(logindex, :);

ffb_vortexdiagnostic_par.n = [10 100 100]';
dx = linspace(-5*b, -0.1*b, ffb_vortexdiagnostic_par.n(1));
dy = linspace(-b, b, ffb_vortexdiagnostic_par.n(2));
dz = linspace(-b/2, b/2, ffb_vortexdiagnostic_par.n(3));

[dX,dY,dZ] = meshgrid(dx,dy,dz);
ndps = size(dX, 1)*size(dX, 2)*size(dX, 3);
dp = zeros(ndps, 3);
for k=1:length(dp)
    dp(k, :) = [dX(k) dY(k) dZ(k)];
end
% Generate timeseries for simulation:
t = 0:uavsim.simConfig.tsample_model:(length(dp)-1)*uavsim.simConfig.tsample_model;
dp_ts = timeseries(dp, t);
sim('ffb_vortexdiagnostic_sweep');

return;