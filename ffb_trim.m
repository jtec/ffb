% Compute trim conditions
for k = 1:length(uavsim.formation)
    % Define initial conditions:
    v0 = uavsim.formation(k).v0_NED;
    p0 = uavsim.formation(k).p0_NED;
    chi0 = -atan2(v0(2), v0(1));
    gamma0 = -v0(3)/norm(v0(1:2));
    Va0 = norm(v0);
    % Compute trim:
    % Do a cached call to save time:
    args = {};
    args{end+1} = uavsim.cularis.trimmodel;
    args{end+1} = chi0;
    args{end+1} = gamma0;
    args{end+1} = Va0;
    [x_trim, u_trim, y_trim, trimresidual] = cachedcall(@uavsimblockset_computeTrim, args);
    
    [uavsim.formation(k).alpha_trim, uavsim.formation(k).beta_trim] = alphabeta(x_trim(1:3)');
    uavsim.formation(k).trimresidual = trimresidual;
    % Set initial conditions to trim conditions:
    uavsim.formation(k).uvw0     = x_trim(1:3);            % initial body frame velocity
    uavsim.formation(k).phithetapsi0   = x_trim(4:6);      % initial Euler angles
    uavsim.formation(k).omega0     = x_trim(7:9);          % initial body frame angular rates
    uavsim.formation(k).u0.da     = u_trim(1);
    uavsim.formation(k).u0.de     = u_trim(2);
    uavsim.formation(k).u0.dr     = u_trim(3);
    uavsim.formation(k).u0.den     = u_trim(4);
    uavsim.formation(k).u0.df     = u_trim(5);
    uavsim.formation(k).u_trim     = u_trim;
    
    % Build linear models for controller synthesis etc.
    % Do a cached call to save time:
    args = {};
    args{end+1} = uavsim.cularis.trimmodel;
    args{end+1} = x_trim;
    args{end+1} = u_trim;
    args{end+1} = y_trim;
    
    uavsim.formation(k).linearModels = cachedcall(@getLinearModels, args);
end

% Trim inputs states etc. for currently simulated model:
flds = fields(uavsim.formation(1));
for k=1:length(flds)
    uavsim.cularis.(flds{k}) = uavsim.formation(1).(flds{k});
end