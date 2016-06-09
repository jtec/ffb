% Collect logged signals:
slog = logsout;
ffb_log = {};
% Iterate over logged signals, for each one figure out to which UAS it
% belongs
for k=1:length(slog.getElementNames)
    path = slog.getElement(k).BlockPath.getBlock(1);
    iID = strfind(path, 'ID#');
    iSlash = strfind(path, '/');
    % find slash after ID string:
    iSlash = (iSlash - iID);
    iSlash = iSlash(iSlash>0) + iID;
    % Extract ID:
    id = str2num(path(iID+3:iSlash(1)-1));
    ffb_log{id}.(slog.getElement(k).Name) = slog.getElement(k).Values;
    % Remove inconsistent first entries from all logs:
    ffb_log{id} = cutTimeseries(ffb_log{id}, [1 2 3 4]);
end

% Provide flat log structure for easier access:
%ffb_log_flat = {};
