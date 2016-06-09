% Removes the first few samples from a given structure of timeseries. The
% members of the set can be structures of timeseries themselves.
function tss_cut = cutTimeseries( tss, idxs )
tss_cut = tss;
names = fields(tss);
for i=1:length(names)
    if isa(tss.(names{i}), 'timeseries')
        ts = tss.( names{i} );
        try
            ts = delsample(ts,'Index', idxs);
        catch e
            disp([mfilename '>> Exception: ' e.message]);
            ts = cutTimeseries(tss.( names{i} ));
        end
        tss_cut.(names{i}) = ts;
    end
end

end

