function exportfigure2latex(fhdl, name)
% Set white background:
set(fhdl, 'color', [1 1 1])
%try
    name = latexify(name);
    % Make given figure the current figure:
    figure(fhdl)
    % export_fig(name, '-png');
    export_fig(name, '-pdf');
    
%catch e
%    disp([mfilename '>> Exception: ' e.message]);
%end

set(0, 'DefaultAxesFontSize',12);
set(0, 'DefaultTextFontSize',12);
disp([mfilename '>> Done exporting figure ' name '.']);

    function cleaned = latexify(in)
        cleaned = strrep(in, '_', '-');
        % cleaned = strrep(cleaned, '-', '');
        % cleaned = strrep(cleaned, '=', 'eq');
    end
end