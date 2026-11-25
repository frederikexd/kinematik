% ============================================================================
%  KinematiK tyre benchmark: reference Fy from MFeval (MATLAB)
%  Run once in MATLAB with MFeval installed (Add-On Explorer: "MFeval").
%  It finds the .tir files that ship with MFeval, evaluates pure lateral force
%  on a fixed grid, and writes one CSV per .tir plus a copy of the .tir into
%  the current folder. Send those files back.
% ============================================================================
function mfeval_reference()
    here = pwd;
    root = fileparts(which('mfeval'));
    if isempty(root)
        error('MFeval is not on the MATLAB path. Install it from Add-On Explorer first.');
    end
    % look in the MFeval folder and two levels up (the toolbox root holds the examples)
    tirs = [dir(fullfile(root, '**', '*.tir')); dir(fullfile(fileparts(fileparts(root)), '**', '*.tir'))];
    [~, keep] = unique(fullfile({tirs.folder}, {tirs.name}));
    tirs = tirs(keep);
    if isempty(tirs)
        error('No .tir files found near %s. Tell Claude where MFeval''s example .tir files are.', root);
    end
    fprintf('MFeval at %s\n', root);
    for k = 1:numel(tirs)
        f = fullfile(tirs(k).folder, tirs(k).name);
        try
            run_one(f, here);
        catch err
            fprintf('  %s: skipped (%s)\n', tirs(k).name, err.message);
        end
    end
end

function run_one(f, here)
    txt = fileread(f);
    fnomin = tirvalue(txt, 'FNOMIN');
    fittyp = tirvalue(txt, 'FITTYP');
    nompres = tirvalue(txt, 'NOMPRES');                 % NaN for MF5.2
    alpha = deg2rad(-15:1:15)';
    fzs = fnomin * [0.5 0.75 1.0 1.25 1.5];
    gammas = deg2rad([0 -2 -4]);
    rows = [];
    for fz = fzs
        for g = gammas
            n = numel(alpha);
            % [Fz kappa alpha gamma phit Vx (P)]: pure lateral (kappa 0), no turn slip, 16.7 m/s
            in = [fz*ones(n,1) zeros(n,1) alpha g*ones(n,1) zeros(n,1) 16.7*ones(n,1)];
            if ~isnan(nompres)
                in = [in nompres*ones(n,1)];
            end
            out = mfeval(f, in, 211);                    % 2xx: no input limits, 1x: combined (= pure at kappa 0)
            rows = [rows; in(:,1:4) out(:,2)];           %#ok<AGROW>  column 2 of the output is Fy
        end
    end
    [~, name] = fileparts(f);
    T = array2table(rows, 'VariableNames', {'Fz_N', 'kappa', 'alpha_rad', 'gamma_rad', 'Fy_N'});
    writetable(T, fullfile(here, ['mfeval_ref_' name '.csv']));
    copyfile(f, fullfile(here, ['mfeval_ref_' name '.tir']));
    fprintf('  %s: FITTYP %g, FNOMIN %g N, %d points written\n', name, fittyp, fnomin, size(rows, 1));
end

function v = tirvalue(txt, key)
    t = regexp(txt, ['(?m)^\s*' key '\s*=\s*([-+0-9.eE]+)'], 'tokens', 'once');
    if isempty(t), v = NaN; else, v = str2double(t{1}); end
end
