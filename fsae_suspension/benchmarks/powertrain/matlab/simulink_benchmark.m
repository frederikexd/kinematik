function simulink_benchmark(mass_kg, speed_signal)
% SIMULINK_BENCHMARK  Turn a finished Simscape Vehicle Templates acceleration run into KinematiK benchmark files.
%
%   simulink_benchmark                  % mass from the tire Fz at t = 0, speed signal found by name
%   simulink_benchmark(mass_kg)         % give the mass yourself
%   simulink_benchmark([], 'path')      % pick the speed signal (paths are listed in achilles/simulink_signals.txt)
%
% Run it after an acceleration test of sm_car has finished, with logsout_sm_car in the base workspace. It writes:
%   achilles/inputs.json, achilles/motor_curve.csv  the same car for KinematiK (export_achilles.m)
%   achilles/simulink_accel.csv                     Simulink's run in MoTeC CSV layout (Time, Ground Speed)
%   achilles/simulink_signals.txt                   every logged signal, to choose another speed signal
% and prints the command that grades KinematiK against the Simulink run.
if nargin < 1, mass_kg = []; end
if nargin < 2, speed_signal = ''; end
if ~evalin('base', 'exist(''logsout_sm_car'', ''var'')')
    error('simulink_benchmark:log', ['No logsout_sm_car in the workspace: open the project (startup_sm_car), ' ...
        'run the acceleration test in sm_car, then call simulink_benchmark again.']);
end
sig = flatten(evalin('base', 'logsout_sm_car'));
if ~exist('achilles', 'dir'), mkdir('achilles'); end
fid = fopen(fullfile('achilles', 'simulink_signals.txt'), 'w');
fprintf(fid, '%s\n', sig.path{:});
fclose(fid);

% ---- mass: sum of the four tire normal loads before the car moves ------------------------------------------------
if isempty(mass_kg)
    k = find(~cellfun(@isempty, regexpi(sig.path, 'Fz', 'once')));
    if numel(k) < 4
        error('simulink_benchmark:mass', ['Tire Fz is not in logsout_sm_car: give the mass, ' ...
            'simulink_benchmark(mass_kg) (Plot tire forces: sum of the four Fz at the start / 9.81).']);
    end
    if numel(k) > 4
        warning('simulink_benchmark:fz', 'more than four Fz signals; using the first four:\n  %s', ...
            strjoin(sig.path(k(1:4)), '\n  '));
    end
    mass_kg = sum(cellfun(@(ts) abs(ts.Data(1)), sig.ts(k(1:4)))) / 9.81;
end

% ---- speed: the chassis's forward velocity ------------------------------------------------------------------------
if isempty(speed_signal)
    leaf = regexprep(sig.path, '.*[./]', '');
    k = find(strcmpi(leaf, 'vx'), 1);
    if isempty(k), k = find(~cellfun(@isempty, regexpi(leaf, '^v(eh(icle)?)?_?x$|^vVeh$', 'once')), 1); end
    if isempty(k)
        k = find(~cellfun(@isempty, regexpi(sig.path, 'speed|vel', 'once')) & ...
            cellfun(@isempty, regexpi(sig.path, 'wheel|motor|rpm|ang|w$|tire', 'once')), 1);
    end
    if isempty(k)
        error('simulink_benchmark:speed', ['No forward-speed signal found: pick one from ' ...
            'achilles/simulink_signals.txt and call simulink_benchmark([], ''that path'').']);
    end
else
    k = find(strcmp(sig.path, speed_signal), 1);
    if isempty(k), error('simulink_benchmark:speed', 'no signal named %s', speed_signal); end
end
ts = sig.ts{k};
t = double(ts.Time(:));
v = abs(double(squeeze(ts.Data)));
v = v(:);
unit = '';
try, unit = char(ts.DataInfo.Units); catch, end
if any(strcmpi(unit, {'km/h', 'kph'})), v = v / 3.6; end
[t, iu] = unique(t);
v = v(iu);

fid = fopen(fullfile('achilles', 'simulink_accel.csv'), 'w');
fprintf(fid, '"Format","MoTeC CSV File"\n"Venue","Simulink sm_car"\n\n"Time","Ground Speed"\n"s","m/s"\n\n');
fprintf(fid, '%.6f,%.6f\n', [t, v]');
fclose(fid);

d = [0; cumsum(0.5 * (v(2:end) + v(1:end-1)) .* diff(t))];
i0 = max(find(v > 0.5, 1) - 1, 1);
d = d - d(i0);
i1 = find(d >= 75, 1);
fprintf('speed signal: %s (max %.1f m/s)\n', sig.path{k}, max(v));
if isempty(i1)
    warning('simulink_benchmark:75m', 'the run never reaches 75 m (%.1f m): lengthen the test', d(end));
else
    fprintf('Simulink 75 m: %.3f s\n', interp1(d(i1-1:i1), t(i1-1:i1), 75) - t(i0));
end

export_achilles(mass_kg);
fprintf(['\nNow, in the kinematik repo (fsae_suspension folder), with the achilles folder copied into it:\n' ...
    '  python scripts/powertrain_benchmark.py motec --kind ev --inputs achilles/inputs.json ' ...
    '--accel achilles/simulink_accel.csv\n']);
end

function sig = flatten(ds)
% Every timeseries in a Simulink.SimulationData.Dataset, buses opened, with its full path.
sig.path = {};
sig.ts = {};
for i = 1:ds.numElements
    el = ds.getElement(i);
    name = el.Name;
    if isempty(name), try, name = el.BlockPath.getBlock(1); catch, name = sprintf('signal%d', i); end, end
    sig = walk(sig, el.Values, name);
end
end

function sig = walk(sig, x, path)
if isa(x, 'timeseries')
    if isnumeric(x.Data) && ~isempty(x.Data)
        sig.path{end+1, 1} = path;
        sig.ts{end+1, 1} = x;
    end
elseif isstruct(x)
    f = fieldnames(x);
    for j = 1:numel(f)
        sig = walk(sig, x(1).(f{j}), [path '.' f{j}]);
    end
end
end
