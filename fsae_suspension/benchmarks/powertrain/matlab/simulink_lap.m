function simulink_lap(mass_kg, chan, outdir)
% SIMULINK_LAP  Turn a finished Simscape Vehicle Templates lap of the Achilles car into KinematiK benchmark files.
%
%   simulink_lap                        % mass from the tire Fz at the start, every signal found by name
%   simulink_lap(mass_kg)               % give the mass yourself
%   simulink_lap([], struct('volts', 'path', 'amps', 'path'))   % pick signals by their path
%   simulink_lap([], struct(), 'runs/hock')                       % write into another folder than achilles/
%
% Run it after a lap event of sm_car has finished (Hockenheim flat, the FSAE set's flat track, or any CRG track),
% with logsout_sm_car in the base workspace. Roles that can be picked: speed, x, y, volts, amps, motor; the paths are
% listed in achilles/simulink_signals.txt. It writes:
%   achilles/simulink_lap.csv        the lap in MoTeC CSV layout: Time, Ground Speed, Distance, X, Y, Pack Voltage,
%                                    Pack Current (positive discharging), Motor Speed, resampled at 100 Hz
%   achilles/simulink_signals.txt    every logged signal
%   achilles/inputs.json, achilles/motor_curve.csv   the same car for KinematiK (export_achilles.m)
% and prints the command that grades KinematiK against the lap.
if nargin < 1, mass_kg = []; end
if nargin < 2 || isempty(chan), chan = struct(); end
if nargin < 3 || isempty(outdir), outdir = 'achilles'; end
if ~evalin('base', 'exist(''logsout_sm_car'', ''var'')')
    error('simulink_lap:log', ['No logsout_sm_car in the workspace: open the project (startup_sm_car), choose ' ...
        'the Achilles car and a lap event (Hockenheim flat), run it, then call simulink_lap again.']);
end
sig = flatten(evalin('base', 'logsout_sm_car'));
if ~exist(outdir, 'dir'), mkdir(outdir); end
fid = fopen(fullfile(outdir, 'simulink_signals.txt'), 'w');
fprintf(fid, '%s\n', sig.path{:});
fclose(fid);
leaf = regexprep(sig.path, '.*[./]', '');
has = @(re) ~cellfun(@isempty, regexpi(sig.path, re, 'once'));
leafis = @(re) ~cellfun(@isempty, regexpi(leaf, re, 'once'));

% ---- the signals: first the paths MathWorks' own plots read (sm_car_plot1speed, sm_car_plot7power), then by name -----
exact = @(p) find(strcmp(sig.path, p), 1);
body = ~has('susp|steer|linkage|shock|whl|tire|wheel|motor|driver');      % not a part of the suspension or drive
ks = pick(chan, 'speed', sig, [exact('VehBus.Chassis.Body.CG.vx'); ...
    find(strcmpi(leaf, 'vx') & has('body|cg') & body, 1); find(strcmpi(leaf, 'vx') & body, 1)], true);
kx = pick(chan, 'x', sig, [exact('VehBus.World.x'); find(strcmpi(leaf, 'x') & has('world') & body, 1)], false);
ky = pick(chan, 'y', sig, [exact('VehBus.World.y'); find(strcmpi(leaf, 'y') & has('world') & body, 1)], false);
kv = pick(chan, 'volts', sig, [exact('VehBus.Power.Battery.v'); ...
    find(has('batt') & leafis('^v$|^vt$|volt|^v_?batt'), 1)], true);
ki = pick(chan, 'amps', sig, [exact('VehBus.Power.Battery.i'); find(has('batt') & leafis('^i$|curr|^i_?batt'), 1)], true);
km = pick(chan, 'motor', sig, [exact('VehBus.Power.MotorL1.w'); ...
    find(has('motor') & leafis('^w$|spd|speed|rpm') & ~has('trq|torque|ref|cmd'), 1)], false);
kz = find(~cellfun(@isempty, regexp(sig.path, 'Chassis\.Whl[^.]*\.Fz$', 'once')));

fprintf('speed signal: %s\n', sig.path{ks});
[t, v] = series(sig.ts{ks});
unit = units(sig.ts{ks});
if any(strcmpi(unit, {'km/h', 'kph'})), v = v / 3.6; end
v = abs(v);
tq = (t(1):0.01:t(end))';
V = interp1(t, v, tq, 'linear');
U = at(sig.ts{kv}, tq);
I = at(sig.ts{ki}, tq);
Fz = zeros(size(tq));
for j = 1:numel(kz), Fz = Fz + abs(at(sig.ts{kz(j)}, tq)); end

% ---- the start: skip the instants before the battery and the tires have taken up their state ------------------------
good = U >= 0.5 * median(U);
if numel(kz) >= 4, good = good & Fz >= 0.5 * median(Fz); end
i0 = find(good, 1);
keep = (i0:numel(tq))';
if tq(i0) > tq(1), fprintf('skipped the first %.2f s (battery or tire loads still starting up)\n', tq(i0) - tq(1)); end
tq = tq(keep); V = V(keep); U = U(keep); I = I(keep); Fz = Fz(keep);

% ---- mass: the four tire loads less the downforce, over the lap ------------------------------------------------------
if isempty(mass_kg)
    if numel(kz) < 4
        error('simulink_lap:mass', ['Tire Fz is not in logsout_sm_car: give the mass, simulink_lap(mass_kg) ' ...
            '(Plot tire forces: sum of the four Fz on a straight at low speed / 9.81).']);
    end
    try
        A = evalin('base', 'Vehicle.Chassis.Aero');
        down = 0.5 * A.rho_air.Value * (-A.CL.Value) * A.ARef.Value * V .^ 2;
    catch
        down = 0;
        warning('simulink_lap:aero', 'Vehicle.Chassis.Aero not found: mass taken without removing the downforce');
    end
    mass_kg = median(Fz - down) / 9.81;
    fprintf('mass from the %d tire loads less the downforce, median over the lap: %.1f kg\n', numel(kz), mass_kg);
end

cols = {tq, V, [0; cumsum(0.5 * (V(2:end) + V(1:end-1)) .* diff(tq))]};
names = {'Time', 'Ground Speed', 'Distance'};
unitrow = {'s', 'm/s', 'm'};
if ~isempty(kx) && ~isempty(ky)
    cols(end+1:end+2) = {at(sig.ts{kx}, tq), at(sig.ts{ky}, tq)};
    names(end+1:end+2) = {'X', 'Y'};
    unitrow(end+1:end+2) = {'m', 'm'};
end
a = gradient(V, tq);
if sum(U .* I .* (a > 1)) < 0          % make the current positive while the car accelerates (discharging)
    I = -I;
end
cols(end+1:end+2) = {U, I};
names(end+1:end+2) = {'Pack Voltage', 'Pack Current'};
unitrow(end+1:end+2) = {'V', 'A'};
kT = find(strcmp(sig.path, 'VehBus.Power.Thermal.TBattery'), 1);
if ~isempty(kT)                                   % the pack's temperature (K)
    cols{end+1} = at(sig.ts{kT}, tq);
    names{end+1} = 'Pack Temperature';
    unitrow{end+1} = 'K';
end
kq = cellfun(@(w) find(strcmp(sig.path, ['VehBus.Power.Motor' w '.trq']), 1), {'L1', 'R1', 'L2', 'R2'}, 'UniformOutput', false);
kw = cellfun(@(w) find(strcmp(sig.path, ['VehBus.Power.Motor' w '.w']), 1), {'L1', 'R1', 'L2', 'R2'}, 'UniformOutput', false);
if all(~cellfun(@isempty, [kq kw]))               % the four motors' shaft power together (W)
    pm = zeros(size(tq));
    for k = 1:4, pm = pm + at(sig.ts{kq{k}}, tq) .* at(sig.ts{kw{k}}, tq); end
    cols{end+1} = pm;
    names{end+1} = 'Motor Power';
    unitrow{end+1} = 'W';
end
if ~isempty(km)
    w = abs(at(sig.ts{km}, tq));
    if ~any(strcmpi(units(sig.ts{km}), {'rpm', 'rev/min'})), w = w * 60 / (2 * pi); end   % Simscape: rad/s
    cols{end+1} = w;
    names{end+1} = 'Motor Speed';
    unitrow{end+1} = 'rpm';
end
M = [cols{:}];
fid = fopen(fullfile(outdir, 'simulink_lap.csv'), 'w');
fprintf(fid, '"Format","MoTeC CSV File"\n"Venue","Simulink sm_car lap"\n\n');
fprintf(fid, '%s\n', strjoin(strcat('"', names, '"'), ','));
fprintf(fid, '%s\n\n', strjoin(strcat('"', unitrow, '"'), ','));
fprintf(fid, [strjoin(repmat({'%.6f'}, 1, numel(names)), ',') '\n'], M');
fclose(fid);
fprintf('signals used:\n  speed  %s\n  volts  %s\n  amps   %s\n', sig.path{ks}, sig.path{kv}, sig.path{ki});
if ~isempty(kx), fprintf('  x, y   %s, %s\n', sig.path{kx}, sig.path{ky}); end
if ~isempty(km), fprintf('  motor  %s\n', sig.path{km}); end
fprintf('Simulink lap: %.3f s, %.1f m, %.1f to %.1f m/s, pack %.1f to %.1f V, peak %.0f A, mass %.1f kg\n', ...
    tq(end) - tq(1), cols{3}(end), min(V), max(V), min(U), max(U), max(I), mass_kg);
if isempty(kx)
    warning('simulink_lap:xy', ['no X/Y position found: the lap time will not be compared. Pick them, e.g. ' ...
        'simulink_lap([], struct(''x'', ''path'', ''y'', ''path'')) from achilles/simulink_signals.txt']);
end

% ---- alignment as the car runs: each wheel's toe (deg), median over the straights ------------------------------------
toe = [];
kt = cellfun(@(p) find(strcmp(sig.path, p), 1), {'VehBus.Chassis.SuspA1.WhlL.aToe', 'VehBus.Chassis.SuspA1.WhlR.aToe', ...
    'VehBus.Chassis.SuspA2.WhlL.aToe', 'VehBus.Chassis.SuspA2.WhlR.aToe'}, 'UniformOutput', false);
kg = find(strcmp(sig.path, 'VehBus.Chassis.Body.CG.gy'), 1);
if all(~cellfun(@isempty, kt)) && ~isempty(kg)
    straight = abs(at(sig.ts{kg}, tq)) < 0.5;
    if nnz(straight) < 10, straight = true(size(tq)); end
    toe = cellfun(@(k) median(at(sig.ts{k}, tq(straight))), kt);
    fprintf('toe as it runs (deg, straights): %s\n', mat2str(toe, 3));
end
% ---- rotating inertia as the car has it: each wheel's spin inertia (motor through its gearbox included) from
% J alpha = T_motor ratio - Fx r - |My| while the wheel speeds up (no brake, past the first 3 s), least squares; the
% rolling radius from road speed over wheel speed. The Vehicle structure's TireA*.Inertia does not reach the wheel.
rot = [];
kn = cellfun(@(w) find(strcmp(sig.path, ['VehBus.Chassis.Whl' w '.n']), 1), {'L1', 'R1', 'L2', 'R2'}, 'UniformOutput', false);
if all(~cellfun(@isempty, kn))
    Vv = evalin('base', 'Vehicle');
    ratio = Vv.Powertrain.Driveline.GearboxL1.ratio.Value;
    nm = zeros(size(tq));
    for k = 1:4, nm = nm + at(sig.ts{kn{k}}, tq) / 4; end
    steady = abs(gradient(V, tq)) < 0.2 & V > 5;
    r_eff = median(V(steady) ./ nm(steady));
    J = 0;
    for w = {'L1', 'R1', 'L2', 'R2'}
        g = @(p) at(sig.ts{find(strcmp(sig.path, p), 1)}, tq);
        n = g(['VehBus.Chassis.Whl' w{1} '.n']);
        al = gradient(n, tq);
        y = ratio * g(['VehBus.Power.Motor' w{1} '.trq']) - g(['VehBus.Chassis.Whl' w{1} '.Fx']) * r_eff ...
            - abs(g(['VehBus.Chassis.Whl' w{1} '.My']));
        m = al > 3 & tq > tq(1) + 3 & abs(g(['VehBus.Brakes.MBrake' w{1}])) < 1;
        J = J + sum(al(m) .* y(m)) / sum(al(m) .^ 2);
    end
    rot = 1 + J / (r_eff ^ 2 * mass_kg);
    fprintf('spin inertia of the four wheels together: %.2f kg m^2 on a rolling radius of %.4f m: factor %.3f\n', ...
        J, r_eff, rot);
end
export_achilles(mass_kg, outdir, toe, rot);
fprintf(['\nNow, in the kinematik repo (fsae_suspension folder), with the achilles folder copied into it:\n' ...
    '  python scripts/powertrain_benchmark.py simulink-lap --inputs achilles/inputs.json ' ...
    '--lap achilles/simulink_lap.csv\n']);
end

function k = pick(chan, role, sig, found, required)
if isfield(chan, role)
    k = find(strcmp(sig.path, chan.(role)), 1);
    if isempty(k), error('simulink_lap:signal', 'no signal named %s (role %s)', chan.(role), role); end
    return
end
k = found(:);
k = k(~isnan(k));
if ~isempty(k), k = k(1); end
if isempty(k) && required
    error('simulink_lap:signal', ['No %s signal found: pick one from achilles/simulink_signals.txt and call ' ...
        'simulink_lap([], struct(''%s'', ''that path''))'], role, role);
end
end

function [t, y] = series(ts)
t = double(ts.Time(:));
y = double(squeeze(ts.Data));
if size(y, 1) ~= numel(t), y = y.'; end      % [k x N] after squeeze: one row per sample
y = y(:, 1);                                 % a vector signal: its first element
[t, iu] = unique(t);
y = y(iu);
end

function y = at(ts, tq)
[t, y] = series(ts);
y = interp1(t, y, tq, 'linear', 'extrap');
end

function u = units(ts)
u = '';
try, u = char(ts.DataInfo.Units); catch, end
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
    if isnumeric(x.Data) && ~isempty(x.Data) && numel(unique(x.Time)) >= 2   % a trace, not a constant
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
