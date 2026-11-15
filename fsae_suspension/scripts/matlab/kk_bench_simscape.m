function kk_bench_simscape(root)
%KK_BENCH_SIMSCAPE  Link-load and direct-acting benchmarks in Simscape Multibody (R2025b).
%
%   kk_bench_simscape            % finds the inputs: the repo around this file, the current folder, or
%                                % a kinematik_matlab_standalone folder in it or in MATLAB Drive
%   kk_bench_simscape(root)      % root = the folder holding manifests/ and benchmarks/
%
%   Shares no code with KinematiK: reads only the hardpoints (manifests, benchmarks/direct_acting/corners) and
%   the load cases (benchmarks/link_loads/limits.json), builds each corner as a Simscape Multibody model, and
%   writes the CSV files the Python compare scripts read:
%     benchmarks/link_loads/matlab_results.csv
%     benchmarks/direct_acting/matlab_damper.csv, matlab_loads.csv
%   Then, in the repo: python scripts/link_load_benchmark.py and python scripts/direct_acting_benchmark.py.
%
%   Models (mm, N; no gravity; unit masses, which a zero-DOF mechanism does not feel):
%     member forces  upright on a 6-DOF joint held by Distance Constraint blocks (two-force members); a damper
%                    on the lower wishbone puts that wishbone on a Revolute Joint with a Spherical Joint to the
%                    upright. Loads by External Force and Torque blocks, resolved in the world frame. Axial
%                    force = the constraint's signed force; its sign is fixed by a closed-form case first.
%     damper length  wishbones on Revolute Joints, Spherical ball joints, the tie rod as a Distance
%                    Constraint; the Kinematics Solver drives the lower wishbone angle and reads wheel-centre
%                    travel and the damper's end-to-end vector.
%
%   If a block parameter is not found, the error lists the parameter names this release has: send that back.

if nargin < 1
    here = fileparts(mfilename('fullpath'));
    cand = {fileparts(fileparts(here)), here, pwd, fullfile(pwd, 'kinematik_matlab_standalone'), ...
            fullfile(fileparts(pwd), 'kinematik_matlab_standalone'), '/MATLAB Drive/kinematik_matlab_standalone'};
    root = '';
    for i = 1:numel(cand)
        if isfile(fullfile(cand{i}, 'benchmarks', 'link_loads', 'limits.json')), root = cand{i}; break; end
    end
    if isempty(root)
        error('Could not find the input folder. Run kk_bench_simscape(''<path to kinematik_matlab_standalone>'').');
    end
end
fprintf('Inputs from %s\n', root);
LL = fullfile(root, 'benchmarks', 'link_loads');
DA = fullfile(root, 'benchmarks', 'direct_acting');
reg = jsondecode(fileread(fullfile(LL, 'limits.json')));
cases = reg.load_cases;
caseNames = fieldnames(cases);
ALL6 = {'UF','UR','LF','LR','TR','PR'};

sgn = calibrate();
fprintf('Calibration passed (signed-force sign %+d).\n', sgn);

% ---- link loads ---------------------------------------------------------------------------------------
llCorners = {'front_v11_final','rear_v11_final','front_v11_joints50'};
fid = fopen(fullfile(LL, 'matlab_results.csv'), 'w');
fprintf(fid, '# matlab: Simscape Multibody %s, scripts/matlab/kk_bench_simscape.m. Axial force, N, + tension.\n', version);
fprintf(fid, 'corner,case,%s\n', strjoin(ALL6, ','));
for k = 1:numel(llCorners)
    m = jsondecode(fileread(fullfile(root, 'manifests', [llCorners{k} '.recorded.genesis.json'])));
    P = points(m.recorded.winner_hardpoints);
    for c = 1:numel(caseNames)
        f = memberForces(P, cases.(caseNames{c}), false, false, sgn);
        fprintf(fid, '%s,%s,%s\n', llCorners{k}, caseNames{c}, rowOf(f, ALL6));
    end
    fprintf('link loads: %s done\n', llCorners{k});
end
fclose(fid);

% ---- direct-acting ------------------------------------------------------------------------------------
daCorners = {'direct_upright','direct_lower'};
fd = fopen(fullfile(DA, 'matlab_damper.csv'), 'w');
fl = fopen(fullfile(DA, 'matlab_loads.csv'), 'w');
fprintf(fd, '# matlab: damper length (mm) at wheel-centre travel (mm), Simscape Multibody %s.\ncorner,travel_mm,damper_mm\n', version);
fprintf(fl, '# matlab: axial force, N, + tension; direct_lower: UF, UR, TR and the damper PR.\ncorner,case,%s\n', strjoin(ALL6, ','));
for k = 1:numel(daCorners)
    w = jsondecode(fileread(fullfile(DA, 'corners', [daCorners{k} '.json'])));
    P = points(w);
    onLca = strcmp(w.pushrod_attach, 'lower');
    [trav, L] = damperCurve(P, onLca);
    for i = 1:numel(trav)
        fprintf(fd, '%s,%g,%.9f\n', daCorners{k}, trav(i), L(i));
    end
    for c = 1:numel(caseNames)
        f = memberForces(P, cases.(caseNames{c}), true, onLca, sgn);
        fprintf(fl, '%s,%s,%s\n', daCorners{k}, caseNames{c}, rowOf(f, ALL6));
    end
    fprintf('direct-acting: %s done\n', daCorners{k});
end
fclose(fd); fclose(fl);
fprintf('Wrote matlab_results.csv, matlab_damper.csv, matlab_loads.csv\n');
end

% ======================================================================================================
function P = points(s)
P = struct();
f = fieldnames(s);
for i = 1:numel(f)
    v = s.(f{i});
    if isnumeric(v) && numel(v) == 3
        P.(f{i}) = v(:)';
    end
end
end

function s = rowOf(f, names)
c = cell(1, numel(names));
for i = 1:numel(names)
    if isfield(f, names{i}), c{i} = sprintf('%.6f', f.(names{i})); else, c{i} = ''; end
end
s = strjoin(c, ',');
end

% ------------------------------------------------------------------------------------------- forces
function f = memberForces(P, cs, direct, onLca, sgn)
ENDS = {'UF','upper_outer','upper_front_inner'; 'UR','upper_outer','upper_rear_inner';
        'LF','lower_outer','lower_front_inner'; 'LR','lower_outer','lower_rear_inner';
        'TR','tie_rod_outer','tie_rod_inner'};
if direct, prIn = P.spring_inner; else, prIn = P.rocker_pushrod; end
[mdl, W] = newModel();
up = freeBody(mdl, W, 'upright');
links = {};
for i = 1:size(ENDS, 1)
    if onLca && any(strcmp(ENDS{i,1}, {'LF','LR'})), continue; end
    links(end+1, :) = {ENDS{i,1}, up, P.(ENDS{i,2}), P.(ENDS{i,3}), [0 0 0]}; %#ok<AGROW>
end
if onLca
    [lca, piv, R] = revoluteBody(mdl, W, 'lca', P.lower_front_inner, P.lower_rear_inner - P.lower_front_inner);
    sph(mdl, 'lbj', at(mdl, lca, 'lca_lo', R' * (P.lower_outer - piv)'), at(mdl, up, 'up_lo', P.lower_outer));
    links(end+1, :) = {'PR', lca, P.pushrod_outer, prIn, {piv, R}};
else
    links(end+1, :) = {'PR', up, P.pushrod_outer, prIn, [0 0 0]};
end
names = cell(1, size(links, 1));
for i = 1:size(links, 1)
    [nm, body, po, pin, ref] = links{i, :};
    if iscell(ref), local = (ref{2}' * (po - ref{1})')'; else, local = po; end
    a = at(mdl, body, ['o_' nm], local);
    b = at(mdl, W, ['i_' nm], pin);
    distance(mdl, ['D_' nm], a, b, norm(pin - po), ['T_' nm]);
    names{i} = nm;
end
extForce(mdl, at(mdl, up, 'cp', P.contact_patch), 'patch', [cs.Fx cs.Fy cs.Fz], [0 0 cs.Mz / 1000]);
extForce(mdl, at(mdl, up, 'wc', P.wheel_center), 'shaft', [cs.Fx_wc 0 0], []);
out = sim(mdl, 'StopTime', '0.01', 'ReturnWorkspaceOutputs', 'on');
f = struct();
for i = 1:numel(names)
    v = out.get(['T_' names{i}]);
    f.(names{i}) = sgn * v(end);
end
close_system(mdl, 0);
end

function sgn = calibrate()
% A body held by six orthogonal distance constraints, 100 N down where the vertical one holds it: that one
% carries 100 N tension, the others nothing. Fixes the sign of the constraint's signed-force output.
[mdl, W] = newModel();
b = freeBody(mdl, W, 'body');
spec = {[0 0 0],[0 0 100]; [0 0 0],[100 0 0]; [0 0 0],[0 100 0]; [50 0 0],[50 0 100]; [0 50 0],[0 50 100]; [0 50 0],[100 50 0]};
for i = 1:6
    distance(mdl, sprintf('D%d', i), at(mdl, b, sprintf('o%d', i), spec{i,1}), at(mdl, W, sprintf('i%d', i), spec{i,2}), ...
             norm(spec{i,2} - spec{i,1}), sprintf('T%d', i));
end
extForce(mdl, at(mdl, b, 'RT_load', [0 0 0]), 'load', [0 0 -100], []);
out = sim(mdl, 'StopTime', '0.01', 'ReturnWorkspaceOutputs', 'on');
t = zeros(1, 6);
for i = 1:6, v = out.get(sprintf('T%d', i)); t(i) = v(end); end
close_system(mdl, 0);
if abs(abs(t(1)) - 100) > 1e-6 || max(abs(t(2:6))) > 1e-6
    error('Calibration failed: signed forces %s', mat2str(t, 6));
end
sgn = sign(t(1));     % + tension = sgn * signed force
end

% ------------------------------------------------------------------------------------------- damper
function [trav, L] = damperCurve(P, onLca)
[mdl, W] = newModel();
up = freeBody(mdl, W, 'upright');
[lca, pl, Rl] = revoluteBody(mdl, W, 'lca', P.lower_front_inner, P.lower_rear_inner - P.lower_front_inner);
[uca, pu, Ru] = revoluteBody(mdl, W, 'uca', P.upper_front_inner, P.upper_rear_inner - P.upper_front_inner);
sph(mdl, 'lbj', at(mdl, lca, 'lca_lo', Rl' * (P.lower_outer - pl)'), at(mdl, up, 'up_lo', P.lower_outer));
sph(mdl, 'ubj', at(mdl, uca, 'uca_uo', Ru' * (P.upper_outer - pu)'), at(mdl, up, 'up_uo', P.upper_outer));
distance(mdl, 'D_TR', at(mdl, up, 'o_TR', P.tie_rod_outer), at(mdl, W, 'i_TR', P.tie_rod_inner), ...
         norm(P.tie_rod_inner - P.tie_rod_outer), '');
if onLca
    dout = at(mdl, lca, 'dmp_o', Rl' * (P.pushrod_outer - pl)');
else
    dout = at(mdl, up, 'dmp_o', P.pushrod_outer);
end
fDo = refFrame(mdl, 'FR_dmp_out', dout);
fDi = refFrame(mdl, 'FR_dmp_in', at(mdl, W, 'dmp_i', P.spring_inner));
fWc = refFrame(mdl, 'FR_wc', at(mdl, up, 'wc', P.wheel_center));
fW0 = refFrame(mdl, 'FR_wc0', at(mdl, W, 'wc0', P.wheel_center));
save_system(mdl, fullfile(tempdir, [mdl '.slx']));

ks = simscape.multibody.KinematicsSolver(mdl);
jp = jointPositionVariables(ks);
tid = jp.ID(contains(jp.BlockPath, 'lca_rev'));
if numel(tid) ~= 1, error('Could not find the lower wishbone joint variable:\n%s', evalc('disp(jp)')); end
addTargetVariables(ks, tid);
addFrameVariables(ks, 'dmp', 'Translation', fDi, fDo);
addFrameVariables(ks, 'wc', 'Translation', fW0, fWc);
fv = frameVariables(ks);
addOutputVariables(ks, fv.ID);
ov = outputVariables(ks);
toMm = cellfun(@unitToMm, cellstr(string(ov.Unit)));
tu = string(targetVariables(ks).Unit);
ang = linspace(-7, 7, 281);
if strcmp(tu, "rad"), ang = deg2rad(ang); end
dz = zeros(size(ang)); Ld = dz;
ids = string(ov.ID);
need = ["dmp.Translation.x","dmp.Translation.y","dmp.Translation.z","wc.Translation.z"];
if ~all(ismember(need, ids)), error('Unexpected output variable IDs: %s', strjoin(ids, ', ')); end
for i = 1:numel(ang)
    [o, flag] = solve(ks, ang(i));
    if flag ~= 1, error('Kinematics Solver did not converge at %g', ang(i)); end
    o = o(:)' .* toMm;
    v = [o(ids == "dmp.Translation.x"), o(ids == "dmp.Translation.y"), o(ids == "dmp.Translation.z")];
    Ld(i) = norm(v);
    dz(i) = o(ids == "wc.Translation.z");
end
close_system(mdl, 0);
[dz, s] = sort(dz); Ld = Ld(s);
if dz(1) > -25 || dz(end) < 25, error('Sweep does not cover -25..+25 mm of wheel travel.'); end
trav = -25:25;
L = interp1(dz, Ld, trav);
end

function k = unitToMm(u)
switch u
    case 'mm', k = 1;
    case 'cm', k = 10;
    case 'm',  k = 1000;
    otherwise, error('Unexpected length unit %s', u);
end
end

% ------------------------------------------------------------------------------------------- model building
function [mdl, W] = newModel()
mdl = sprintf('kk_bench_%09d', randi(999999999));
while bdIsLoaded(mdl), mdl = sprintf('kk_bench_%09d', randi(999999999)); end
new_system(mdl);
try, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off'); catch, end
set_param(mdl, 'Solver', 'daessc', 'StopTime', '0.01');
W = blk(mdl, 'sm_lib/Frames and Transforms/World Frame', 'W');
mc = blk(mdl, 'sm_lib/Utilities/Mechanism Configuration', 'MC');
setEnum(mc, 'Gravity', 'none');
sc = blk(mdl, 'nesl_utility/Solver Configuration', 'SC');
link(mdl, framePort(W), framePort(mc));
link(mdl, framePort(W), framePort(sc));
end

function h = blk(mdl, lib, name)
% Add a library block; if the name is taken, add a numeric suffix so two helpers can never collide.
path = [mdl '/' name];
k = 1;
while getSimulinkBlockHandle(path) ~= -1
    k = k + 1;
    path = sprintf('%s/%s_%d', mdl, name, k);
end
h = add_block(lib, path);
end

function p = framePort(h)
ph = get_param(h, 'PortHandles');
c = [ph.LConn ph.RConn];
p = c(1);
end

function link(mdl, a, b)
add_line(mdl, a, b, 'autorouting', 'on');
end

function F = freeBody(mdl, W, name)
% A body whose reference frame starts at the world origin, free in all six directions.
j = blk(mdl, 'sm_lib/Joints/6-DOF Joint', [name '_6dof']);
ph = get_param(j, 'PortHandles');
link(mdl, framePort(W), ph.LConn(1));
F = ph.RConn(1);
inertia(mdl, [name '_m'], F);
end

function [F, piv, R] = revoluteBody(mdl, W, name, piv, axis)
% A body on a revolute pivot through piv about axis. Its frame is at piv with z along the axis, so a point
% p on the body sits at R' * (p - piv) in that frame.
z = axis(:) / norm(axis);
if abs(z(3)) < 0.9, x = cross([0;0;1], z); else, x = cross([1;0;0], z); end
x = x / norm(x); y = cross(z, x);
R = [x y z];
b = at(mdl, framePort(W), [name '_pivB'], piv, R);
j = blk(mdl, 'sm_lib/Joints/Revolute Joint', [name '_rev']);
ph = get_param(j, 'PortHandles');
link(mdl, b, ph.LConn(1));
F = ph.RConn(1);
inertia(mdl, [name '_m'], F);
end

function inertia(mdl, name, F)
h = blk(mdl, 'sm_lib/Body Elements/Inertia', name);
setNum(h, '^Mass$', '1', 'kg');
setNum(h, 'MomentsOfInertia', '[1 1 1]', 'kg*m^2');
link(mdl, F, framePort(h));
end

function F = at(mdl, base, name, p, R)
% A Rigid Transform from base: translation p (mm, in base's frame), optional rotation matrix R.
if strcmp(get_param(base, 'Type'), 'block'), base = framePort(base); end   % World Frame block or a frame port
h = blk(mdl, 'sm_lib/Frames and Transforms/Rigid Transform', name);
setEnum(h, 'TranslationMethod', 'cartesian');
setNum(h, 'TranslationCartesianOffset', mat2str(p(:)', 17), 'mm');
if nargin > 4
    setEnum(h, 'RotationMethod', 'matrix');
    setNum(h, 'RotationMatrix', mat2str(R, 17), '');
end
ph = get_param(h, 'PortHandles');
link(mdl, base, ph.LConn(1));
F = ph.RConn(1);
end

function sph(mdl, name, a, b)
h = blk(mdl, 'sm_lib/Joints/Spherical Joint', name);
ph = get_param(h, 'PortHandles');
link(mdl, a, ph.LConn(1));
link(mdl, b, ph.RConn(1));
end

function path = refFrame(mdl, name, F)
% A named frame for the Kinematics Solver: a Reference Frame block, or if this release has none, the Rigid
% Transform's own follower port.
try
    h = blk(mdl, 'sm_lib/Frames and Transforms/Reference Frame', name);
    link(mdl, F, framePort(h));
    path = getfullname(h);
catch
    path = [get_param(F, 'Parent') '/F'];
end
end

function distance(mdl, name, a, b, d, outVar)
h = blk(mdl, 'sm_lib/Constraints/Distance Constraint', name);
setNum(h, '^Distance$', sprintf('%.12g', d), 'mm');
ph0 = get_param(h, 'PortHandles');
if ~isempty(outVar)
    setFlag(h, 'Sens.*Force|Force.*Sens|SignedForce');
end
ph = get_param(h, 'PortHandles');
link(mdl, a, ph0.LConn(1));
link(mdl, b, ph0.RConn(1));
if ~isempty(outVar)
    newp = setdiff([ph.LConn ph.RConn], [ph0.LConn ph0.RConn]);
    if numel(newp) ~= 1, error('Distance Constraint: expected one sensing port, found %d.', numel(newp)); end
    toWorkspace(mdl, [name '_out'], newp, 'N', outVar);
end
end

function extForce(mdl, F, name, force, torque)
h = blk(mdl, 'sm_lib/Forces and Torques/External Force and Torque', ['F_' name]);
ph0 = get_param(h, 'PortHandles');
link(mdl, F, framePort(h));
setFlag(h, '^EnableForce$');            % R2025b: one 3-vector force input
setEnum(h, '^ForceResolutionFrame$', 'world');
ph1 = get_param(h, 'PortHandles');
pf = setdiff([ph1.LConn ph1.RConn], [ph0.LConn ph0.RConn]);
fromConst(mdl, [name '_F'], force, 'N', pf);
if ~isempty(torque)
    setFlag(h, '^EnableTorque$');
    setEnum(h, '^TorqueResolutionFrame$', 'world');
    ph2 = get_param(h, 'PortHandles');
    pt = setdiff([ph2.LConn ph2.RConn], [ph1.LConn ph1.RConn]);
    fromConst(mdl, [name '_T'], torque, 'N*m', pt);
end
end

function fromConst(mdl, name, value, unit, psPort)
c = blk(mdl, 'simulink/Sources/Constant', [name '_c']);
set_param(c, 'Value', mat2str(value, 17));
s = blk(mdl, 'nesl_utility/Simulink-PS Converter', [name '_s2ps']);
setNum(s, '^Unit$', unit, '');
pc = get_param(c, 'PortHandles'); ps = get_param(s, 'PortHandles');
add_line(mdl, pc.Outport(1), ps.Inport(1), 'autorouting', 'on');
link(mdl, ps.RConn(1), psPort);
end

function toWorkspace(mdl, name, psPort, unit, var)
s = blk(mdl, 'nesl_utility/PS-Simulink Converter', [name '_ps2s']);
setNum(s, '^Unit$', unit, '');
t = blk(mdl, 'simulink/Sinks/To Workspace', [name '_tw']);
set_param(t, 'VariableName', var, 'SaveFormat', 'Array');
ps = get_param(s, 'PortHandles'); pt = get_param(t, 'PortHandles');
link(mdl, psPort, ps.LConn(1));
add_line(mdl, ps.Outport(1), pt.Inport(1), 'autorouting', 'on');
end

% ------------------------------------------------------------------------------------------- parameters by name
function n = findParam(h, pattern)
dp = fieldnames(get_param(h, 'DialogParameters'));
hit = dp(~cellfun(@isempty, regexpi(dp, pattern, 'once')));
if isempty(regexpi(pattern, 'unit', 'once'))      % skip the companion ...Units parameter
    hit = hit(cellfun(@isempty, regexpi(hit, 'Units?$', 'once')));
end
if isempty(hit)
    error('%s: no parameter matching "%s". Parameters in this release:\n%s', ...
          get_param(h, 'Name'), pattern, strjoin(dp, ', '));
end
n = hit{1};
end

function setNum(h, pattern, value, unit)
n = findParam(h, pattern);
set_param(h, n, value);
if ~isempty(unit)
    dp = fieldnames(get_param(h, 'DialogParameters'));
    if any(strcmp(dp, [n 'Units'])), set_param(h, [n 'Units'], unit);
    elseif any(strcmp(dp, [n 'Unit'])), set_param(h, [n 'Unit'], unit);
    end
end
end

function setFlag(h, pattern)
dp = get_param(h, 'DialogParameters');
names = fieldnames(dp);
hit = names(~cellfun(@isempty, regexpi(names, pattern, 'once')));
for i = 1:numel(hit)
    e = dp.(hit{i});
    if isfield(e, 'Type') && strcmpi(e.Type, 'boolean'), set_param(h, hit{i}, 'on'); return; end
    if isfield(e, 'Enum') && any(strcmpi(e.Enum, 'on')), set_param(h, hit{i}, 'on'); return; end
end
error('%s: no on/off parameter matching "%s". Parameters in this release:\n%s', ...
      get_param(h, 'Name'), pattern, strjoin(names, ', '));
end

function setEnum(h, pattern, valuePattern)
dp = get_param(h, 'DialogParameters');
names = fieldnames(dp);
hit = names(~cellfun(@isempty, regexpi(names, pattern, 'once')));
for i = 1:numel(hit)
    e = dp.(hit{i});
    if isfield(e, 'Enum')
        v = e.Enum(~cellfun(@isempty, regexpi(e.Enum, valuePattern, 'once')));
        if ~isempty(v), set_param(h, hit{i}, v{1}); return; end
    end
end
error('%s: no option matching "%s" in a parameter matching "%s". Parameters in this release:\n%s', ...
      get_param(h, 'Name'), valuePattern, pattern, strjoin(names, ', '));
end
