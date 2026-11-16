function kk_knc_simscape(kkroot, svtroot)
% KinematiK corners on MathWorks' quarter-car K&C test rig (Simscape Vehicle Templates), with MathWorks' own metric
% definitions (sm_car_knc_calc_susp_metrics): caster and KPI from camber change while steering, trail and scrub from the
% steering axis found by the wheel centre's path, bump steer and camber, damper ratio, roll steer and camber.
%
%   kk_knc_simscape(kkroot, svtroot)
%     kkroot   folder holding manifests/ (and where benchmarks/knc_simscape/ is written)
%     svtroot  folder of the Simscape Vehicle Templates clone (contains sm_car_proj.prj)
%
% Writes benchmarks/knc_simscape/<corner>_metrics.csv, _curves.csv, _reference.csv and _timeseries.csv; compare with
% scripts/simscape_knc_benchmark.py. Shares no code with KinematiK: it reads only the recorded hardpoints.

corners = {'front_v11_final', 'rear_v11_final'};
outdir = fullfile(kkroot, 'benchmarks', 'knc_simscape');
if ~exist(outdir, 'dir'), mkdir(outdir); end

proj = matlab.project.rootProject;
if isempty(proj) || ~strcmp(proj.Name, 'Simscape Vehicle Templates') && ~contains(proj.RootFolder, 'Simscape-Vehicle-Templates')
    fprintf('Opening Simscape Vehicle Templates (the first time takes about 10 minutes)...\n');
    openProject(fullfile(svtroot, 'sm_car_proj.prj'));
end

for c = 1:numel(corners)
    name = corners{c};
    fprintf('\n=== %s ===\n', name);
    man = jsondecode(fileread(fullfile(kkroot, 'manifests', [name '.recorded.genesis.json'])));
    w = man.recorded.winner_hardpoints;

    % Load the rig FIRST: its PostLoadFcn runs sm_car_load_vehicle_data('none','213') and resets Maneuver, which
    % silently replaced the hardpoints whenever the model loaded after they were set. Loaded here, it is overwritten below.
    mdl = 'testrig_quarter_car_doublewishbone';
    load_system(mdl);
    try
        set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');      % no Mechanics Explorer window per run
    catch
    end

    sm_car_load_vehicle_data('none', '213');
    Vehicle = evalin('base', 'Vehicle');
    % The rig's metrics take the design position at a wheel-centre height equal to the dataset's tyre radius. Translate
    % the whole corner vertically so the wheel centre sits there: a rigid shift, the kinematics are unchanged.
    [tire_radius, ~] = sm_car_get_TireRadius(Vehicle);
    dz = 1000 * tire_radius(1) - w.wheel_center(3);
    fprintf('corner raised %.3f mm so its wheel centre sits at the rig tyre radius (%.2f mm)\n', dz, 1000 * tire_radius(1));
    % KinematiK right corner (mm, x rearward, y outboard, z up) -> Templates left corner (m, x forward, y outboard)
    T = @(p) [-p(1), p(2), p(3) + dz] / 1000;
    Vehicle = sm_car_vehcfg_setPeopleOnOff(Vehicle, [0 0 0 0 0], 'SuspA1');
    L = Vehicle.Chassis.SuspA1.Linkage;
    L.UpperWishbone.sInboardF.Value = T(w.upper_front_inner);
    L.UpperWishbone.sInboardR.Value = T(w.upper_rear_inner);
    L.UpperWishbone.sOutboard.Value = T(w.upper_outer);
    L.LowerWishbone.sInboardF.Value = T(w.lower_front_inner);
    L.LowerWishbone.sInboardR.Value = T(w.lower_rear_inner);
    L.LowerWishbone.sOutboard.Value = T(w.lower_outer);
    mid = (w.lower_front_inner(:)' + w.lower_rear_inner(:)') / 2;
    L.LowerWishbone.sARB.Value = T(mid + 0.5 * (w.lower_outer(:)' - mid));
    L.Upright.sWheelCentre.Value = T(w.wheel_center);
    L.Upright.sCG.Value = T(w.wheel_center(:)' - [0 30 0]);
    L.Upright.aCamber.Value = w.static_camber;          % Templates: + top out, as KinematiK
    L.Upright.aToe.Value = -w.static_toe;               % Templates: + toe in; KinematiK: + toe out
    L.TrackRod.sInboard.Value = T(w.tie_rod_inner);
    L.TrackRod.sOutboard.Value = T(w.tie_rod_outer);
    % Damper straight onto the lower wishbone (the rig's layout): 70 % out from the inboard axis, 15 mm above the
    % arm, to a chassis point 60 mm inboard and 300 mm up (306 mm long: the Templates' damper body needs its length to
    % exceed 3 caps, piston, bump stop and twice the rebound travel plus the bump travel). simscape_knc_benchmark.py builds the same direct-acting
    % corner in KinematiK for the damper ratio.
    bot = mid + 0.7 * (w.lower_outer(:)' - mid) + [0 0 15];
    top = bot + [0 -60 300];
    L.Shock.sBottom.Value = T(bot);
    L.Shock.sTop.Value = T(top);
    L.Endstop.xMax.Value = 0.05;            % +-50 mm of damper travel; the test needs about +-30
    L.Endstop.xMin.Value = -0.05;
    Vehicle.Chassis.SuspA1.Linkage = L;
    Vehicle.Chassis.Spring.Axle1.sTop.Value = T(top);
    Vehicle.Chassis.Spring.Axle1.sBottom.Value = T(bot);
    Vehicle.Chassis.Damper.Axle1.Damping.sTop.Value = T(top);
    Vehicle.Chassis.Damper.Axle1.Damping.sBottom.Value = T(bot);
    Vehicle.Chassis.Damper.Axle1.Endstop.xMax.Value = 0.05;
    Vehicle.Chassis.Damper.Axle1.Endstop.xMin.Value = -0.05;
    assignin('base', 'Vehicle', Vehicle);

    % FSAE-sized test: posts +-40 mm, bump test +-10 mm, steering wheel +-0.95 rad, roll +-30 mm, the Templates'
    % FSAE forces for the compliance phases (rigid joints, so they do not move the kinematics).
    Maneuver = sm_car_maneuverdata_knc(0.04, -0.04, 0.01, 0.95, 0.03, 1000, 2400, 2400, 2400, 2400, -0.3);
    assignin('base', 'Maneuver', Maneuver);

    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');                  % every logged variable inside 'out'
    logs = findLogs(out);
    if isempty(logs)
        error('kk_knc_simscape:noLogs', ['The run finished but no logged dataset with VehBus was found. ' ...
              'Variables in the simulation output: %s'], strjoin(out.who, ', '));
    end
    % The whole run, raw: wheel-centre position, toe, camber, rack, post force. Lets the comparison take the rig's own
    % reference height from this log, and shows whether the corner assembled as designed.
    vb = logs.get('VehBus').Values;
    t = vb.Chassis.WhlL1.xyz.Time;
    xyz = vb.Chassis.WhlL1.xyz.Data;
    toeT = squeeze(vb.Chassis.SuspA1.WhlL.aToe.Data);
    camT = squeeze(vb.Chassis.SuspA1.WhlL.aCamber.Data);
    rackT = squeeze(vb.Chassis.SuspA1.Steer.xRack.Data);
    if numel(rackT) ~= numel(t), rackT = nan(size(t)); end
    ts = table(t(:), 1000 * xyz(:, 1), 1000 * xyz(:, 2), 1000 * xyz(:, 3), toeT(:), camT(:), 1000 * rackT(:), ...
               'VariableNames', {'t_s', 'px_mm', 'py_mm', 'pz_mm', 'toe_in_deg', 'camber_deg', 'rack_mm'});
    % the wheel-frame angles MathWorks' KPI estimate uses (sm_car_knc_calc_susp_metrics), so it can be reproduced too
    try
        toeX = squeeze(vb.Chassis.SuspA1.WhlL.aToeX.Data);
        camX = squeeze(vb.Chassis.SuspA1.WhlL.aCamberX.Data);
        if numel(toeX) == numel(t) && numel(camX) == numel(t)
            ts.toeX_deg = toeX(:);
            ts.camberX_deg = camX(:);
        end
    catch
    end
    writetable(ts, fullfile(outdir, [name '_timeseries.csv']));
    ref = table(1000 * tire_radius(1), 1000 * xyz(1, 1), 1000 * xyz(1, 2), 1000 * xyz(1, 3), camT(1), toeT(1), ...
                1000 * Maneuver.zOffsetBumpTest.Value, Maneuver.qToeForCaster.Value, ...
                'VariableNames', {'tire_radius_mm', 'wc0_x_mm', 'wc0_y_mm', 'wc0_z_mm', 'camber0_deg', 'toe0_deg', ...
                                  'z_bump_mm', 'q_toe_caster_deg'});
    writetable(ref, fullfile(outdir, [name '_reference.csv']));
    % Assembly check: at t = 0 the corner must sit at its design camber and toe.
    wcT = T(w.wheel_center);
    if abs(camT(1) - w.static_camber) > 0.05 || abs(toeT(1) + w.static_toe) > 0.05 || abs(xyz(1, 2) - wcT(2)) > 1e-4
        warning('kk_knc_simscape:assembly', ['%s did not assemble at its design position: camber %.3f deg (design ' ...
                '%.3f), toe-in %.3f deg (design %.3f). Its metrics are not usable; send %s_timeseries.csv.'], ...
                name, camT(1), w.static_camber, toeT(1), -w.static_toe, name);
    end

    [TSuspMetrics, toeCurve, camCurve] = sm_car_knc_plot1toecamber(logs, 1, false, true, false);
    disp(TSuspMetrics(:, 1:3));

    writetable(TSuspMetrics, fullfile(outdir, [name '_metrics.csv']));
    curves = table(1000 * toeCurve.pzTire(:), toeCurve.qToe(:), camCurve.qCam(:), ...
                   'VariableNames', {'pz_mm', 'toe_in_deg', 'camber_deg'});
    writetable(curves, fullfile(outdir, [name '_curves.csv']));
    fprintf('wrote %s\n', fullfile(outdir, [name '_metrics.csv']));
end
bdclose('all');
fprintf('\nDone. Zip the folder %s and send it.\n', outdir);
end


function logs = findLogs(out)
% The logged signals, whatever the model names them: the Dataset holding MathWorks' VehBus, in the simulation output
% or, as in MathWorks' own example scripts, in the base workspace.
logs = [];
names = {};
if isa(out, 'Simulink.SimulationOutput')
    names = out.who;
end
for i = 1:numel(names)
    v = out.get(names{i});
    if isa(v, 'Simulink.SimulationData.Dataset') && any(strcmp(v.getElementNames, 'VehBus'))
        logs = v;
        return
    end
end
vars = evalin('base', 'who');
for i = 1:numel(vars)
    v = evalin('base', vars{i});
    if isa(v, 'Simulink.SimulationData.Dataset') && any(strcmp(v.getElementNames, 'VehBus'))
        logs = v;
        return
    end
end
end
