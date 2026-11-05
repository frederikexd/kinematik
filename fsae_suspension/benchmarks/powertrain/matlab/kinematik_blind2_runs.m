% KINEMATIK_BLIND2_RUNS  Round 2 of the pre-registered Simulink runs (benchmarks/powertrain/simulink_blind2/).
%
% Run it as a script, with the Formula Student project open (openProject SSVT_FSAE.prj) and this folder's
% simulink_lap.m and export_achilles.m next to it. Two modes, the same cases and settings:
%
%   mode = 'check'; kinematik_blind2_runs     % BEFORE the seal: every case runs to its end, and each setting takes
%                                             % effect. Prints only that (mass, alignment, target speed, finished or
%                                             % not); writes no results. Send the printout before anything is sealed.
%   mode = 'run';   kinematik_blind2_runs     % AFTER the seal is pushed: the runs, one folder per case in
%                                             % kinematik_blind2/ (skips cases already there)
%
% Round 1 failed (docs/POWERTRAIN_BENCHMARK_PLAN.md); what changed: the Formula Student-sized car keeps its ride
% height and alignment (spring preload scaled with the sprung weight), its pack has higher-power cells, the speed is
% varied through the trajectory itself (Maneuver.vGain is not read by the driver), the launches use a pedal the
% model can simulate, and every case is checked to run before the seal.

cases = {
%   name                     vehicle data               event                fs_car  speed
    'hock_fs2',              'Vehicle_data_dwishbone',  'CRG Hockenheim F',  true,   1.00
    'hock_fs2_fast',         'Vehicle_data_dwishbone',  'CRG Hockenheim F',  true,   1.15
    'hock_fs2_pushrod_fast', 'Vehicle_data_dwpushrod',  'CRG Hockenheim F',  true,   1.15
    'hock_stock_fast',       'Vehicle_data_dwishbone',  'CRG Hockenheim F',  false,  1.15
    'skidpad_fs2',           'Vehicle_data_dwishbone',  'Skidpad',           true,   1.00
    'launch_fs2',            'Vehicle_data_dwishbone',  'WOT Braking',       true,   NaN
    'launch_stock2',         'Vehicle_data_dwishbone',  'WOT Braking',       false,  NaN
};
kb_pedal = 0.10;          % launches: each motor is asked for 400 N m x pedal = 40 N m, which the model can simulate
if ~exist('mode', 'var'), mode = 'run'; end
if ~exist('only', 'var'), only = cases(:, 1)'; end
root = 'kinematik_blind2';
if ~exist(root, 'dir'), mkdir(root); end
for c = 1:size(cases, 1)
    [kb_name, kb_data, kb_event, kb_fs, kb_speed] = cases{c, :};
    kb_dir = fullfile(root, kb_name);
    if ~any(strcmp(only, kb_name)) || (strcmp(mode, 'run') && exist(fullfile(kb_dir, 'simulink_lap.csv'), 'file'))
        continue
    end
    fprintf('\n==== %s (%s): %s, %s ====\n', kb_name, mode, kb_data, kb_event);
    clear logsout_sm_car out
    eval(kb_data);                                   % as the model's hyperlink does
    sm_car_config_maneuver('sm_car', kb_event);
    if kb_fs
        % A Formula Student-sized car: 114.8 kg of sprung mass off MathWorks' 405 kg car (body 165 -> 95 kg, each
        % anti-roll bar 20 -> 5 kg, steering rack 19.8 -> 5 kg); 0.25 kg m^2 per wheel (it has 2); the spring preload
        % scaled with the sprung weight, so the car sits at the same ride height and alignment (round 1 did not, and
        % its rear toe went from -0.2 to +2.1 deg); a 25 Ah pack of higher-power cells (3 times the resistance).
        % The wheels keep their shipped spin inertia (about 2.4 kg m^2 each): TireA*.Inertia, which round 1 set to
        % 0.25, does not reach Simscape's wheel; simulink_lap.m measures the inertia from the run instead.
        kb_sprung_old = 405.1 - 92.9;                % 92.9 kg unsprung: 4 x (tire 10, upright 1.05, half the three
        kb_sprung_new = kb_sprung_old - 114.8;       %   links 8.18, half the shock 3.98)
        Vehicle.Chassis.Body.m.Value = 95;
        Vehicle.Chassis.Body.J.Value = Vehicle.Chassis.Body.J.Value * 95 / 165;
        Vehicle.Chassis.SuspA1.AntiRollBar.m.Value = 5;
        Vehicle.Chassis.SuspA2.AntiRollBar.m.Value = 5;
        Vehicle.Chassis.SuspA1.Steer.Rack.m.Value = 5;
        Vehicle.Chassis.Spring.Axle1.xPreload.Value = Vehicle.Chassis.Spring.Axle1.xPreload.Value * kb_sprung_new / kb_sprung_old;
        Vehicle.Chassis.Spring.Axle2.xPreload.Value = Vehicle.Chassis.Spring.Axle2.xPreload.Value * kb_sprung_new / kb_sprung_old;
        Vehicle.Powertrain.Power.Battery.AhTable.Ah.Value = Vehicle.Powertrain.Power.Battery.AhTable.Ah.Value * 25 / 230;
        Vehicle.Powertrain.Power.Battery.rTerminal.R0.Value = Vehicle.Powertrain.Power.Battery.rTerminal.R0.Value * 3;
    end
    if ~isnan(kb_speed) && kb_speed ~= 1
        % faster: the target speed along the whole trajectory, and the start speed with it
        Maneuver.Trajectory.vx.Value = Maneuver.Trajectory.vx.Value * kb_speed;
        Init.Chassis.vChassis.Value = Init.Chassis.vChassis.Value * kb_speed;
        Init.Axle1.nWheel.Value(1:2) = Init.Axle1.nWheel.Value(1:2) * kb_speed;
        Init.Axle2.nWheel.Value(1:2) = Init.Axle2.nWheel.Value(1:2) * kb_speed;
    end
    if strcmp(kb_event, 'WOT Braking')
        % from standstill, pedal at 0.2 s, off at 10 s, brakes at 11 s; 12 s is past 75 m
        Init.Chassis.vChassis.Value = [0.05 0 0];
        Init.Axle1.nWheel.Value(1:2) = 0.05 / Init.Axle1.nWheel.Value(3);
        Init.Axle2.nWheel.Value(1:2) = 0.05 / Init.Axle2.nWheel.Value(3);
        Maneuver.Accel.t.Value = [0 0.2 0.3 10 10.2 200];
        Maneuver.Accel.rPedal.Value = [0 0 kb_pedal kb_pedal 0 0];
        set_param('sm_car', 'StopTime', '12');
    end
    kb_t0 = tic;
    kb_ok = true;
    try
        out = sim('sm_car');
    catch kb_err
        kb_ok = false;
        fprintf('FAILED: %s\n', kb_err.message);
    end
    if kb_ok && ~exist('logsout_sm_car', 'var') && isa(out, 'Simulink.SimulationOutput') && ...
            any(strcmp(out.who, 'logsout_sm_car'))
        logsout_sm_car = out.logsout_sm_car;
    end
    if strcmp(mode, 'check')
        if kb_ok
            vb = logsout_sm_car.get('VehBus').Values;
            t = vb.Chassis.Body.CG.vx.Time;
            fz = 0;
            for w = {'WhlL1', 'WhlR1', 'WhlL2', 'WhlR2'}, fz = fz + abs(squeeze(vb.Chassis.(w{1}).Fz.Data)); end
            v = squeeze(vb.Chassis.Body.CG.vx.Data);
            A = Vehicle.Chassis.Aero;
            mass = median(fz - 0.5 * A.rho_air.Value * (-A.CL.Value) * A.ARef.Value * v .^ 2) / 9.81;
            gy = abs(squeeze(vb.Chassis.Body.CG.gy.Data));
            s = gy < 0.5 & t > 3;
            if nnz(s) < 10, s = t > 1; end
            toe = [median(squeeze(vb.Chassis.SuspA1.WhlL.aToe.Data(s))), median(squeeze(vb.Chassis.SuspA1.WhlR.aToe.Data(s))), ...
                   median(squeeze(vb.Chassis.SuspA2.WhlL.aToe.Data(s))), median(squeeze(vb.Chassis.SuspA2.WhlR.aToe.Data(s)))];
            fprintf('finished: yes, to t = %.1f s (%.0f s wall)\n', t(end), toc(kb_t0));
            fprintf('mass from the tire loads: %.1f kg; toe on straights (deg): %s; CG height %.4f m\n', mass, ...
                mat2str(toe, 3), median(squeeze(vb.Chassis.Body.CG.z.Data)));
            if ~strcmp(kb_event, 'WOT Braking')
                fprintf('target speed at the start: %.2f m/s (the shipped trajectory starts at %.2f)\n', ...
                    Maneuver.Trajectory.vx.Value(1), Maneuver.Trajectory.vx.Value(1) / kb_speed);
                try
                    vt = logsout_sm_car.get('DrvBus').Values.Reference.vTarget;
                    fprintf('driver target over the run: %.2f to %.2f m/s\n', min(vt.Data), max(vt.Data));
                catch
                end
            end
        end
        continue
    end
    if ~kb_ok
        continue                                     % reported as not run
    end
    simulink_lap([], struct(), kb_dir);
    fid = fopen(fullfile(kb_dir, 'run_info.txt'), 'w');
    fprintf(fid, 'case %s\nvehicle data %s\nevent %s\nfs_car %d\nspeed %g\npedal %g\nrun %s\nwall time %.0f s\nMATLAB %s\n', ...
        kb_name, kb_data, kb_event, kb_fs, kb_speed, kb_pedal, ...
        char(datetime('now', 'TimeZone', 'UTC'), 'yyyy-MM-dd HH:mm:ss''Z'''), toc(kb_t0), version);
    fclose(fid);
end
fprintf('\nDone (%s).\n', mode);
