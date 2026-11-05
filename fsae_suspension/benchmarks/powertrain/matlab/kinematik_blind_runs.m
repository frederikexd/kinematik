% KINEMATIK_BLIND_RUNS  The pre-registered Simulink runs (benchmarks/powertrain/simulink_blind/PREREGISTRATION.json).
%
% Run it as a script, with the Formula Student project open (openProject SSVT_FSAE.prj) and this folder on the path
% (simulink_lap.m and export_achilles.m next to it):
%
%   kinematik_blind_runs                 % every case not yet in kinematik_blind/
%   only = {'launch_fs_car'}; kinematik_blind_runs     % just those cases
%
% Each case loads the suspension's vehicle data and the event exactly as the model's own hyperlinks do, applies the
% case's changes, runs sm_car, and writes kinematik_blind/<case>/ (simulink_lap.csv, inputs.json, motor_curve.csv,
% simulink_signals.txt, run_info.txt). Nothing in KinematiK is involved: grading happens afterwards in Python, from
% the code sealed before these runs. A lap takes several minutes; all six take up to an hour.
%
% The cases (keep them as sealed; a changed case is a new, separate pre-registration):
%   hock_fs_car        Hockenheim (flat), double wishbone, a Formula Student-sized car (see fs_car below)
%   hock_fs_car_fast   the same, driven 25 % faster (Maneuver.vGain 1.25): the motors near full power
%   hock_pushrod_fast  the same with pushrod suspension: the powertrain result must not depend on it
%   skidpad_fs_car     Skidpad, Formula Student-sized car: steady cornering, where cornering drag dominates
%   launch_fs_car      full-pedal standing start, Formula Student-sized car: 75 m time and speed, predicted
%   launch_stock       the same with MathWorks' car as shipped (405 kg, 2 kg m^2 wheels)

cases = {
%   name                 vehicle data               event                fs_car  vGain
    'hock_fs_car',       'Vehicle_data_dwishbone',  'CRG Hockenheim F',  true,   1.00
    'hock_fs_car_fast',  'Vehicle_data_dwishbone',  'CRG Hockenheim F',  true,   1.25
    'hock_pushrod_fast', 'Vehicle_data_dwpushrod',  'CRG Hockenheim F',  true,   1.25
    'skidpad_fs_car',    'Vehicle_data_dwishbone',  'Skidpad',           true,   1.00
    'launch_fs_car',     'Vehicle_data_dwishbone',  'WOT Braking',       true,   NaN
    'launch_stock',      'Vehicle_data_dwishbone',  'WOT Braking',       false,  NaN
};
if ~exist('only', 'var'), only = cases(:, 1)'; end
root = 'kinematik_blind';
if ~exist(root, 'dir'), mkdir(root); end
for c = 1:size(cases, 1)
    [kb_name, kb_data, kb_event, kb_fs, kb_gain] = cases{c, :};
    kb_dir = fullfile(root, kb_name);
    if ~any(strcmp(only, kb_name)) || exist(fullfile(kb_dir, 'simulink_lap.csv'), 'file')
        continue
    end
    fprintf('\n==== %s: %s, %s ====\n', kb_name, kb_data, kb_event);
    clear logsout_sm_car out
    eval(kb_data);                                   % as the model's hyperlink does
    sm_car_config_maneuver('sm_car', kb_event);
    if kb_fs
        % A Formula Student-sized car: about 290 kg with driver (MathWorks' car is 405 kg), 0.25 kg m^2 per wheel
        % (it has 2), a 25 Ah pack (7 kWh at its 286 V; it has 230 Ah) with the same cells, so 9.2 times the resistance.
        Vehicle.Chassis.Body.m.Value = 95;
        Vehicle.Chassis.Body.J.Value = Vehicle.Chassis.Body.J.Value * 95 / 165;
        Vehicle.Chassis.SuspA1.AntiRollBar.m.Value = 5;
        Vehicle.Chassis.SuspA2.AntiRollBar.m.Value = 5;
        Vehicle.Chassis.SuspA1.Steer.Rack.m.Value = 5;
        Vehicle.Chassis.TireA1.Inertia.Value = [0.15 0.25];
        Vehicle.Chassis.TireA2.Inertia.Value = [0.15 0.25];
        Vehicle.Powertrain.Power.Battery.AhTable.Ah.Value = Vehicle.Powertrain.Power.Battery.AhTable.Ah.Value * 25 / 230;
        Vehicle.Powertrain.Power.Battery.rTerminal.R0.Value = Vehicle.Powertrain.Power.Battery.rTerminal.R0.Value * 230 / 25;
    end
    if ~isnan(kb_gain)
        Maneuver.vGain.Value = kb_gain;
    end
    if strcmp(kb_event, 'WOT Braking')
        % from standstill, full pedal at 0.2 s, off at 10 s, brakes at 11 s
        Init.Chassis.vChassis.Value = [0.05 0 0];
        Init.Axle1.nWheel.Value(1:2) = 0.05 / Init.Axle1.nWheel.Value(3);
        Init.Axle2.nWheel.Value(1:2) = 0.05 / Init.Axle2.nWheel.Value(3);
        Maneuver.Accel.t.Value = [0 0.2 0.3 10 10.2 200];
        Maneuver.Accel.rPedal.Value = [0 0 1 1 0 0];
    end
    kb_t0 = tic;
    out = sim('sm_car');
    if ~exist('logsout_sm_car', 'var') && isa(out, 'Simulink.SimulationOutput') && any(strcmp(out.who, 'logsout_sm_car'))
        logsout_sm_car = out.logsout_sm_car;
    end
    simulink_lap([], struct(), kb_dir);
    fid = fopen(fullfile(kb_dir, 'run_info.txt'), 'w');
    fprintf(fid, 'case %s\nvehicle data %s\nevent %s\nfs_car %d\nvGain %g\nrun %s\nwall time %.0f s\nMATLAB %s\n', ...
        kb_name, kb_data, kb_event, kb_fs, kb_gain, char(datetime('now', 'TimeZone', 'UTC'), 'yyyy-MM-dd HH:mm:ss''Z'''), ...
        toc(kb_t0), version);
    fclose(fid);
end
fprintf('\nDone. Zip the kinematik_blind folder and grade it: python scripts/powertrain_benchmark.py simulink-blind PATH\n');
