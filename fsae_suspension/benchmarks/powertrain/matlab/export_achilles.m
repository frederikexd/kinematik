function export_achilles(mass_kg, outdir, toe_deg, rot_measured)
% EXPORT_ACHILLES  Write KinematiK's benchmark inputs for the car loaded in the Simscape Vehicle Templates.
%
%   export_achilles(mass_kg)
%
% Reads the Vehicle structure in the base workspace (the Formula Student 'Achilles' car: four hub motors, one per
% wheel) and writes achilles/inputs.json and achilles/motor_curve.csv in the layout of
% fsae_suspension/benchmarks/powertrain/inputs.json, so KinematiK models the same car as Simscape.
%
% mass_kg is the whole car as simulated, which the Vehicle structure does not hold in one place: run the Accel/Decel
% test, open "Plot tire forces", and take (Fz_FL + Fz_FR + Fz_RL + Fz_RR) / 9.81 at the start, before the car moves.
%
% Conversions made here, so nothing is copied by hand:
%   * the four motors are summed into one equivalent motor on the common gearbox ratio, capped at the converter's
%     power (KinematiK's motor map is the whole drive), and the car is all-wheel drive;
%   * the pack is its open-circuit voltage and terminal resistance at the initial state of charge and 25 degC;
%   * tire radius, peak longitudinal friction at the static wheel load ((PDX1 + PDX2 dfz) x LMUX) and rolling
%     resistance (QSY1) come from the .tir file;
%   * Cd x ARef and -Cl x ARef give the drag and downforce areas (Cl is negative for downforce here);
%   * efficiencies given in percent are converted to fractions.
if nargin < 1 || isempty(mass_kg)
    error('export_achilles:mass', ['Give the simulated car''s mass: run Accel/Decel, open "Plot tire forces", ' ...
        'and use (Fz_FL + Fz_FR + Fz_RL + Fz_RR) / 9.81 at the start.']);
end
if nargin < 2 || isempty(outdir), outdir = 'achilles'; end
if nargin < 3, toe_deg = []; end
if nargin < 4, rot_measured = []; end
V = evalin('base', 'Vehicle');
P = V.Powertrain.Power;
D = V.Powertrain.Driveline;
C = V.Chassis;
if ~exist(outdir, 'dir'), mkdir(outdir); end

% ---- motors: four summed into one, on the common gearbox ratio ----------------------------------------------------
names = {'MotorL1', 'MotorR1', 'MotorL2', 'MotorR2'};
w = P.MotorL1.TorqueSpd.w.Value(:);
T = zeros(size(w));
for k = 1:numel(names)
    m = P.(names{k});
    T = T + interp1(m.TorqueSpd.w.Value(:), m.TorqueSpd.trq.Value(:), w, 'linear', 'extrap');
end
ratios = [D.GearboxL1.ratio.Value, D.GearboxR1.ratio.Value, D.GearboxL2.ratio.Value, D.GearboxR2.ratio.Value];
if any(abs(ratios - ratios(1)) > 1e-9)
    warning('export_achilles:ratios', 'the four gearboxes differ (%s): using the first', mat2str(ratios));
end
eta_m = P.MotorL1.LossTable.efficiency.Value(:);
eta_m = median(eta_m(eta_m > 0));
if eta_m > 1, eta_m = eta_m / 100; end
eta_c = P.Converter.efficiency.Value;
if eta_c > 1, eta_c = eta_c / 100; end
P_shaft = P.Converter.Power.Value * eta_c * eta_m;              % W at the motor shafts, all four together
omega = max(w, 1) * 2 * pi / 60;
T = min(T, P_shaft ./ omega);
fid = fopen(fullfile(outdir, 'motor_curve.csv'), 'w');
fprintf(fid, 'rpm,torque_nm\n');
fprintf(fid, '%.6g,%.6g\n', [w, T]');
fclose(fid);

% ---- battery: pack level, at the initial state of charge and 25 degC ----------------------------------------------
B = P.Battery;
soc = B.SOCInit.Value;
Tk = 298.1;
V0 = interp2(B.vNoLoad.T.Value, B.vNoLoad.SOC.Value, B.vNoLoad.V0.Value, Tk, soc);
R0 = interp2(B.rTerminal.T.Value, B.rTerminal.SOC.Value, B.rTerminal.R0.Value, Tk, soc);
Ah = interp1(B.AhTable.T.Value, B.AhTable.Ah.Value, Tk);

% ---- the battery block's own heat capacity and starting temperature (not in the Vehicle structure) --------------------
batt_c = []; batt_t0 = Tk;
try
    blk = [{'sm_car/Vehicle/Vehicle/Powertrain/Power/Electric L1 R1 L2 R2/Battery/Battery'}; ...
        find_system('sm_car', 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', ...
        'ReferenceBlock', 'batteryecm|table_battery|Battery')];
    for k = 1:numel(blk)
        try
            c = evalin('base', get_param(blk{k}, 'thermal_mass'));
            if strcmp(get_param(blk{k}, 'thermal_mass_unit'), 'kJ/K'), c = c * 1e3; end
            batt_c = c;
            batt_t0 = evalin('base', get_param(blk{k}, 'temperature'));
            break
        catch
        end
    end
catch
end
if isempty(batt_c)
    warning('export_achilles:thermal', 'battery heat capacity not found: the pack is treated as isothermal');
end

% ---- tires: from the .tir file -------------------------------------------------------------------------------------
txt = fileread(eval(C.TireA1.tirFile.Value));
num = @(key) str2double(regexp(txt, ['(?mi)^\s*' key '\s*=\s*([-+0-9.eE]+)'], 'tokens', 'once'));
radius = num('UNLOADED_RADIUS');
% peak longitudinal friction at the car's static wheel load (MF: (PDX1 + PDX2 dfz) LMUX, dfz = Fz / Fz0 - 1)
pdx2 = num('PDX2'); if isnan(pdx2), pdx2 = 0; end
lmux = num('LMUX'); if isnan(lmux), lmux = 1; end
fz0 = num('FNOMIN');
dfz = 0; if ~isnan(fz0) && fz0 > 0, dfz = mass_kg * 9.81 / 4 / fz0 - 1; end
mu = (num('PDX1') + pdx2 * dfz) * lmux;
if isnan(mu), mu = 1.4; warning('export_achilles:mu', 'PDX1 not found: friction set to 1.4'); end
crr = num('QSY1');
if isnan(crr) || crr <= 0, crr = 0.015; end

% ---- geometry and aero ---------------------------------------------------------------------------------------------
L = abs(C.Body.sAxle2.Value(1) - C.Body.sAxle1.Value(1));
a = abs(C.Body.sCG.Value(1) - C.Body.sAxle1.Value(1));          % front axle to CG
A = C.Aero.ARef.Value;
xp = abs(C.Aero.sPressureCentre.Value(1) - C.Body.sAxle1.Value(1));

% ---- motor map, tire losses and rotating inertia (KinematiK reads them when present) -------------------------------
LT = P.MotorL1.LossTable;
E = LT.efficiency.Value;
if max(E(:)) > 1, E = E / 100; end
for k = 2:numel(names)
    o = P.(names{k}).LossTable.efficiency.Value;
    if ~isequal(size(o), size(LT.efficiency.Value)) || any(abs(o(:) - LT.efficiency.Value(:)) > 1e-9)
        warning('export_achilles:maps', 'motor %s has its own loss table: using MotorL1''s for all four', names{k});
    end
end
motor_map = struct('rpm', LT.w.Value(:)', 'torque_nm', LT.trq.Value(:)', 'efficiency', E, 'n_motors', numel(names));
tire = struct('qsy1', num('QSY1'), 'qsy3', num('QSY3'), 'qsy4', num('QSY4'), 'qsy7', num('QSY7'), ...
    'fz_nom_n', num('FNOMIN'), 'v_ref_ms', num('LONGVL'), 'pky1', num('PKY1'), 'pky2', num('PKY2'), ...
    'pky4', num('PKY4'), 'n_tires', 4);
fn = fieldnames(tire);
for k = 1:numel(fn)
    if isnan(tire.(fn{k})), tire = rmfield(tire, fn{k}); end      % not in the file: KinematiK's default
end
J1 = C.TireA1.Inertia.Value; J2 = C.TireA2.Inertia.Value;       % [Ixx Iyy]: Iyy is the spin axis
Jm = 0;
for k = 1:numel(names)
    try, Jm = Jm + P.(names{k}).J.Value * ratios(k) ^ 2; catch, end
end
rot = 1 + (2 * J1(end) + 2 * J2(end) + Jm) / (radius ^ 2 * mass_kg);
if ~isempty(rot_measured), rot = rot_measured; end   % measured from the run (simulink_lap.m): TireA*.Inertia is not
                                                     % what Simscape spins

in.x_note = 'Written by export_achilles.m from the Simscape Vehicle Templates Vehicle structure.';
in.vehicle = struct('mass_kg', mass_kg, 'cg_height_m', C.Body.sCG.Value(3), 'wheelbase_m', L, ...
    'tire_radius_m', radius, 'mu', mu, 'crr', crr, 'cda_m2', C.Aero.CD.Value * A, ...
    'cla_m2', -C.Aero.CL.Value * A, 'rear_aero_frac', xp / L, 'rho_kg_m3', C.Aero.rho_air.Value, ...
    'rot_mass_factor', rot, 'tire', tire);
if numel(toe_deg) == 4, in.vehicle.toe_deg = toe_deg(:)'; end   % L1 R1 L2 R2, as the car runs
in.ev = struct('rear_static_share', a / L, 'drive', 'awd', 'pack_power_kw', P.Converter.Power.Value / 1e3, ...
    'inverter_eff', eta_c, 'motor_eff', eta_m, 'driveline_eff', 1.0, 'final_drive', ratios(1), ...
    'motor_peak_torque_nm', max(T), 'motor_max_rpm', max(w), 'motor_curve_csv', 'motor_curve.csv', ...
    'pack', struct('series', 1, 'parallel', 1, 'cell_v', V0, 'cell_ah', Ah, 'cell_ohm', R0, 'soc_init', soc, ...
        'ocv_soc', struct('soc', B.vNoLoad.SOC.Value(:)', 'v', interp2(B.vNoLoad.T.Value, B.vNoLoad.SOC.Value, ...
        B.vNoLoad.V0.Value, Tk, B.vNoLoad.SOC.Value(:))'), ...
        'r_soc_t', struct('soc', B.rTerminal.SOC.Value(:)', 't_k', B.rTerminal.T.Value(:)', 'r', B.rTerminal.R0.Value), ...
        'thermal_mass_j_k', batt_c, 'temp0_k', batt_t0), ...
    'motor_eff_map', motor_map);
fid = fopen(fullfile(outdir, 'inputs.json'), 'w');
fprintf(fid, '%s\n', jsonencode(in, 'PrettyPrint', true));
fclose(fid);
fprintf(['written: ' outdir '/inputs.json and ' outdir '/motor_curve.csv\n' ...
    '  mass %.1f kg, wheelbase %.3f m, rear share %.3f, tire radius %.4f m, mu %.3f, crr %.4f\n' ...
    '  CdA %.3f m^2, ClA %.3f m^2, pack %.1f V / %.4f ohm / %.0f Ah, %.0f kW, ratio %.2f, peak %.0f Nm (4 motors)\n' ...
    '  motor map %d speeds x %d torques (%.0f-%.0f %%), rotating-inertia factor %.3f, tire file terms: %s\n'], ...
    mass_kg, L, a / L, radius, mu, crr, C.Aero.CD.Value * A, -C.Aero.CL.Value * A, V0, R0, Ah, ...
    P.Converter.Power.Value / 1e3, ratios(1), max(T), numel(motor_map.rpm), numel(motor_map.torque_nm), ...
    100 * min(E(:)), 100 * max(E(:)), rot, strjoin(fieldnames(tire)', ' '));
end
