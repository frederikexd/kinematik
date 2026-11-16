! KinematiK corner front_v11_joints50 (inputs SHA-256 3678a72d786b) for Adams View: steering benchmark, rack -30 to +30 mm in 240 steps at ride height (lower wishbone held at its static angle). Writes front_v11_joints50_steer__<measure>.tab for rack_pos, toe_in, camber.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=front_v11_joints50_steer_v3
part create rigid_body name_and_position part_name=.front_v11_joints50_steer_v3.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_steer_v3.uca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_steer_v3.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_steer_v3.tierod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_steer_v3.rack location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.front_v11_joints50_steer_v3.ground.g_lca location=-90.3834, 187.0542, 122.7397 orientation=85.5842, 87.4093, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.lca.lca_piv location=-90.3834, 187.0542, 122.7397 orientation=85.5842, 87.4093, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.ground.g_uca location=-111.2342, 232.2605, 280.9244 orientation=113.0216, 94.0942, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.uca.uca_piv location=-111.2342, 232.2605, 280.9244 orientation=113.0216, 94.0942, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.lca.lca_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.upright.u_lbj location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.uca.uca_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.upright.u_ubj location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.upright.u_tro location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.tierod.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.tierod.tr_in location=-7.7661, 183.2894, 147.1171 orientation=170.2891, 89.8211, 180.0000
marker create marker_name=.front_v11_joints50_steer_v3.rack.r_tri location=-7.7661, 183.2894, 147.1171 orientation=170.2891, 89.8211, 180.0000
marker create marker_name=.front_v11_joints50_steer_v3.rack.r_sl location=-7.7661, 183.2894, 147.1171 orientation=180.0000, 90.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.ground.g_sl location=-7.7661, 183.2894, 147.1171 orientation=180.0000, 90.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_steer_v3.upright.u_tip location=0.0000, 699.9048, 232.3619 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.front_v11_joints50_steer_v3.lca_rev i_marker_name=.front_v11_joints50_steer_v3.lca.lca_piv j_marker_name=.front_v11_joints50_steer_v3.ground.g_lca
constraint create joint revolute joint_name=.front_v11_joints50_steer_v3.uca_rev i_marker_name=.front_v11_joints50_steer_v3.uca.uca_piv j_marker_name=.front_v11_joints50_steer_v3.ground.g_uca
constraint create joint spherical joint_name=.front_v11_joints50_steer_v3.lbj i_marker_name=.front_v11_joints50_steer_v3.upright.u_lbj j_marker_name=.front_v11_joints50_steer_v3.lca.lca_out
constraint create joint spherical joint_name=.front_v11_joints50_steer_v3.ubj i_marker_name=.front_v11_joints50_steer_v3.upright.u_ubj j_marker_name=.front_v11_joints50_steer_v3.uca.uca_out
constraint create joint spherical joint_name=.front_v11_joints50_steer_v3.tro i_marker_name=.front_v11_joints50_steer_v3.upright.u_tro j_marker_name=.front_v11_joints50_steer_v3.tierod.tr_out
constraint create joint hooke joint_name=.front_v11_joints50_steer_v3.tri i_marker_name=.front_v11_joints50_steer_v3.tierod.tr_in j_marker_name=.front_v11_joints50_steer_v3.rack.r_tri
constraint create joint translational joint_name=.front_v11_joints50_steer_v3.rack_trans i_marker_name=.front_v11_joints50_steer_v3.rack.r_sl j_marker_name=.front_v11_joints50_steer_v3.ground.g_sl
constraint create motion_generator motion_name=.front_v11_joints50_steer_v3.lca_hold joint_name=.front_v11_joints50_steer_v3.lca_rev time_derivative=displacement function="0.0"
constraint create motion_generator motion_name=.front_v11_joints50_steer_v3.rack_motion joint_name=.front_v11_joints50_steer_v3.rack_trans time_derivative=displacement function="60.0*time - 30.0"
measure create function measure_name=.front_v11_joints50_steer_v3.rack_pos function="DY(.front_v11_joints50_steer_v3.rack.r_sl,.front_v11_joints50_steer_v3.ground.g_sl)"
measure create function measure_name=.front_v11_joints50_steer_v3.toe_in function="-RTOD*ATAN(DX(.front_v11_joints50_steer_v3.upright.u_tip,.front_v11_joints50_steer_v3.upright.u_wc)/DY(.front_v11_joints50_steer_v3.upright.u_tip,.front_v11_joints50_steer_v3.upright.u_wc))"
measure create function measure_name=.front_v11_joints50_steer_v3.camber function="-RTOD*ATAN(DZ(.front_v11_joints50_steer_v3.upright.u_tip,.front_v11_joints50_steer_v3.upright.u_wc)/DY(.front_v11_joints50_steer_v3.upright.u_tip,.front_v11_joints50_steer_v3.upright.u_wc))"
default model model_name=.front_v11_joints50_steer_v3
simulation single_run reset
simulation single_run transient type=kinematic end_time=1.0 number_of_steps=240 model_name=.front_v11_joints50_steer_v3 initial_static=no
defaults command_file on_error=abort echo_commands=on
file spread_sheet write file_name="front_v11_joints50_steer__rack_pos.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.rack_pos
file spread_sheet write file_name="front_v11_joints50_steer__toe_in.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.toe_in
file spread_sheet write file_name="front_v11_joints50_steer__camber.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.camber
