! KinematiK corner rear_v11_final (inputs SHA-256 1d2b4a0a7fca) for Adams View: kinematic benchmark, parallel wheel travel -25 to +25 mm.
! No masses: a kinematic sweep does not use them.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.
! Tie rod: spherical at the upright, Hooke (universal) joint at the fixed rack end (locks its spin). The jack drives wheel-centre height through an inplane primitive, leaving the wheel centre free in x and y.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=rear_v11_final_v4
part create rigid_body name_and_position part_name=.rear_v11_final_v4.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_v4.uca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_v4.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_v4.tierod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_v4.jack location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.rear_v11_final_v4.ground.g_lca location=-264.2220, 183.9600, 97.8845 orientation=71.0149, 81.4006, 0.0000
marker create marker_name=.rear_v11_final_v4.lca.lca_piv location=-264.2220, 183.9600, 97.8845 orientation=71.0149, 81.4006, 0.0000
marker create marker_name=.rear_v11_final_v4.ground.g_uca location=-330.4000, 337.3884, 381.7560 orientation=59.0106, 106.6243, 0.0000
marker create marker_name=.rear_v11_final_v4.uca.uca_piv location=-330.4000, 337.3884, 381.7560 orientation=59.0106, 106.6243, 0.0000
marker create marker_name=.rear_v11_final_v4.lca.lca_out location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.upright.u_lbj location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.uca.uca_out location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.upright.u_ubj location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.upright.u_tro location=78.0000, 564.2000, 162.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.tierod.tr_out location=78.0000, 564.2000, 162.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.tierod.tr_in location=-40.6917, 341.5715, 146.3011 orientation=151.9362, 86.4393, 180.0000
marker create marker_name=.rear_v11_final_v4.ground.g_tri location=-40.6917, 341.5715, 146.3011 orientation=151.9362, 86.4393, 180.0000
marker create marker_name=.rear_v11_final_v4.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.upright.u_tip location=0.0000, 699.8971, 232.5363 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.ground.g_wc0 location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.jack.j_pad location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.jack.j_slide location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_v4.ground.g_jack location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.rear_v11_final_v4.lca_rev i_marker_name=.rear_v11_final_v4.lca.lca_piv j_marker_name=.rear_v11_final_v4.ground.g_lca
constraint create joint revolute joint_name=.rear_v11_final_v4.uca_rev i_marker_name=.rear_v11_final_v4.uca.uca_piv j_marker_name=.rear_v11_final_v4.ground.g_uca
constraint create joint spherical joint_name=.rear_v11_final_v4.lbj i_marker_name=.rear_v11_final_v4.upright.u_lbj j_marker_name=.rear_v11_final_v4.lca.lca_out
constraint create joint spherical joint_name=.rear_v11_final_v4.ubj i_marker_name=.rear_v11_final_v4.upright.u_ubj j_marker_name=.rear_v11_final_v4.uca.uca_out
constraint create joint spherical joint_name=.rear_v11_final_v4.tro i_marker_name=.rear_v11_final_v4.upright.u_tro j_marker_name=.rear_v11_final_v4.tierod.tr_out
constraint create joint hooke joint_name=.rear_v11_final_v4.tri i_marker_name=.rear_v11_final_v4.tierod.tr_in j_marker_name=.rear_v11_final_v4.ground.g_tri
constraint create joint translational joint_name=.rear_v11_final_v4.jack_trans i_marker_name=.rear_v11_final_v4.jack.j_slide j_marker_name=.rear_v11_final_v4.ground.g_jack
constraint create primitive_joint inplane jprim_name=.rear_v11_final_v4.jack_pad i_marker_name=.rear_v11_final_v4.upright.u_wc j_marker_name=.rear_v11_final_v4.jack.j_pad
constraint create motion_generator motion_name=.rear_v11_final_v4.jack_motion joint_name=.rear_v11_final_v4.jack_trans time_derivative=displacement function="50.0*time - 25.0"
measure create function measure_name=.rear_v11_final_v4.travel function="DZ(.rear_v11_final_v4.upright.u_wc,.rear_v11_final_v4.ground.g_wc0)"
measure create function measure_name=.rear_v11_final_v4.camber function="-RTOD*ATAN(DZ(.rear_v11_final_v4.upright.u_tip,.rear_v11_final_v4.upright.u_wc)/DY(.rear_v11_final_v4.upright.u_tip,.rear_v11_final_v4.upright.u_wc))"
measure create function measure_name=.rear_v11_final_v4.toe_in function="-RTOD*ATAN(DX(.rear_v11_final_v4.upright.u_tip,.rear_v11_final_v4.upright.u_wc)/DY(.rear_v11_final_v4.upright.u_tip,.rear_v11_final_v4.upright.u_wc))"
measure create function measure_name=.rear_v11_final_v4.caster function="RTOD*ATAN(DX(.rear_v11_final_v4.upright.u_ubj,.rear_v11_final_v4.upright.u_lbj)/DZ(.rear_v11_final_v4.upright.u_ubj,.rear_v11_final_v4.upright.u_lbj))"
measure create function measure_name=.rear_v11_final_v4.kpi function="RTOD*ATAN(-DY(.rear_v11_final_v4.upright.u_ubj,.rear_v11_final_v4.upright.u_lbj)/DZ(.rear_v11_final_v4.upright.u_ubj,.rear_v11_final_v4.upright.u_lbj))"
measure create function measure_name=.rear_v11_final_v4.wc_dx function="DX(.rear_v11_final_v4.upright.u_wc,.rear_v11_final_v4.ground.g_wc0)"
measure create function measure_name=.rear_v11_final_v4.wc_dy function="DY(.rear_v11_final_v4.upright.u_wc,.rear_v11_final_v4.ground.g_wc0)"
default model model_name=.rear_v11_final_v4
simulation single_run reset
simulation single_run transient type=kinematic end_time=1.0 number_of_steps=50 model_name=.rear_v11_final_v4 initial_static=no
