! KinematiK direct-acting corner direct_upright: damper length over wheel travel -25 to +25 mm. Damper outer on the upright, chassis mount fixed. Writes direct_upright_mr__travel.tab and direct_upright_mr__damper_len.tab.
! KinematiK corner direct_upright_mr (inputs SHA-256 247b21888cad) for Adams View: kinematic benchmark, parallel wheel travel -25 to +25 mm.
! No masses: a kinematic sweep does not use them.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.
! Tie rod: spherical at the upright, Hooke (universal) joint at the fixed rack end (locks its spin). The jack drives wheel-centre height through an inplane primitive, leaving the wheel centre free in x and y.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=direct_upright_mr_v4
part create rigid_body name_and_position part_name=.direct_upright_mr_v4.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_upright_mr_v4.uca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_upright_mr_v4.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_upright_mr_v4.tierod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_upright_mr_v4.jack location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.direct_upright_mr_v4.ground.g_lca location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.direct_upright_mr_v4.lca.lca_piv location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.direct_upright_mr_v4.ground.g_uca location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.direct_upright_mr_v4.uca.uca_piv location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.direct_upright_mr_v4.lca.lca_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.upright.u_lbj location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.uca.uca_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.upright.u_ubj location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.upright.u_tro location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.tierod.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.tierod.tr_in location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.direct_upright_mr_v4.ground.g_tri location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.direct_upright_mr_v4.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.upright.u_tip location=0.0000, 699.9048, 232.3619 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.ground.g_wc0 location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.jack.j_pad location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.jack.j_slide location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.ground.g_jack location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.upright.dmp_out location=8.2200, 522.8100, 132.8400 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_upright_mr_v4.ground.g_dmp location=8.2200, 332.8100, 372.8400 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.direct_upright_mr_v4.lca_rev i_marker_name=.direct_upright_mr_v4.lca.lca_piv j_marker_name=.direct_upright_mr_v4.ground.g_lca
constraint create joint revolute joint_name=.direct_upright_mr_v4.uca_rev i_marker_name=.direct_upright_mr_v4.uca.uca_piv j_marker_name=.direct_upright_mr_v4.ground.g_uca
constraint create joint spherical joint_name=.direct_upright_mr_v4.lbj i_marker_name=.direct_upright_mr_v4.upright.u_lbj j_marker_name=.direct_upright_mr_v4.lca.lca_out
constraint create joint spherical joint_name=.direct_upright_mr_v4.ubj i_marker_name=.direct_upright_mr_v4.upright.u_ubj j_marker_name=.direct_upright_mr_v4.uca.uca_out
constraint create joint spherical joint_name=.direct_upright_mr_v4.tro i_marker_name=.direct_upright_mr_v4.upright.u_tro j_marker_name=.direct_upright_mr_v4.tierod.tr_out
constraint create joint hooke joint_name=.direct_upright_mr_v4.tri i_marker_name=.direct_upright_mr_v4.tierod.tr_in j_marker_name=.direct_upright_mr_v4.ground.g_tri
constraint create joint translational joint_name=.direct_upright_mr_v4.jack_trans i_marker_name=.direct_upright_mr_v4.jack.j_slide j_marker_name=.direct_upright_mr_v4.ground.g_jack
constraint create primitive_joint inplane jprim_name=.direct_upright_mr_v4.jack_pad i_marker_name=.direct_upright_mr_v4.upright.u_wc j_marker_name=.direct_upright_mr_v4.jack.j_pad
constraint create motion_generator motion_name=.direct_upright_mr_v4.jack_motion joint_name=.direct_upright_mr_v4.jack_trans time_derivative=displacement function="50.0*time - 25.0"
measure create function measure_name=.direct_upright_mr_v4.travel function="DZ(.direct_upright_mr_v4.upright.u_wc,.direct_upright_mr_v4.ground.g_wc0)"
measure create function measure_name=.direct_upright_mr_v4.camber function="-RTOD*ATAN(DZ(.direct_upright_mr_v4.upright.u_tip,.direct_upright_mr_v4.upright.u_wc)/DY(.direct_upright_mr_v4.upright.u_tip,.direct_upright_mr_v4.upright.u_wc))"
measure create function measure_name=.direct_upright_mr_v4.toe_in function="-RTOD*ATAN(DX(.direct_upright_mr_v4.upright.u_tip,.direct_upright_mr_v4.upright.u_wc)/DY(.direct_upright_mr_v4.upright.u_tip,.direct_upright_mr_v4.upright.u_wc))"
measure create function measure_name=.direct_upright_mr_v4.caster function="RTOD*ATAN(DX(.direct_upright_mr_v4.upright.u_ubj,.direct_upright_mr_v4.upright.u_lbj)/DZ(.direct_upright_mr_v4.upright.u_ubj,.direct_upright_mr_v4.upright.u_lbj))"
measure create function measure_name=.direct_upright_mr_v4.kpi function="RTOD*ATAN(-DY(.direct_upright_mr_v4.upright.u_ubj,.direct_upright_mr_v4.upright.u_lbj)/DZ(.direct_upright_mr_v4.upright.u_ubj,.direct_upright_mr_v4.upright.u_lbj))"
measure create function measure_name=.direct_upright_mr_v4.wc_dx function="DX(.direct_upright_mr_v4.upright.u_wc,.direct_upright_mr_v4.ground.g_wc0)"
measure create function measure_name=.direct_upright_mr_v4.wc_dy function="DY(.direct_upright_mr_v4.upright.u_wc,.direct_upright_mr_v4.ground.g_wc0)"
measure create function measure_name=.direct_upright_mr_v4.damper_len function="DM(.direct_upright_mr_v4.upright.dmp_out,.direct_upright_mr_v4.ground.g_dmp)"
default model model_name=.direct_upright_mr_v4
simulation single_run reset
simulation single_run transient type=kinematic end_time=1.0 number_of_steps=50 model_name=.direct_upright_mr_v4 initial_static=no
file spread_sheet write file_name="direct_upright_mr__travel.tab" result_set_name=.direct_upright_mr_v4.Last_Run.travel
file spread_sheet write file_name="direct_upright_mr__damper_len.tab" result_set_name=.direct_upright_mr_v4.Last_Run.damper_len
