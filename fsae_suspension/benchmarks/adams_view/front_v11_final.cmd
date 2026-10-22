! KinematiK corner front_v11_final (inputs SHA-256 2d35663ecd27) for Adams View: kinematic benchmark, parallel wheel travel -25 to +25 mm.
! No masses: a kinematic sweep does not use them.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.
! Tie rod: spherical at the upright, Hooke (universal) joint at the fixed rack end (locks its spin). The jack drives wheel-centre height through an inplane primitive, leaving the wheel centre free in x and y.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=front_v11_final_v4
part create rigid_body name_and_position part_name=.front_v11_final_v4.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_v4.uca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_v4.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_v4.tierod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_v4.jack location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.front_v11_final_v4.ground.g_lca location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.front_v11_final_v4.lca.lca_piv location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.front_v11_final_v4.ground.g_uca location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.front_v11_final_v4.uca.uca_piv location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.front_v11_final_v4.lca.lca_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.upright.u_lbj location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.uca.uca_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.upright.u_ubj location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.upright.u_tro location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.tierod.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.tierod.tr_in location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.front_v11_final_v4.ground.g_tri location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.front_v11_final_v4.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.upright.u_tip location=0.0000, 699.9048, 232.3619 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.ground.g_wc0 location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.jack.j_pad location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.jack.j_slide location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_v4.ground.g_jack location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.front_v11_final_v4.lca_rev i_marker_name=.front_v11_final_v4.lca.lca_piv j_marker_name=.front_v11_final_v4.ground.g_lca
constraint create joint revolute joint_name=.front_v11_final_v4.uca_rev i_marker_name=.front_v11_final_v4.uca.uca_piv j_marker_name=.front_v11_final_v4.ground.g_uca
constraint create joint spherical joint_name=.front_v11_final_v4.lbj i_marker_name=.front_v11_final_v4.upright.u_lbj j_marker_name=.front_v11_final_v4.lca.lca_out
constraint create joint spherical joint_name=.front_v11_final_v4.ubj i_marker_name=.front_v11_final_v4.upright.u_ubj j_marker_name=.front_v11_final_v4.uca.uca_out
constraint create joint spherical joint_name=.front_v11_final_v4.tro i_marker_name=.front_v11_final_v4.upright.u_tro j_marker_name=.front_v11_final_v4.tierod.tr_out
constraint create joint hooke joint_name=.front_v11_final_v4.tri i_marker_name=.front_v11_final_v4.tierod.tr_in j_marker_name=.front_v11_final_v4.ground.g_tri
constraint create joint translational joint_name=.front_v11_final_v4.jack_trans i_marker_name=.front_v11_final_v4.jack.j_slide j_marker_name=.front_v11_final_v4.ground.g_jack
constraint create primitive_joint inplane jprim_name=.front_v11_final_v4.jack_pad i_marker_name=.front_v11_final_v4.upright.u_wc j_marker_name=.front_v11_final_v4.jack.j_pad
constraint create motion_generator motion_name=.front_v11_final_v4.jack_motion joint_name=.front_v11_final_v4.jack_trans time_derivative=displacement function="50.0*time - 25.0"
measure create function measure_name=.front_v11_final_v4.travel function="DZ(.front_v11_final_v4.upright.u_wc,.front_v11_final_v4.ground.g_wc0)"
measure create function measure_name=.front_v11_final_v4.camber function="-RTOD*ATAN(DZ(.front_v11_final_v4.upright.u_tip,.front_v11_final_v4.upright.u_wc)/DY(.front_v11_final_v4.upright.u_tip,.front_v11_final_v4.upright.u_wc))"
measure create function measure_name=.front_v11_final_v4.toe_in function="-RTOD*ATAN(DX(.front_v11_final_v4.upright.u_tip,.front_v11_final_v4.upright.u_wc)/DY(.front_v11_final_v4.upright.u_tip,.front_v11_final_v4.upright.u_wc))"
measure create function measure_name=.front_v11_final_v4.caster function="RTOD*ATAN(DX(.front_v11_final_v4.upright.u_ubj,.front_v11_final_v4.upright.u_lbj)/DZ(.front_v11_final_v4.upright.u_ubj,.front_v11_final_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_v4.kpi function="RTOD*ATAN(-DY(.front_v11_final_v4.upright.u_ubj,.front_v11_final_v4.upright.u_lbj)/DZ(.front_v11_final_v4.upright.u_ubj,.front_v11_final_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_v4.wc_dx function="DX(.front_v11_final_v4.upright.u_wc,.front_v11_final_v4.ground.g_wc0)"
measure create function measure_name=.front_v11_final_v4.wc_dy function="DY(.front_v11_final_v4.upright.u_wc,.front_v11_final_v4.ground.g_wc0)"
default model model_name=.front_v11_final_v4
simulation single_run reset
simulation single_run transient type=kinematic end_time=1.0 number_of_steps=50 model_name=.front_v11_final_v4 initial_static=no
