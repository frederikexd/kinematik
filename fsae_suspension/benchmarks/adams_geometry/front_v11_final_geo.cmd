! KinematiK corner front_v11_final: contact-patch path, scrub, trail and the rocker's damper length over wheel travel -25 to +25 mm. Pushrod on the upright: spherical there, Hooke at the rocker; rocker on a revolute about its axis. Writes front_v11_final_geo__<measure>.tab for travel, cp_dx, cp_dz, scrub, trail, damper_len.
! KinematiK corner front_v11_final_geo (inputs SHA-256 2d35663ecd27) for Adams View: kinematic benchmark, parallel wheel travel -25 to +25 mm.
! No masses: a kinematic sweep does not use them.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm and degrees.
! Tie rod: spherical at the upright, Hooke (universal) joint at the fixed rack end (locks its spin). The jack drives wheel-centre height through an inplane primitive, leaving the wheel centre free in x and y.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=front_v11_final_geo_v4
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.uca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.tierod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.jack location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.pushrod location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_final_geo_v4.rocker location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.front_v11_final_geo_v4.ground.g_lca location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.front_v11_final_geo_v4.lca.lca_piv location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_uca location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.front_v11_final_geo_v4.uca.uca_piv location=-91.2342, 220.2605, 280.9244 orientation=119.6968, 94.2875, 0.0000
marker create marker_name=.front_v11_final_geo_v4.lca.lca_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_lbj location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.uca.uca_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_ubj location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_tro location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.tierod.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.tierod.tr_in location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_tri location=-23.7661, 167.2894, 147.1171 orientation=168.3238, 89.8299, 180.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_tip location=0.0000, 699.9048, 232.3619 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_wc0 location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.jack.j_pad location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.jack.j_slide location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_jack location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.u_cp location=0.0000, 605.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_cp0 location=0.0000, 605.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_o location=0.0000, 0.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.upright.c_pr location=8.2200, 522.8100, 132.8400 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.pushrod.pr_out location=8.2200, 522.8100, 132.8400 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.pushrod.pr_in location=13.5000, 435.4000, 360.8700 orientation=3.4568, 21.0081, 180.0000
marker create marker_name=.front_v11_final_geo_v4.rocker.r_pr location=13.5000, 435.4000, 360.8700 orientation=3.4568, 21.0081, 180.0000
marker create marker_name=.front_v11_final_geo_v4.rocker.r_piv location=27.0400, 365.4600, 319.9200 orientation=103.4801, 87.5367, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_piv location=27.0400, 365.4600, 319.9200 orientation=103.4801, 87.5367, 0.0000
marker create marker_name=.front_v11_final_geo_v4.rocker.r_spr location=31.2900, 332.3700, 387.9800 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_final_geo_v4.ground.g_spr location=87.5710, 170.6844, 343.6941 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.front_v11_final_geo_v4.lca_rev i_marker_name=.front_v11_final_geo_v4.lca.lca_piv j_marker_name=.front_v11_final_geo_v4.ground.g_lca
constraint create joint revolute joint_name=.front_v11_final_geo_v4.uca_rev i_marker_name=.front_v11_final_geo_v4.uca.uca_piv j_marker_name=.front_v11_final_geo_v4.ground.g_uca
constraint create joint spherical joint_name=.front_v11_final_geo_v4.lbj i_marker_name=.front_v11_final_geo_v4.upright.u_lbj j_marker_name=.front_v11_final_geo_v4.lca.lca_out
constraint create joint spherical joint_name=.front_v11_final_geo_v4.ubj i_marker_name=.front_v11_final_geo_v4.upright.u_ubj j_marker_name=.front_v11_final_geo_v4.uca.uca_out
constraint create joint spherical joint_name=.front_v11_final_geo_v4.tro i_marker_name=.front_v11_final_geo_v4.upright.u_tro j_marker_name=.front_v11_final_geo_v4.tierod.tr_out
constraint create joint hooke joint_name=.front_v11_final_geo_v4.tri i_marker_name=.front_v11_final_geo_v4.tierod.tr_in j_marker_name=.front_v11_final_geo_v4.ground.g_tri
constraint create joint translational joint_name=.front_v11_final_geo_v4.jack_trans i_marker_name=.front_v11_final_geo_v4.jack.j_slide j_marker_name=.front_v11_final_geo_v4.ground.g_jack
constraint create primitive_joint inplane jprim_name=.front_v11_final_geo_v4.jack_pad i_marker_name=.front_v11_final_geo_v4.upright.u_wc j_marker_name=.front_v11_final_geo_v4.jack.j_pad
constraint create motion_generator motion_name=.front_v11_final_geo_v4.jack_motion joint_name=.front_v11_final_geo_v4.jack_trans time_derivative=displacement function="50.0*time - 25.0"
constraint create joint spherical joint_name=.front_v11_final_geo_v4.pr_sph i_marker_name=.front_v11_final_geo_v4.pushrod.pr_out j_marker_name=.front_v11_final_geo_v4.upright.c_pr
constraint create joint hooke joint_name=.front_v11_final_geo_v4.pr_hk i_marker_name=.front_v11_final_geo_v4.pushrod.pr_in j_marker_name=.front_v11_final_geo_v4.rocker.r_pr
constraint create joint revolute joint_name=.front_v11_final_geo_v4.rk_rev i_marker_name=.front_v11_final_geo_v4.rocker.r_piv j_marker_name=.front_v11_final_geo_v4.ground.g_piv
measure create function measure_name=.front_v11_final_geo_v4.travel function="DZ(.front_v11_final_geo_v4.upright.u_wc,.front_v11_final_geo_v4.ground.g_wc0)"
measure create function measure_name=.front_v11_final_geo_v4.camber function="-RTOD*ATAN(DZ(.front_v11_final_geo_v4.upright.u_tip,.front_v11_final_geo_v4.upright.u_wc)/DY(.front_v11_final_geo_v4.upright.u_tip,.front_v11_final_geo_v4.upright.u_wc))"
measure create function measure_name=.front_v11_final_geo_v4.toe_in function="-RTOD*ATAN(DX(.front_v11_final_geo_v4.upright.u_tip,.front_v11_final_geo_v4.upright.u_wc)/DY(.front_v11_final_geo_v4.upright.u_tip,.front_v11_final_geo_v4.upright.u_wc))"
measure create function measure_name=.front_v11_final_geo_v4.caster function="RTOD*ATAN(DX(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj)/DZ(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_geo_v4.kpi function="RTOD*ATAN(-DY(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj)/DZ(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_geo_v4.wc_dx function="DX(.front_v11_final_geo_v4.upright.u_wc,.front_v11_final_geo_v4.ground.g_wc0)"
measure create function measure_name=.front_v11_final_geo_v4.wc_dy function="DY(.front_v11_final_geo_v4.upright.u_wc,.front_v11_final_geo_v4.ground.g_wc0)"
measure create function measure_name=.front_v11_final_geo_v4.cp_dx function="DX(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_cp0)"
measure create function measure_name=.front_v11_final_geo_v4.cp_dz function="DZ(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_cp0)"
measure create function measure_name=.front_v11_final_geo_v4.scrub function="DY(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_o)-(DY(.front_v11_final_geo_v4.upright.u_lbj,.front_v11_final_geo_v4.ground.g_o)+((DZ(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_o)-DZ(.front_v11_final_geo_v4.upright.u_lbj,.front_v11_final_geo_v4.ground.g_o))/DZ(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))*DY(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_geo_v4.trail function="DX(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_o)-(DX(.front_v11_final_geo_v4.upright.u_lbj,.front_v11_final_geo_v4.ground.g_o)+((DZ(.front_v11_final_geo_v4.upright.u_cp,.front_v11_final_geo_v4.ground.g_o)-DZ(.front_v11_final_geo_v4.upright.u_lbj,.front_v11_final_geo_v4.ground.g_o))/DZ(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))*DX(.front_v11_final_geo_v4.upright.u_ubj,.front_v11_final_geo_v4.upright.u_lbj))"
measure create function measure_name=.front_v11_final_geo_v4.damper_len function="DM(.front_v11_final_geo_v4.rocker.r_spr,.front_v11_final_geo_v4.ground.g_spr)"
default model model_name=.front_v11_final_geo_v4
simulation single_run reset
simulation single_run transient type=kinematic end_time=1.0 number_of_steps=50 model_name=.front_v11_final_geo_v4 initial_static=no
defaults command_file on_error=abort echo_commands=on
file spread_sheet write file_name="front_v11_final_geo__travel.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.travel
file spread_sheet write file_name="front_v11_final_geo__cp_dx.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.cp_dx
file spread_sheet write file_name="front_v11_final_geo__cp_dz.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.cp_dz
file spread_sheet write file_name="front_v11_final_geo__scrub.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.scrub
file spread_sheet write file_name="front_v11_final_geo__trail.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.trail
file spread_sheet write file_name="front_v11_final_geo__damper_len.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.damper_len
