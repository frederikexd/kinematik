! KinematiK corner front_v11_joints50 for Adams View: link-load benchmark at the design position.
! Each member is a link, spherical at the upright and Hooke at the chassis: zero degrees of freedom.
! Load cases one per step, by time: 0=bump, 1=corner, 2=brake, 3=corner_drive.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=front_v11_joints50_loads_v1
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.uf location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.ur location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.lf location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.lr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.tr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.front_v11_joints50_loads_v1.pr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_ref location=0.0000, 0.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_cp location=0.0000, 605.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_uf location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.uf.uf_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.uf.uf_in location=-111.2342, 232.2605, 280.9244 orientation=-19.7321, 93.1311, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_uf location=-111.2342, 232.2605, 280.9244 orientation=-19.7321, 93.1311, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.uf_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_uf j_marker_name=.front_v11_joints50_loads_v1.uf.uf_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.uf_hk i_marker_name=.front_v11_joints50_loads_v1.uf.uf_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_uf
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_ur location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.ur.ur_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.ur.ur_in location=91.0146, 318.2000, 265.1950 orientation=19.2288, 97.7239, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_ur location=91.0146, 318.2000, 265.1950 orientation=19.2288, 97.7239, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.ur_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_ur j_marker_name=.front_v11_joints50_loads_v1.ur.ur_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.ur_hk i_marker_name=.front_v11_joints50_loads_v1.ur.ur_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_ur
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_lf location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.lf.lf_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.lf.lf_in location=-90.3834, 187.0542, 122.7397 orientation=-12.1513, 89.6130, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_lf location=-90.3834, 187.0542, 122.7397 orientation=-12.1513, 89.6130, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.lf_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_lf j_marker_name=.front_v11_joints50_loads_v1.lf.lf_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.lf_hk i_marker_name=.front_v11_joints50_loads_v1.lf.lf_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_lf
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_lr location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.lr.lr_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.lr.lr_in location=59.1195, 175.5091, 129.5244 orientation=8.9293, 88.6792, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_lr location=59.1195, 175.5091, 129.5244 orientation=8.9293, 88.6792, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.lr_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_lr j_marker_name=.front_v11_joints50_loads_v1.lr.lr_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.lr_hk i_marker_name=.front_v11_joints50_loads_v1.lr.lr_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_lr
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_tr location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.tr.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.tr.tr_in location=-7.7661, 183.2894, 147.1171 orientation=-9.7109, 90.1789, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_tr location=-7.7661, 183.2894, 147.1171 orientation=-9.7109, 90.1789, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.tr_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_tr j_marker_name=.front_v11_joints50_loads_v1.tr.tr_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.tr_hk i_marker_name=.front_v11_joints50_loads_v1.tr.tr_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_tr
marker create marker_name=.front_v11_joints50_loads_v1.upright.u_pr location=8.2200, 522.8100, 132.8400 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.pr.pr_out location=8.2200, 522.8100, 132.8400 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.front_v11_joints50_loads_v1.pr.pr_in location=13.5000, 435.4000, 360.8700 orientation=3.4568, 21.0081, 180.0000
marker create marker_name=.front_v11_joints50_loads_v1.ground.g_pr location=13.5000, 435.4000, 360.8700 orientation=3.4568, 21.0081, 180.0000
constraint create joint spherical joint_name=.front_v11_joints50_loads_v1.pr_sph i_marker_name=.front_v11_joints50_loads_v1.upright.u_pr j_marker_name=.front_v11_joints50_loads_v1.pr.pr_out
constraint create joint hooke joint_name=.front_v11_joints50_loads_v1.pr_hk i_marker_name=.front_v11_joints50_loads_v1.pr.pr_in j_marker_name=.front_v11_joints50_loads_v1.ground.g_pr
force create direct force_vector force_vector_name=.front_v11_joints50_loads_v1.patch_load i_marker_name=.front_v11_joints50_loads_v1.upright.u_cp j_part_name=.front_v11_joints50_loads_v1.ground ref_marker_name=.front_v11_joints50_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 1933.0, 1933.0, 0.0)))" y_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -1800.0, -1800.0, IF(time-2.5: 0.0, 0.0, -1400.0)))" z_force_function="IF(time-0.5: 1200.0, 1200.0, IF(time-1.5: 1200.0, 1200.0, IF(time-2.5: 1161.0, 1161.0, 1300.0)))"
force create direct force_vector force_vector_name=.front_v11_joints50_loads_v1.shaft_load i_marker_name=.front_v11_joints50_loads_v1.upright.u_wc j_part_name=.front_v11_joints50_loads_v1.ground ref_marker_name=.front_v11_joints50_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 0.0, 0.0, -1560.0)))" y_force_function="0.0" z_force_function="0.0"
force create direct torque_vector torque_vector_name=.front_v11_joints50_loads_v1.aligning i_marker_name=.front_v11_joints50_loads_v1.upright.u_cp j_part_name=.front_v11_joints50_loads_v1.ground ref_marker_name=.front_v11_joints50_loads_v1.ground.g_ref x_torque_function="0.0" y_torque_function="0.0" z_torque_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -47000.0, -47000.0, IF(time-2.5: 0.0, 0.0, -37000.0)))"
measure create function measure_name=.front_v11_joints50_loads_v1.T_UF function="FX(.front_v11_joints50_loads_v1.upright.u_uf,.front_v11_joints50_loads_v1.uf.uf_out)*(-0.337118572) + FY(.front_v11_joints50_loads_v1.upright.u_uf,.front_v11_joints50_loads_v1.uf.uf_out)*(-0.939876397) + FZ(.front_v11_joints50_loads_v1.upright.u_uf,.front_v11_joints50_loads_v1.uf.uf_out)*(-0.054620751)"
measure create function measure_name=.front_v11_joints50_loads_v1.T_UR function="FX(.front_v11_joints50_loads_v1.upright.u_ur,.front_v11_joints50_loads_v1.ur.ur_out)*(0.326353986) + FY(.front_v11_joints50_loads_v1.upright.u_ur,.front_v11_joints50_loads_v1.ur.ur_out)*(-0.935644031) + FZ(.front_v11_joints50_loads_v1.upright.u_ur,.front_v11_joints50_loads_v1.ur.ur_out)*(-0.134399863)"
measure create function measure_name=.front_v11_joints50_loads_v1.T_LF function="FX(.front_v11_joints50_loads_v1.upright.u_lf,.front_v11_joints50_loads_v1.lf.lf_out)*(-0.210489019) + FY(.front_v11_joints50_loads_v1.upright.u_lf,.front_v11_joints50_loads_v1.lf.lf_out)*(-0.977572891) + FZ(.front_v11_joints50_loads_v1.upright.u_lf,.front_v11_joints50_loads_v1.lf.lf_out)*(0.006753877)"
measure create function measure_name=.front_v11_joints50_loads_v1.T_LR function="FX(.front_v11_joints50_loads_v1.upright.u_lr,.front_v11_joints50_loads_v1.lr.lr_out)*(0.155175204) + FY(.front_v11_joints50_loads_v1.upright.u_lr,.front_v11_joints50_loads_v1.lr.lr_out)*(-0.987618024) + FZ(.front_v11_joints50_loads_v1.upright.u_lr,.front_v11_joints50_loads_v1.lr.lr_out)*(0.023049823)"
measure create function measure_name=.front_v11_joints50_loads_v1.T_TR function="FX(.front_v11_joints50_loads_v1.upright.u_tr,.front_v11_joints50_loads_v1.tr.tr_out)*(-0.168676168) + FY(.front_v11_joints50_loads_v1.upright.u_tr,.front_v11_joints50_loads_v1.tr.tr_out)*(-0.985666579) + FZ(.front_v11_joints50_loads_v1.upright.u_tr,.front_v11_joints50_loads_v1.tr.tr_out)*(-0.003121729)"
measure create function measure_name=.front_v11_joints50_loads_v1.T_PR function="FX(.front_v11_joints50_loads_v1.upright.u_pr,.front_v11_joints50_loads_v1.pr.pr_out)*(0.021615745) + FY(.front_v11_joints50_loads_v1.upright.u_pr,.front_v11_joints50_loads_v1.pr.pr_out)*(-0.357847027) + FZ(.front_v11_joints50_loads_v1.upright.u_pr,.front_v11_joints50_loads_v1.pr.pr_out)*(0.933530002)"
default model model_name=.front_v11_joints50_loads_v1
simulation single_run reset
simulation single_run transient type=kinematic end_time=3.0 number_of_steps=3 model_name=.front_v11_joints50_loads_v1 initial_static=no
