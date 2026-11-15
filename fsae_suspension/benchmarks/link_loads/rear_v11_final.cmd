! KinematiK corner rear_v11_final for Adams View: link-load benchmark at the design position.
! Each member is a link, spherical at the upright and Hooke at the chassis: zero degrees of freedom.
! Load cases one per step, by time: 0=bump, 1=corner, 2=brake, 3=corner_drive.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=rear_v11_final_loads_v1
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.uf location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.ur location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.lf location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.lr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.tr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.rear_v11_final_loads_v1.pr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.rear_v11_final_loads_v1.ground.g_ref location=0.0000, 0.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.upright.u_cp location=0.0000, 605.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.upright.u_uf location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.uf.uf_out location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.uf.uf_in location=-330.4000, 337.3884, 381.7560 orientation=-57.9654, 78.2567, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_uf location=-330.4000, 337.3884, 381.7560 orientation=-57.9654, 78.2567, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.uf_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_uf j_marker_name=.rear_v11_final_loads_v1.uf.uf_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.uf_hk i_marker_name=.rear_v11_final_loads_v1.uf.uf_in j_marker_name=.rear_v11_final_loads_v1.ground.g_uf
marker create marker_name=.rear_v11_final_loads_v1.upright.u_ur location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.ur.ur_out location=3.0000, 546.0000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.ur.ur_in location=-112.3068, 206.4000, 305.7968 orientation=-18.7543, 89.0740, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_ur location=-112.3068, 206.4000, 305.7968 orientation=-18.7543, 89.0740, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.ur_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_ur j_marker_name=.rear_v11_final_loads_v1.ur.ur_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.ur_hk i_marker_name=.rear_v11_final_loads_v1.ur.ur_in j_marker_name=.rear_v11_final_loads_v1.ground.g_ur
marker create marker_name=.rear_v11_final_loads_v1.upright.u_lf location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.lf.lf_out location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.lf.lf_in location=-264.2220, 183.9600, 97.8845 orientation=-33.8672, 95.1734, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_lf location=-264.2220, 183.9600, 97.8845 orientation=-33.8672, 95.1734, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.lf_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_lf j_marker_name=.rear_v11_final_loads_v1.lf.lf_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.lf_hk i_marker_name=.rear_v11_final_loads_v1.lf.lf_in j_marker_name=.rear_v11_final_loads_v1.ground.g_lf
marker create marker_name=.rear_v11_final_loads_v1.upright.u_lr location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.lr.lr_out location=-5.0000, 570.2000, 140.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.lr.lr_in location=23.4217, 85.0000, 143.8859 orientation=3.3524, 89.5419, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_lr location=23.4217, 85.0000, 143.8859 orientation=3.3524, 89.5419, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.lr_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_lr j_marker_name=.rear_v11_final_loads_v1.lr.lr_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.lr_hk i_marker_name=.rear_v11_final_loads_v1.lr.lr_in j_marker_name=.rear_v11_final_loads_v1.ground.g_lr
marker create marker_name=.rear_v11_final_loads_v1.upright.u_tr location=78.0000, 564.2000, 162.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.tr.tr_out location=78.0000, 564.2000, 162.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.tr.tr_in location=-40.6917, 341.5715, 146.3011 orientation=-28.0638, 93.5607, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_tr location=-40.6917, 341.5715, 146.3011 orientation=-28.0638, 93.5607, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.tr_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_tr j_marker_name=.rear_v11_final_loads_v1.tr.tr_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.tr_hk i_marker_name=.rear_v11_final_loads_v1.tr.tr_in j_marker_name=.rear_v11_final_loads_v1.ground.g_tr
marker create marker_name=.rear_v11_final_loads_v1.upright.u_pr location=29.0926, 540.9097, 154.2929 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.pr.pr_out location=29.0926, 540.9097, 154.2929 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.rear_v11_final_loads_v1.pr.pr_in location=67.6641, 403.7393, 322.4686 orientation=15.7057, 40.2736, 180.0000
marker create marker_name=.rear_v11_final_loads_v1.ground.g_pr location=67.6641, 403.7393, 322.4686 orientation=15.7057, 40.2736, 180.0000
constraint create joint spherical joint_name=.rear_v11_final_loads_v1.pr_sph i_marker_name=.rear_v11_final_loads_v1.upright.u_pr j_marker_name=.rear_v11_final_loads_v1.pr.pr_out
constraint create joint hooke joint_name=.rear_v11_final_loads_v1.pr_hk i_marker_name=.rear_v11_final_loads_v1.pr.pr_in j_marker_name=.rear_v11_final_loads_v1.ground.g_pr
force create direct force_vector force_vector_name=.rear_v11_final_loads_v1.patch_load i_marker_name=.rear_v11_final_loads_v1.upright.u_cp j_part_name=.rear_v11_final_loads_v1.ground ref_marker_name=.rear_v11_final_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 1933.0, 1933.0, 0.0)))" y_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -1800.0, -1800.0, IF(time-2.5: 0.0, 0.0, -1400.0)))" z_force_function="IF(time-0.5: 1200.0, 1200.0, IF(time-1.5: 1200.0, 1200.0, IF(time-2.5: 1161.0, 1161.0, 1300.0)))"
force create direct force_vector force_vector_name=.rear_v11_final_loads_v1.shaft_load i_marker_name=.rear_v11_final_loads_v1.upright.u_wc j_part_name=.rear_v11_final_loads_v1.ground ref_marker_name=.rear_v11_final_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 0.0, 0.0, -1560.0)))" y_force_function="0.0" z_force_function="0.0"
force create direct torque_vector torque_vector_name=.rear_v11_final_loads_v1.aligning i_marker_name=.rear_v11_final_loads_v1.upright.u_cp j_part_name=.rear_v11_final_loads_v1.ground ref_marker_name=.rear_v11_final_loads_v1.ground.g_ref x_torque_function="0.0" y_torque_function="0.0" z_torque_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -47000.0, -47000.0, IF(time-2.5: 0.0, 0.0, -37000.0)))"
measure create function measure_name=.rear_v11_final_loads_v1.T_UF function="FX(.rear_v11_final_loads_v1.upright.u_uf,.rear_v11_final_loads_v1.uf.uf_out)*(-0.829984190) + FY(.rear_v11_final_loads_v1.upright.u_uf,.rear_v11_final_loads_v1.uf.uf_out)*(-0.519329004) + FZ(.rear_v11_final_loads_v1.upright.u_uf,.rear_v11_final_loads_v1.uf.uf_out)*(0.203527958)"
measure create function measure_name=.rear_v11_final_loads_v1.T_UR function="FX(.rear_v11_final_loads_v1.upright.u_ur,.rear_v11_final_loads_v1.ur.ur_out)*(-0.321467908) + FY(.rear_v11_final_loads_v1.upright.u_ur,.rear_v11_final_loads_v1.ur.ur_out)*(-0.946782556) + FZ(.rear_v11_final_loads_v1.upright.u_ur,.rear_v11_final_loads_v1.ur.ur_out)*(0.016160963)"
measure create function measure_name=.rear_v11_final_loads_v1.T_LF function="FX(.rear_v11_final_loads_v1.upright.u_lf,.rear_v11_final_loads_v1.lf.lf_out)*(-0.555000223) + FY(.rear_v11_final_loads_v1.upright.u_lf,.rear_v11_final_loads_v1.lf.lf_out)*(-0.826948657) + FZ(.rear_v11_final_loads_v1.upright.u_lf,.rear_v11_final_loads_v1.lf.lf_out)*(-0.090170235)"
measure create function measure_name=.rear_v11_final_loads_v1.T_LR function="FX(.rear_v11_final_loads_v1.upright.u_lr,.rear_v11_final_loads_v1.lr.lr_out)*(0.058475268) + FY(.rear_v11_final_loads_v1.upright.u_lr,.rear_v11_final_loads_v1.lr.lr_out)*(-0.998256843) + FZ(.rear_v11_final_loads_v1.upright.u_lr,.rear_v11_final_loads_v1.lr.lr_out)*(0.007994940)"
measure create function measure_name=.rear_v11_final_loads_v1.T_TR function="FX(.rear_v11_final_loads_v1.upright.u_tr,.rear_v11_final_loads_v1.tr.tr_out)*(-0.469545770) + FY(.rear_v11_final_loads_v1.upright.u_tr,.rear_v11_final_loads_v1.tr.tr_out)*(-0.880721139) + FZ(.rear_v11_final_loads_v1.upright.u_tr,.rear_v11_final_loads_v1.tr.tr_out)*(-0.062105114)"
measure create function measure_name=.rear_v11_final_loads_v1.T_PR function="FX(.rear_v11_final_loads_v1.upright.u_pr,.rear_v11_final_loads_v1.pr.pr_out)*(0.174988437) + FY(.rear_v11_final_loads_v1.upright.u_pr,.rear_v11_final_loads_v1.pr.pr_out)*(-0.622303725) + FZ(.rear_v11_final_loads_v1.upright.u_pr,.rear_v11_final_loads_v1.pr.pr_out)*(0.762966002)"
default model model_name=.rear_v11_final_loads_v1
simulation single_run reset
simulation single_run transient type=kinematic end_time=3.0 number_of_steps=3 model_name=.rear_v11_final_loads_v1 initial_static=no
