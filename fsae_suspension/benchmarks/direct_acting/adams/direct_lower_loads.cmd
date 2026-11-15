! KinematiK direct-acting corner direct_lower for Adams View: member forces at the design position.
! Links spherical at the moving body, Hooke at the chassis; the lower wishbone is a rigid body on a revolute pivot carrying the damper: zero degrees of freedom. T_PR is the damper force.
! Load cases one per step, by time: 0=bump, 1=corner, 2=brake, 3=corner_drive.
! Frame: x rearward, y outboard (right-hand corner), z up from the ground; mm, N, N*mm.
default units length=mm mass=kg force=newton time=second angle=degrees
model create model_name=direct_lower_loads_v1
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.upright location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.lca location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.uf location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.ur location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.tr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
part create rigid_body name_and_position part_name=.direct_lower_loads_v1.pr location=0.0, 0.0, 0.0 orientation=0.0, 0.0, 0.0
marker create marker_name=.direct_lower_loads_v1.ground.g_ref location=0.0000, 0.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.upright.u_cp location=0.0000, 605.0000, 0.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.upright.u_wc location=0.0000, 600.0000, 228.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.ground.g_lca location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.direct_lower_loads_v1.lca.lca_piv location=-90.3834, 155.0542, 122.7397 orientation=97.0456, 87.6702, 0.0000
marker create marker_name=.direct_lower_loads_v1.lca.lca_out location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.upright.u_lbj location=-5.0000, 583.6000, 120.0000 orientation=0.0000, 0.0000, 0.0000
constraint create joint revolute joint_name=.direct_lower_loads_v1.lca_rev i_marker_name=.direct_lower_loads_v1.lca.lca_piv j_marker_name=.direct_lower_loads_v1.ground.g_lca
constraint create joint spherical joint_name=.direct_lower_loads_v1.lbj i_marker_name=.direct_lower_loads_v1.upright.u_lbj j_marker_name=.direct_lower_loads_v1.lca.lca_out
marker create marker_name=.direct_lower_loads_v1.upright.m_uf location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.uf.uf_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.uf.uf_in location=-91.2342, 220.2605, 280.9244 orientation=-16.0268, 93.0845, 180.0000
marker create marker_name=.direct_lower_loads_v1.ground.g_uf location=-91.2342, 220.2605, 280.9244 orientation=-16.0268, 93.0845, 180.0000
constraint create joint spherical joint_name=.direct_lower_loads_v1.uf_sph i_marker_name=.direct_lower_loads_v1.upright.m_uf j_marker_name=.direct_lower_loads_v1.uf.uf_out
constraint create joint hooke joint_name=.direct_lower_loads_v1.uf_hk i_marker_name=.direct_lower_loads_v1.uf.uf_in j_marker_name=.direct_lower_loads_v1.ground.g_uf
marker create marker_name=.direct_lower_loads_v1.upright.m_ur location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.ur.ur_out location=6.5000, 560.5000, 300.0000 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.ur.ur_in location=91.0146, 324.2000, 265.1950 orientation=19.6800, 97.8959, 180.0000
marker create marker_name=.direct_lower_loads_v1.ground.g_ur location=91.0146, 324.2000, 265.1950 orientation=19.6800, 97.8959, -180.0000
constraint create joint spherical joint_name=.direct_lower_loads_v1.ur_sph i_marker_name=.direct_lower_loads_v1.upright.m_ur j_marker_name=.direct_lower_loads_v1.ur.ur_out
constraint create joint hooke joint_name=.direct_lower_loads_v1.ur_hk i_marker_name=.direct_lower_loads_v1.ur.ur_in j_marker_name=.direct_lower_loads_v1.ground.g_ur
marker create marker_name=.direct_lower_loads_v1.upright.m_tr location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.tr.tr_out location=53.3754, 540.5729, 148.2487 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.tr.tr_in location=-23.7661, 167.2894, 147.1171 orientation=-11.6762, 90.1701, 180.0000
marker create marker_name=.direct_lower_loads_v1.ground.g_tr location=-23.7661, 167.2894, 147.1171 orientation=-11.6762, 90.1701, -180.0000
constraint create joint spherical joint_name=.direct_lower_loads_v1.tr_sph i_marker_name=.direct_lower_loads_v1.upright.m_tr j_marker_name=.direct_lower_loads_v1.tr.tr_out
constraint create joint hooke joint_name=.direct_lower_loads_v1.tr_hk i_marker_name=.direct_lower_loads_v1.tr.tr_in j_marker_name=.direct_lower_loads_v1.ground.g_tr
marker create marker_name=.direct_lower_loads_v1.lca.m_pr location=-5.6580, 479.0204, 136.5330 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.pr.pr_out location=-5.6580, 479.0204, 136.5330 orientation=0.0000, 0.0000, 0.0000
marker create marker_name=.direct_lower_loads_v1.pr.pr_in location=-5.6580, 329.0204, 396.5330 orientation=0.0000, 29.9816, 180.0000
marker create marker_name=.direct_lower_loads_v1.ground.g_pr location=-5.6580, 329.0204, 396.5330 orientation=0.0000, 29.9816, 180.0000
constraint create joint spherical joint_name=.direct_lower_loads_v1.pr_sph i_marker_name=.direct_lower_loads_v1.lca.m_pr j_marker_name=.direct_lower_loads_v1.pr.pr_out
constraint create joint hooke joint_name=.direct_lower_loads_v1.pr_hk i_marker_name=.direct_lower_loads_v1.pr.pr_in j_marker_name=.direct_lower_loads_v1.ground.g_pr
force create direct force_vector force_vector_name=.direct_lower_loads_v1.patch_load i_marker_name=.direct_lower_loads_v1.upright.u_cp j_part_name=.direct_lower_loads_v1.ground ref_marker_name=.direct_lower_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 1933.0, 1933.0, 0.0)))" y_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -1800.0, -1800.0, IF(time-2.5: 0.0, 0.0, -1400.0)))" z_force_function="IF(time-0.5: 1200.0, 1200.0, IF(time-1.5: 1200.0, 1200.0, IF(time-2.5: 1161.0, 1161.0, 1300.0)))"
force create direct force_vector force_vector_name=.direct_lower_loads_v1.shaft_load i_marker_name=.direct_lower_loads_v1.upright.u_wc j_part_name=.direct_lower_loads_v1.ground ref_marker_name=.direct_lower_loads_v1.ground.g_ref x_force_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: 0.0, 0.0, IF(time-2.5: 0.0, 0.0, -1560.0)))" y_force_function="0.0" z_force_function="0.0"
force create direct torque_vector torque_vector_name=.direct_lower_loads_v1.aligning i_marker_name=.direct_lower_loads_v1.upright.u_cp j_part_name=.direct_lower_loads_v1.ground ref_marker_name=.direct_lower_loads_v1.ground.g_ref x_torque_function="0.0" y_torque_function="0.0" z_torque_function="IF(time-0.5: 0.0, 0.0, IF(time-1.5: -47000.0, -47000.0, IF(time-2.5: 0.0, 0.0, -37000.0)))"
measure create function measure_name=.direct_lower_loads_v1.T_UF function="FX(.direct_lower_loads_v1.upright.m_uf,.direct_lower_loads_v1.uf.uf_out)*(-0.275686663) + FY(.direct_lower_loads_v1.upright.m_uf,.direct_lower_loads_v1.uf.uf_out)*(-0.959740363) + FZ(.direct_lower_loads_v1.upright.m_uf,.direct_lower_loads_v1.uf.uf_out)*(-0.053807993)"
measure create function measure_name=.direct_lower_loads_v1.T_UR function="FX(.direct_lower_loads_v1.upright.m_ur,.direct_lower_loads_v1.ur.ur_out)*(0.333573797) + FY(.direct_lower_loads_v1.upright.m_ur,.direct_lower_loads_v1.ur.ur_out)*(-0.932661322) + FZ(.direct_lower_loads_v1.upright.m_ur,.direct_lower_loads_v1.ur.ur_out)*(-0.137373143)"
measure create function measure_name=.direct_lower_loads_v1.T_TR function="FX(.direct_lower_loads_v1.upright.m_tr,.direct_lower_loads_v1.tr.tr_out)*(-0.202379571) + FY(.direct_lower_loads_v1.upright.m_tr,.direct_lower_loads_v1.tr.tr_out)*(-0.979302658) + FZ(.direct_lower_loads_v1.upright.m_tr,.direct_lower_loads_v1.tr.tr_out)*(-0.002968631)"
measure create function measure_name=.direct_lower_loads_v1.T_PR function="FX(.direct_lower_loads_v1.lca.m_pr,.direct_lower_loads_v1.pr.pr_out)*(0.000000000) + FY(.direct_lower_loads_v1.lca.m_pr,.direct_lower_loads_v1.pr.pr_out)*(-0.499722453) + FZ(.direct_lower_loads_v1.lca.m_pr,.direct_lower_loads_v1.pr.pr_out)*(0.866185586)"
default model model_name=.direct_lower_loads_v1
simulation single_run reset
simulation single_run transient type=kinematic end_time=3.0 number_of_steps=3 model_name=.direct_lower_loads_v1 initial_static=no
