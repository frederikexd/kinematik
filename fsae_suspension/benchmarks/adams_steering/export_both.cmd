! Writes the six result tables of both steering runs (models _steer_v3), if steer_all.cmd ran but wrote none.
defaults command_file on_error=continue echo_commands=on
file spread_sheet write file_name="front_v11_final_steer__rack_pos.tab" result_set_name=.front_v11_final_steer_v3.Last_Run.rack_pos
file spread_sheet write file_name="front_v11_final_steer__toe_in.tab" result_set_name=.front_v11_final_steer_v3.Last_Run.toe_in
file spread_sheet write file_name="front_v11_final_steer__camber.tab" result_set_name=.front_v11_final_steer_v3.Last_Run.camber
file spread_sheet write file_name="front_v11_joints50_steer__rack_pos.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.rack_pos
file spread_sheet write file_name="front_v11_joints50_steer__toe_in.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.toe_in
file spread_sheet write file_name="front_v11_joints50_steer__camber.tab" result_set_name=.front_v11_joints50_steer_v3.Last_Run.camber
