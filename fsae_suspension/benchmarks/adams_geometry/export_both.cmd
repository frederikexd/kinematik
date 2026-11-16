! Writes the 12 result tables of both geometry runs. Run both _geo.cmd files first in this session, then set
! File > Select Directory to the folder you want the files in (e.g. your Desktop), then import this file.
defaults command_file on_error=continue echo_commands=on
file spread_sheet write file_name="front_v11_final_geo__travel.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.travel
file spread_sheet write file_name="front_v11_final_geo__cp_dx.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.cp_dx
file spread_sheet write file_name="front_v11_final_geo__cp_dz.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.cp_dz
file spread_sheet write file_name="front_v11_final_geo__scrub.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.scrub
file spread_sheet write file_name="front_v11_final_geo__trail.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.trail
file spread_sheet write file_name="front_v11_final_geo__damper_len.tab" result_set_name=.front_v11_final_geo_v4.Last_Run.damper_len
file spread_sheet write file_name="rear_v11_final_geo__travel.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.travel
file spread_sheet write file_name="rear_v11_final_geo__cp_dx.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.cp_dx
file spread_sheet write file_name="rear_v11_final_geo__cp_dz.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.cp_dz
file spread_sheet write file_name="rear_v11_final_geo__scrub.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.scrub
file spread_sheet write file_name="rear_v11_final_geo__trail.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.trail
file spread_sheet write file_name="rear_v11_final_geo__damper_len.tab" result_set_name=.rear_v11_final_geo_v4.Last_Run.damper_len
