! KinematiK geometry benchmark: runs both corners in one import (File > Import > Adams View Command File, this file).
! Set Adams' working directory to this folder first (File > Select Directory): the two command files are read from it,
! and each writes its six <corner>_geo__<measure>.tab results into it when its run ends (12 files in all).
file command read file_name="front_v11_final_geo.cmd"
file command read file_name="rear_v11_final_geo.cmd"
