REM RDA\Debug\RDAL_Client.exe -file scans\D-1.txt -clustering_eps 80 -clustering_minPts 3 -min_rdp_eps 40 -max_dist 100 -min_part_size 8 -merge_dist -50 -merge_angle 20 -filter_kN 5 -filter_threshold 0.9
RDA\Debug\RDAL_Client.exe -file scans\sector\D-32.txt -reduce_median_window 11 -max_dist_diff 100 -min_segm_points 10 -min_rdp_eps 5 -min_rdp_size 10 -statistacal_kN 15 -statistacal_threashold 0.5
REM @echo Press any key to exit
REM @pause > nul