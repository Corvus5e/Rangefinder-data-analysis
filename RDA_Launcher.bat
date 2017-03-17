
REM RDA\Debug\RDA.exe -file scans\sector\D-1.txt -clustering_eps 80 -clustering_minPts 3 -min_rdp_eps 40 -max_dist 100 -min_part_size 8 -merge_dist -50 -merge_angle 20 -filter_kN 5 -filter_threshold 0.9
REM RDA\Debug\RDA.exe -file scans\sector\D-37.txt -reduce_median_window 11 -max_dist_diff 100 -min_segm_points 10 -min_rdp_eps 10 -min_rdp_size 10 -statistacal_kN 15 -statistacal_threashold 1.0
REM RDA\Debug\RDA.exe -file scans\sector\D-31.txt -min_rdp_eps 10 -rdp_eps 20 -min_rdp_size 1
RDA\Debug\RDA.exe scans\dist_error_measurements 25.txt 30.txt 35.txt 40.txt 45.txt 50.txt 55.txt 60.txt 65.txt 70.txt 75.txt 80.txt 85.txt 90.txt 95.txt 100.txt
