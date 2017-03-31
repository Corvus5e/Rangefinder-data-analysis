
REM RDA\Debug\RDA.exe -file scans\sector\D-1.txt -clustering_eps 80 -clustering_minPts 3 -min_rdp_eps 40 -max_dist 100 -min_part_size 8 -merge_dist -50 -merge_angle 20 -filter_kN 5 -filter_threshold 0.9
REM RDA\Debug\RDA.exe -file scans\sector\D-1.txt -reduce_median_window 11 -max_dist_diff 100 -min_segm_points 10 -min_rdp_eps 10 -min_rdp_size 10 -statistacal_kN 15 -statistacal_threashold 1.0
REM RDA\Debug\RDA.exe -file scans\sector\D-31.txt -min_rdp_eps 10 -rdp_eps 20 -min_rdp_size 1
RDA\Debug\RDA.exe -raw_file scans\sector\D-28.txt -error_file scans\dist_error_measurements\standart_deviations.txt -rdp_error 4 -filter_window 5 -filter_error 1 -min_segm_points 5 -breakpoint_error 2
REM RDA\Debug\RDA.exe scans\dist_error_measurements 28_m.txt 50_m.txt 63_m.txt 100_m.txt D:\git\Rangefinder_Data_Analysis\scans\actual\28_m.txt 3.0
