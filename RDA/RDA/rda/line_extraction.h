
#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <rda\common.h>
#include <rda\approximied_cloud_part.h>


namespace rda {
	
	/*
	 * Clustering Adaptive Split and Merge
	 * 
	 * 1. Statistical filter (Normal distribution)
	 * 2. Clustering DbScan
	 * 3. Naive breakpoint detector
	 * 4. Adaptive split and merge
	 * 5. Least Squares
	 * 6. Merging segmants
	 *
	*/

	void casmExtractor(rda::CloudPtr cloud,
						std::vector<double>& distances,
						double clustering_eps,	  // 80
						int clustering_minPts,	  // 0
						double max_dist,	 	  // 100
						double min_rdp_eps,		  // 15
						int min_part_size,		  // 8
						double merge_dist,		  // 50
						double merge_angle,		  // 20
						int filter_kn,			  // 5
						double filter_threshold,  // 0.9
						std::vector<std::vector<rda::ApproximiedCloudPart>>& line_clusters);
	
	/*
	 * 
	 * Adaptive Split and Merge with new 2 filters (statistacil and reduce median filter)
	 * 
	 * 1. Split by 3 n/a
	 * 2. Split by Naive breakpoint detector in 2d (cont dists in 2d then split - find out indexes)
	 * 3. Filtration by Statistacal filter
	 * 2. Filtration by Reduce median filter
	 * 4. Adaptive Rumer
	 * 5. Least Squares Approximation
	 *
	 */

	void basmExtractor(rda::CloudPtr cloud,
						std::vector<double>& distances,
						std::vector<rda::Range>& part_ranges,
						int statistical_kn, // 
						double statistical_threashold,
						int min_segm_points,
						double max_dist_diff, 
						int rmed_window_size, 
						int min_rdp_eps, 
						int min_rdp_size,
						std::vector<std::vector<rda::ApproximiedCloudPart>>& line_clusters);
}

#endif