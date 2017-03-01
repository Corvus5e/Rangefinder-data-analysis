
#include <rda\line_extraction.h>

#include <rda\approximied_cloud_part.h>
#include <rda\filtering.h>
#include <rda\curve.h>
#include <rda\segmentation.h>

void rda::casmExtractor(rda::CloudPtr cloud,
						std::vector<double>& distances,
						double clustering_eps,
						int clustering_minpts, 
						double max_dist,
						double min_rdp_eps,
						int min_part_size,
						double merge_dist,
						double merge_angle,
						int filter_kn,
						double filter_threshold,
						std::vector<std::vector<rda::ApproximiedCloudPart>>& line_clusters)
{
	rda::CloudPtr cloud_filtered (new rda::Cloud);

	if(filter_kn > distances.size())
		filter_kn = distances.size();

	if(min_rdp_eps < 4)
		min_rdp_eps = 4;

	std::vector<int> filtered_dists_indexes;	
	rda::statisticalDistanceFilter(distances, filter_kn, filter_threshold, filtered_dists_indexes); 					

	for(int i = 0; i < filtered_dists_indexes.size(); i++){				
		cloud_filtered->push_back(cloud->at(filtered_dists_indexes[i]));
	}

	std::vector<rda::CloudPtr> clusters;
	rda::euclideanClusterExctraction(cloud_filtered, clusters, clustering_eps, clustering_minpts, cloud_filtered->size());		

	std::vector<rda::CloudPart> clusters_patrs;		

	std::vector<rda::Line> min_clusters_patrs;
	std::vector<rda::CloudPart> min_clusters_parts_lines;

	std::vector<std::vector<rda::ApproximiedCloudPart>> approximied_parts;	
	
	for(auto i = 0; i < clusters.size(); i++){					

		std::vector<rda::CloudPart> parts;			

		std::vector<rda::Line> min_partition_lines;
		std::vector<rda::CloudPart> min_parts;

		std::vector<rda::ApproximiedCloudPart> approximied;
		std::vector<rda::ApproximiedCloudPart> merged;

		rda::monotonePartitioning(clusters[i], max_dist, min_part_size, parts);
		rda::adaptiveLineSegmentation(parts, min_part_size, min_rdp_eps, min_parts);
		rda::lsLineApproximation(min_parts, approximied);
		rda::segmentsMerging(approximied, merge_dist, merge_angle, merged);

		line_clusters.push_back(merged);
	}
}


void rda::basmExtractor(rda::CloudPtr cloud,
						std::vector<double>& distances,
						std::vector<rda::Range>& part_ranges,
						int statistical_kn, 
						double statistical_threashold,
						int min_segm_points,
						double max_dist_diff, 
						int rmed_window_size, 
						int min_rdp_eps, 
						int min_rdp_size,
						std::vector<std::vector<rda::ApproximiedCloudPart>>& line_clusters)
{
	//statistical filter
	std::vector<std::vector<int>> stat_filtered_indexes(part_ranges.size());		
	for(auto i = 0; i < part_ranges.size(); i++){			
		rda::statisticalDistanceFilter(distances, part_ranges[i], statistical_kn, statistical_threashold, stat_filtered_indexes[i]);			
	}				
	
	
	// naive breakponit detector
	std::vector<std::vector<int>> break_indexes;
	for(auto i = 0; i < stat_filtered_indexes.size(); i++){
		//rda::naive_beakpoint_detector(distances, stat_filtered_indexes[i], max_dist_diff, min_segm_points, break_indexes);  			
		rda::naiveBreakpointDetector(cloud, stat_filtered_indexes[i], max_dist_diff, min_segm_points, break_indexes);
	}
	
	// reduce median filter	
	std::vector<rda::CloudPtr> reduce_median_clouds;
	std::vector<std::vector<int>> reduce_median_indexes(break_indexes.size());		
	for(auto i = 0; i < break_indexes.size(); i++){
		int window_size;
		if(break_indexes[i].size() < rmed_window_size){
			window_size = break_indexes[i].size();
		}
		else
			window_size = rmed_window_size;

		rda::reduceMedianFilter(distances, break_indexes[i], window_size, reduce_median_indexes[i]);						
	
		rda::CloudPtr rm_cloud(new rda::Cloud);				
		for(auto j = reduce_median_indexes[i].begin(); j != reduce_median_indexes[i].end(); ++j){
			rm_cloud->push_back(cloud->at(*j));
		}
		reduce_median_clouds.push_back(rm_cloud);
	}
	
	// Adaptive Split&Merge
	if(min_rdp_eps < 4)
		min_rdp_eps = 4;

	std::vector<std::vector<rda::CloudPart>> min_cloud_parts;
	for(auto it = reduce_median_clouds.begin(); it != reduce_median_clouds.end(); ++it){
		std::vector<rda::CloudPart> min_parts;
		rda::CloudPart cp(*it);
		rda::adaptiveRDP(cp, min_rdp_eps, min_rdp_size, min_parts, rda::stDevSignificanceEstimator);		
		min_cloud_parts.push_back(min_parts);
	}
	
	//Least Squares	
	line_clusters.resize(min_cloud_parts.size());
	std::vector<rda::Line> ls_lines;
	for(auto i = 0; i < min_cloud_parts.size(); i++){
		rda::lsLineApproximation(min_cloud_parts[i], line_clusters[i]);
	}

}