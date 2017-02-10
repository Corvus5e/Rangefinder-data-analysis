
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