
#include <rda\line_extraction.h>

#include <rda\approximied_cloud_part.h>
#include <rda\filtering.h>
#include <rda\curve.h>
#include <rda\segmentation.h>
#include <rda\function\LeastSquares.h>

#include <fstream>

#ifdef USE_CASM_EXTRACTOR

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

#endif

#ifdef USE_BASM_EXTRACTOR

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

#endif

#ifdef USE_MSM_EXTRACTOR

namespace {
	
	double maxDeviationFromEndLine2d(rda::CloudPart& cloud, std::vector<double>& distances, function::LeastSquares& error_function, int& index)
	{
		index = 0;

		double max_dev = 0;
		double curr_dev(max_dev);
		double dist_hyp = 0;
		rda::Line line_hyp(cloud.first_point(), cloud.last_point());				
		double max_diff = 0;
		rda::Point proj_point(0,0,0);

		std::vector<double> devs;
		for(auto i = cloud.begin() + 1; i < cloud.end(); i++){
			proj_point = rda::projectionPointToLine(line_hyp, cloud.at(i));
			dist_hyp = (distances[i-1] + distances[i+1]) / 2;
			curr_dev = rda::distancePointToLine(cloud.first_point(), cloud.last_point(), cloud.at(i)) / (error_function.value(dist_hyp));
			devs.push_back(curr_dev);
			if(curr_dev > max_dev){
				max_dev = curr_dev;
				index = i;
				max_diff = std::abs(cloud.at(i).y - dist_hyp);
			}
		}

		return  max_dev;		
	}

	void rdp2d(rda::CloudPart& cloud, std::vector<double>& distances, double error, function::LeastSquares& error_function, int min_segm_size, std::vector<rda::Range>& ranges)
	{
		int mid_index = 0;
		double max_dev = maxDeviationFromEndLine2d(cloud, distances, error_function, mid_index);			

		if(max_dev > error){
			rdp2d(rda::CloudPart(cloud.cloud(), rda::Range(cloud.begin(), mid_index)), distances, error, error_function, min_segm_size, ranges);
			rdp2d(rda::CloudPart(cloud.cloud(), rda::Range(mid_index, cloud.end())), distances, error, error_function, min_segm_size, ranges);
		}
		else{			
			ranges.push_back(cloud.range());
		}
	}

	void filter(rda::CloudPart cloud, int window_size, function::LeastSquares& error_function, double error, std::vector<int>& indexes)
	{		
		int w = window_size / 2;		

		indexes.push_back(0);

		for(auto it = cloud.begin(); it != cloud.end(); ++it){

			rda::Range window(it - w, it + w);
			if(window.start < cloud.begin())
				window = rda::Range(cloud.begin(), cloud.begin() + window_size -1);			
			if(window.end > cloud.end())
				window = rda::Range(cloud.end() - w + 1, cloud.end());			
			
			function::LeastSquares ls(1);
			ls.init(cloud.cloud(), window.start, window.end);
			ls.approximate();

			double dist_hyp = ls.value(cloud.at(it).x);
			if(abs(cloud.at(it).y - dist_hyp) <= error*error_function.value(dist_hyp)){
				indexes.push_back(it);
			}
		}

		indexes.push_back(cloud.end());
	}

	void adaptiveBreakpointDetector(rda::CloudPart cloud, function::LeastSquares& error_function, double error, int min_points, std::vector<rda::Range>& ranges)
	{		
		auto start = cloud.begin();
		auto end = start + 1;

		for(end = cloud.begin() + 1; end <= cloud.end(); end++){		
			double expected_error = error_function.value(std::min(cloud.at(end - 1).y, cloud.at(end).y));
			if( std::abs( cloud.at(end - 1).y - cloud.at(end).y)  > error *  expected_error){
				if( end - start >= min_points){
					ranges.push_back(rda::Range(start, end - 1));										
				}
				start = end;
			}		
		}
		
		if(end - start >= min_points)
			ranges.push_back(rda::Range(start, end - 1));	
	}

	void readErrorFile(std::string& file, rda::CloudPtr error_cloud)
	{
		std::string line;
		std::ifstream my_file(file);
		
		if(my_file.is_open()){
			int line_num = 0;

			while(getline(my_file, line)){
				if(line_num >= 1){
					std::vector<std::string> slices;
					rda::split(line, "|",  slices);
					error_cloud->push_back( rda::Point(atof(slices[0].c_str()), atof(slices[1].c_str()), 1.0) );
				}

				line_num++;
			}

			my_file.close();
		}
	}
}

void rda::msmExtractor(rda::CloudPtr cloud,
			   std::vector<double>& distances,
			   std::vector<rda::Range>& part_ranges,
			   std::string& error_file, 
			   double rdp_eps,
			   int filter_window_size,
			   double filter_error, 
			   int min_segment_points,
			   double breakpoint_error,
			   std::vector<std::vector<rda::ApproximiedCloudPart>>& line_clusters)
	{
		rda::CloudPtr error_cloud(new rda::Cloud);
		rda::CloudPtr raw_dist_cloud(new rda::Cloud);
		std::vector<std::vector<double>> dists_filtered((part_ranges.size()));
		std::vector<rda::CloudPtr> dist_filtered_cloud;
		std::vector<rda::CloudPtr> filtered_clouds;

		// read errors and interpolate
		readErrorFile(error_file, error_cloud);
		function::LeastSquares ls(8);
		ls.init(error_cloud, 0, error_cloud->size() - 1);
		ls.approximate();

		for(auto i = 0; i < distances.size(); i++){
			raw_dist_cloud->push_back(rda::Point(i, distances[i], 1.0f));
		}

		// breakpoint detector using measured error function
		std::vector<std::vector<rda::Range>> filtered_ranges(part_ranges.size());
		for(auto i = 0; i < part_ranges.size(); i++){
			adaptiveBreakpointDetector(rda::CloudPart(raw_dist_cloud, part_ranges[i]), ls, breakpoint_error, min_segment_points, filtered_ranges[i]);
			dist_filtered_cloud.push_back(rda::CloudPtr(new rda::Cloud));
			filtered_clouds.push_back(rda::CloudPtr(new rda::Cloud));
			for(auto it = filtered_ranges[i].begin(); it != filtered_ranges[i].end(); ++it){
				for(auto jt = it->start; jt <= it->end; jt++){
					dist_filtered_cloud.back()->push_back(raw_dist_cloud->at(jt));
					dists_filtered[i].push_back(distances[jt]);
					filtered_clouds.back()->push_back(cloud->at(jt));
				}
			}
		}

		std::vector<rda::CloudPtr> twise_filtered_clouds;
		std::vector<std::vector<double>> tf_distances(filtered_clouds.size());
		std::vector<std::vector<int>> tf_indexes(filtered_clouds.size());				
		
		for(auto i = 0; i < filtered_clouds.size(); i++){	

			twise_filtered_clouds.push_back(rda::CloudPtr(new rda::Cloud));

			if(filtered_clouds[i]->size() > 0){

				if(dist_filtered_cloud[i]->size() >= filter_window_size)
					filter(rda::CloudPart(dist_filtered_cloud[i]), filter_window_size, ls, filter_error, tf_indexes[i]);								
				else
					filter(rda::CloudPart(dist_filtered_cloud[i]), dist_filtered_cloud[i]->size(), ls, filter_error, tf_indexes[i]);								

				for(auto jt = tf_indexes[i].begin(); jt != tf_indexes[i].end(); ++jt){
					twise_filtered_clouds.back()->push_back(filtered_clouds[i]->at(*jt));
					tf_distances[i].push_back(dists_filtered[i].at(*jt));
				}
			}
		}

		std::vector<std::vector<rda::Range>> rdp_ranges(filtered_ranges.size());
		std::vector<rda::CloudPtr> rdp_clouds;
		
		for(auto i = 0; i < twise_filtered_clouds.size(); i++){
			if(twise_filtered_clouds[i]->size() > 0){
				rdp2d(rda::CloudPart(twise_filtered_clouds[i]), tf_distances[i], rdp_eps, ls, min_segment_points, rdp_ranges[i]);
				rdp_clouds.push_back(rda::CloudPtr(new rda::Cloud));
				for(auto j = 0; j < rdp_ranges[i].size(); j++){					
					rdp_clouds.back()->push_back(twise_filtered_clouds[i]->at(rdp_ranges[i][j].start));
					rdp_clouds.back()->push_back(twise_filtered_clouds[i]->at(rdp_ranges[i][j].end));
				}
			}
		}

		line_clusters.resize(rdp_ranges.size());

		for(auto i = 0; i < rdp_ranges.size(); i++){			
			for(auto j = rdp_ranges[i].begin(); j != rdp_ranges[i].end(); ++j){
				if(j->size() >=3 )
					line_clusters[i].push_back(rda::ApproximiedCloudPart(twise_filtered_clouds[i], *j, rda::lsLineApproximation(rda::CloudPart(twise_filtered_clouds[i], *j))));				
				else
					line_clusters[i].push_back(rda::ApproximiedCloudPart(twise_filtered_clouds[i], *j, rda::Line(twise_filtered_clouds[i]->at(j->start), twise_filtered_clouds[i]->at(j->end))));				
			}
		}
	}

#endif