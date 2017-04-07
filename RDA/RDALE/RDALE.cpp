
#define USE_MSM_EXTRACTOR

#include <rda\line_extraction.h>
#include <rda\rda_exception.h>
#include <rda\curve.h>

#include "RDALE.h"

namespace {

	const int  POINT_SIZE = 5;
	const int NO_SIGNAL_SPLIT = 5;

	void convertRawArrayToPointCloud(double* input, rda::CloudPtr output_cloud,	 std::vector<double>& distances, std::vector<rda::Range>& part_ranges)
	{
		int size = input[0] - 1;
	
		if( ( size % POINT_SIZE) == 0){
			int no_data = 0;
			int start_index = 0;
			bool closed_part = true;
		
			for(int i = 0; i < size ; i+= POINT_SIZE){
				if(input[i+5] != 0.0){
					if(closed_part){
						start_index = distances.size();
						closed_part = false;
					}
					output_cloud->points.push_back(rda::Point(input[i+1], input[i+2], input[i+3]));
					distances.push_back(input[i+5]);			
					no_data = 0;
				}
				else{
					if(++no_data == NO_SIGNAL_SPLIT){
						if(!closed_part){
							part_ranges.push_back(rda::Range(start_index, distances.size() - 1));
							closed_part = true;
						}
					}
				}
			}
			if(!closed_part)
				part_ranges.push_back(rda::Range(start_index, distances.size() - 1));
		}
		else{		
			throw rda::RdaException(std::string("Failed to read input format: Wrong size."));
		}
	}

	void convertApproxCloudPartsToFormatArray(std::vector<std::vector<rda::ApproximiedCloudPart>>& lines, double**& output, int& clusters_size){
	
	clusters_size = lines.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[2 * POINT_SIZE * lines[i].size() + 1]; // +1 for size element
		output[i][0] = 2 * POINT_SIZE * lines[i].size() + 1; // line is 2 points

		int k = 0;

		for(int j=0; j < lines[i].size(); j++){

			output[i][k+1] = lines[i][j].approx_line().start().x;
			output[i][k+2] = lines[i][j].approx_line().start().y;
			output[i][k+3] = 1;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			output[i][k+6] = lines[i][j].approx_line().end().x;
			output[i][k+7] = lines[i][j].approx_line().end().y;
			output[i][k+8] = 1;
			output[i][k+9] = 1;
			output[i][k+10] = 1;

			k+= POINT_SIZE * 2;
		}

	}
}
}

void extractLines(double* input, std::string& error_file, double rdp_eps, int filter_window_size, double filter_error,
				  int min_segment_points, double breakpoint_error, double merge_distance, double merge_angle, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);
	std::vector<std::vector<rda::ApproximiedCloudPart>> lines_clusters;
	std::vector<std::vector<rda::ApproximiedCloudPart>> merged_lines(1); // merging in one cluster

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	rda::msmExtractor(cloud, distances, part_ranges, error_file, rdp_eps, filter_window_size, filter_error, min_segment_points, breakpoint_error, lines_clusters);

	if(merge_distance < 0){ // no merging
		convertApproxCloudPartsToFormatArray(lines_clusters, output, clusters_size);
	}
	else{ // merging

		std::vector<rda::ApproximiedCloudPart> line_segments;		
		for(auto i = 0; i < lines_clusters.size(); i++){
			for(auto j = 0; j < lines_clusters[i].size(); j++){
				line_segments.push_back(lines_clusters[i][j]);
			}
		}

		rda::segmentsMerging(line_segments, merge_distance, merge_angle, merged_lines[0]);
		convertApproxCloudPartsToFormatArray(merged_lines, output, clusters_size);
	}
}

int pointSize(){

	return POINT_SIZE;
}

void clearMemory(double**& ptr, int size){
	for(int i = 0; i < size; i++){
		delete[] ptr[i];
		ptr[i] = 0;
	}
	delete[] ptr;

}