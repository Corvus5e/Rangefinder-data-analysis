
#include <rda\common.h>
#include <rda\line.h>
#include <rda\filtering.h>
#include <rda\cloud_part.h>
#include <rda\approximied_cloud_part.h>
#include <rda\rangefinder.h>
#include <rda\rda_exception.h>
#include <rda\segmentation.h>
#include <rda\curve.h>
#include <rda\line_extraction.h>
#include <rda\function\LeastSquares.h>

#include "RDAL.h"

const int  POINT_SIZE = 5;

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
				if(++no_data == 3){
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

// Ignores zero data
void convertRawArrayToPointCloud(double* input, rda::CloudPtr output_cloud,	 std::vector<double>& distances)
{		
	int size = input[0] - 1;

	if( ( size % POINT_SIZE) == 0){		 
	
		for(int i = 0; i < size ; i+= POINT_SIZE){
			if(input[i+5] != 0.0){
				output_cloud->points.push_back(rda::Point(input[i+1], input[i+2], input[i+3]));
				distances.push_back(input[i+5]);
			}
		}
	}
	else{		
		throw rda::RdaException(std::string("Failed to read input format: Wrong size."));
	}
}

void convertRawArrayToPointCloud(double* input, rda::CloudPtr output_cloud){
		
	int size = input[0] - 1;

	if( ( size % POINT_SIZE) == 0){		 
	
		for(int i = 0; i < size ; i+= POINT_SIZE){
			output_cloud->points.push_back(rda::Point(input[i+1], input[i+2], input[i+3]));
		}
	}
	else{		

		throw rda::RdaException(std::string("Wrong size."));
	}

}

void convertPointCloudToFormatArray(std::vector<rda::CloudPtr>& clusters, double**& output, int& clusters_size){

	clusters_size = clusters.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[POINT_SIZE * clusters[i]->size() + 1]; // +1 for size element
		output[i][0] = POINT_SIZE * clusters[i]->size() + 1;

		int k = 0;

		for(int j=0; j < clusters[i]->size(); j++){

			output[i][k+1] = clusters[i]->points[j].x;
			output[i][k+2] = clusters[i]->points[j].y;
			output[i][k+3] = clusters[i]->points[j].z;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			k+= POINT_SIZE;
		}

	}
}

void convertLinesToFormatArray(std::vector<std::vector<rda::Line>>& lines, double**& output, int& clusters_size){

	clusters_size = lines.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[2 * POINT_SIZE * lines[i].size() + 1]; // +1 for size element
		output[i][0] = 2 * POINT_SIZE * lines[i].size() + 1; // line is 2 points

		int k = 0;

		for(int j=0; j < lines[i].size(); j++){

			output[i][k+1] = lines[i][j].start().x;
			output[i][k+2] = lines[i][j].start().y;
			output[i][k+3] = 1;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			output[i][k+6] = lines[i][j].end().x;
			output[i][k+7] = lines[i][j].end().y;
			output[i][k+8] = 1;
			output[i][k+9] = 1;
			output[i][k+10] = 1;

			k+= POINT_SIZE * 2;
		}

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

void convertCloudPartsLinesToFormatArray(std::vector<std::vector<rda::CloudPart>>& lines, double**& output, int& clusters_size){
	
	clusters_size = lines.size();

	output = new double*[clusters_size];	


	for(int i=0; i < clusters_size; i++){

		output[i] = new double[2 * POINT_SIZE * lines[i].size() + 1]; // +1 for size element
		output[i][0] = 2 * POINT_SIZE * lines[i].size() + 1; // line is 2 points

		int k = 0;

		for(int j=0; j < lines[i].size(); j++){

			output[i][k+1] = lines[i][j].line().start().x;
			output[i][k+2] = lines[i][j].line().start().y;
			output[i][k+3] = 1;
			output[i][k+4] = 1;
			output[i][k+5] = 1;

			output[i][k+6] = lines[i][j].line().end().x;
			output[i][k+7] = lines[i][j].line().end().y;
			output[i][k+8] = 1;
			output[i][k+9] = 1;
			output[i][k+10] = 1;

			k+= POINT_SIZE * 2;
		}

	}
}



 /*  ------- RDAL.h implementation ------- */

void casmLineExtractor(double* input, double clustering_eps, int clustering_minPts, double min_rdp_eps,
									  double max_dist, int min_part_size, double merge_dist,
									  double merge_angle, int filter_kN, double filter_treshold,
									  double**& output, int& clusters_size)
{	
	rda::CloudPtr cloud(new rda::Cloud);
	std::vector<double> distances;
	std::vector<std::vector<rda::ApproximiedCloudPart>> merged_lines;


	convertRawArrayToPointCloud(input, cloud, distances); 

	rda::casmExtractor(cloud, distances, clustering_eps, clustering_minPts, max_dist,
						min_rdp_eps, min_part_size, merge_dist,
						merge_angle, filter_kN, filter_treshold,merged_lines); 

	convertApproxCloudPartsToFormatArray(merged_lines, output, clusters_size);	
}

void basmLineExtractor(double* input, int statistical_kn, double statistical_threashold, int min_segm_points,
									  double max_dist_diff,	int rmed_window_size, int min_rdp_eps, int min_rdp_size,
									  double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);
	std::vector<std::vector<rda::ApproximiedCloudPart>> lines_clusters;

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	rda::basmExtractor(cloud, distances, part_ranges, statistical_kn, statistical_threashold,
					   min_segm_points, max_dist_diff, rmed_window_size, min_rdp_eps,
					   min_rdp_size, lines_clusters);

	convertApproxCloudPartsToFormatArray(lines_clusters, output, clusters_size);
}

void rdpMinimization(double* input, double threshold, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	std::vector<std::vector<rda::CloudPart>> lines_clusters(part_ranges.size());

	for(auto i = 0; i < part_ranges.size(); i++){
		rda::rdpMinimization(rda::CloudPart(cloud, part_ranges[i]), part_ranges[i].start, part_ranges[i].end, threshold, lines_clusters[i]);
	}

	convertCloudPartsLinesToFormatArray(lines_clusters, output, clusters_size);
}

void lsLineApproximation(double* input, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);
	std::vector<std::vector<rda::ApproximiedCloudPart>> lines_clusters(part_ranges.size());	

	for(auto i = 0; i < part_ranges.size(); i++){
		std::vector<rda::CloudPart> cp;
		cp.push_back(rda::CloudPart(cloud, part_ranges[i]));

		rda::lsLineApproximation(cp, lines_clusters[i]);
	}

	convertApproxCloudPartsToFormatArray(lines_clusters, output, clusters_size);

}

void lsRDPApproximation(double* input, int order, double step, double threashold, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	std::vector<std::vector<rda::CloudPart>> lines_clusters(part_ranges.size());

	for(auto i = 0; i < part_ranges.size(); i++){

		rda::lsRDPApproximation(rda::CloudPart(cloud, part_ranges[i]), order, step, threashold, lines_clusters[i]);
	}

	convertCloudPartsLinesToFormatArray(lines_clusters, output, clusters_size);
}

void statisticalDistanceFilter(double* input, int neighbours_number, double threshold, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	std::vector<rda::CloudPtr> filtered_clouds;

	for(auto i = 0; i < part_ranges.size(); i++){
		rda::CloudPtr c(new rda::Cloud);
		std::vector<int> indexes;
		rda::statisticalDistanceFilter(distances, part_ranges[i], neighbours_number, threshold, indexes);
		for(auto j = 0; j < indexes.size(); j++){
			c->push_back(cloud->at(indexes[j]));
		}
		filtered_clouds.push_back(c);
	}
	convertPointCloudToFormatArray(filtered_clouds, output, clusters_size);
}

void statisticalFilter(double* input, int neighbours_number, double threshold, double**& output, int& clusters_size)
{
	std::vector<double> distances;			
	rda::CloudPtr cloud (new rda::Cloud);

	convertRawArrayToPointCloud(input, cloud, distances);

	rda::CloudPtr filtered_cloud(new rda::Cloud);
	std::vector<rda::CloudPtr> filtered_clouds;

	rda::statisticalFilter(cloud, *filtered_cloud, neighbours_number, threshold);
	filtered_clouds.push_back(filtered_cloud);

	convertPointCloudToFormatArray(filtered_clouds, output, clusters_size);
}

void reduceMedianFilter(double* input, int window_size, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);
	std::vector<rda::CloudPtr> filtered_clouds;

	for(auto i = 0; i < part_ranges.size(); i++)
	{
		std::vector<int> indexes;
		rda::reduceMedianFilter(distances, part_ranges[i], window_size, indexes);
		rda::CloudPtr fc(new rda::Cloud);
		for(auto j = 0; j < indexes.size(); j++){
			fc->push_back(cloud->at(indexes[j]));
		}
		filtered_clouds.push_back(fc);
	}

	convertPointCloudToFormatArray(filtered_clouds, output, clusters_size);
}

void naiveBreakpointDetector(double* input, double max_diff, int min_points, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);
	std::vector<rda::CloudPtr> clouds;

	for(auto i = 0; i < part_ranges.size(); i++){		
		std::vector<std::vector<int>> indexes;
		rda::naiveBreakpointDetector(rda::CloudPart(cloud, part_ranges[i]), max_diff, min_points, indexes);		
		for(auto j = 0; j < indexes.size(); j++){
			rda::CloudPtr pc(new rda::Cloud);
			for(auto k = 0; k < indexes[j].size(); k++){
				pc->push_back(cloud->at(indexes[j][k]));
			}			
			clouds.push_back(pc);
		}		
	}

	convertPointCloudToFormatArray(clouds, output, clusters_size);
}

void euclideanClusterExctraction(double* input, double eps, int min_points, int max_points, double**& output, int& clusters_size)
{
	rda::CloudPtr cloud(new rda::Cloud);
	std::vector<double> distances;	

	convertRawArrayToPointCloud(input, cloud, distances); 

	std::vector<rda::CloudPtr> clusters;
	rda::euclideanClusterExctraction(cloud, clusters, eps, min_points, max_points);

	convertPointCloudToFormatArray(clusters, output, clusters_size);
}

void adaptiveRDP(double* input, double min_error, int min_size, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	std::vector<std::vector<rda::CloudPart>> lines_clusters(part_ranges.size());

	for(auto i = 0; i < part_ranges.size(); i++){		
		rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, lines_clusters[i]);
	}

	convertCloudPartsLinesToFormatArray(lines_clusters, output, clusters_size);	
}

void adaptiveRDPStD(double* input, double min_error, int min_size, double**& output, int& clusters_size)
{
	std::vector<double> distances;		
	std::vector<rda::Range> part_ranges;
	rda::CloudPtr cloud (new rda::Cloud);	

	convertRawArrayToPointCloud(input, cloud, distances, part_ranges);

	std::vector<std::vector<rda::CloudPart>> lines_clusters(part_ranges.size());

	for(auto i = 0; i < part_ranges.size(); i++){		
		rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, lines_clusters[i], rda::stDevSignificanceEstimator);
	}

	convertCloudPartsLinesToFormatArray(lines_clusters, output, clusters_size);	
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

