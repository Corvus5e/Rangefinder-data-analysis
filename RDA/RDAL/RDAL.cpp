// RDAL.cpp: определяет экспортированные функции для приложения DLL.
//

#include "stdafx.h"

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

#include "RDAL.h"

const int  POINT_SIZE = 5;

/* ---- Functions to work with points in X,Y ----- */

void convertRawArrayToPointCloud(double* input, rda::CloudPtr output_cloud,	 std::vector<double>& distances){
		
	int size = input[0] - 1;

	if( ( size % POINT_SIZE) == 0){		 
	
		for(int i = 0; i < size ; i+= POINT_SIZE){
			output_cloud->points.push_back(rda::Point(input[i+1], input[i+2], input[i+3]));
			distances.push_back(input[i+5]);
		}
	}
	else{		

		throw rda::RdaException(std::string("Wrong size."));
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

/* ---- Functions to work with distances and robots positions ----- */

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




 /*  ------- RDAL.h implementation ------- */

void casmExtractor(double* input, double clustering_eps, int clustering_minPts, double min_rdp_eps, double max_dist, int min_part_size, double merge_dist, double merge_angle, int filter_kN, double filter_treshold, double**& output, int& clusters_size){
	
	rda::CloudPtr cloud(new rda::Cloud);
	std::vector<double> distances;
	std::vector<std::vector<rda::ApproximiedCloudPart>> merged_lines;


	convertRawArrayToPointCloud(input, cloud, distances); 

	rda::casmExtractor(cloud, distances, clustering_eps, clustering_minPts, max_dist,
						min_rdp_eps, min_part_size, merge_dist,
						merge_angle, filter_kN, filter_treshold,merged_lines); 

	convertApproxCloudPartsToFormatArray(merged_lines, output, clusters_size);

	clusters_size = 49;
}

int pointSize(){

	return POINT_SIZE;
}

/*void clearMemory(double*& ptr){
	delete[] ptr;
	ptr = 0;
}*/

void clearMemory(double**& ptr, int size){
	for(int i = 0; i < size; i++){
		delete[] ptr[i];
		ptr[i] = 0;
	}
	delete[] ptr;

}

