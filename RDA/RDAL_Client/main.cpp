// RDAL_Client.cpp: определяет точку входа для консольного приложения.
//

#include <iostream>
#include <iomanip>
#include <time.h>

#include "rda_client\common.h"
#include "rda_client\io.h"
#include "rda_client\console.h"
#include "rda_client\point_cloud.h"
#include "rda_client\vizualizer.h"

#include <RDAL.h>


using namespace std;
using namespace client;

void printOutput(double**& output, int size){

	std::cout << "Size : " << size << std:: endl << std::endl;

	for(int i = 0; i < size; i++){
		std::cout << "Cluster : " << i << std:: endl;
		for(int j = 0; i < output[i][0]; j++){			
			std::cout << output[i][j] << " ";
			if(j % pointSize() == 0)
				std::cout << std::endl;
		}
		
	}
}

int main(int argc, char* argv[])
{	
	Console::readArgs(argc, argv);
	std::string file = Console::getParam("-file");

	//casm params
	double clustering_eps = atof(client::Console::getParam("-clustering_eps").c_str());
	int clustering_minPts = atoi(client::Console::getParam("-clustering_minPts").c_str());
	double min_rdp_eps = atof(client::Console::getParam("-min_rdp_eps").c_str());
	double max_dist = atof(client::Console::getParam("-max_dist").c_str());
	double min_part_size = atof(client::Console::getParam("-min_part_size").c_str());
	double merge_dist = atof(client::Console::getParam("-merge_dist").c_str());
	double merge_angle = atof(client::Console::getParam("-merge_angle").c_str());
	int filter_kN = atoi(client::Console::getParam("-filter_kN").c_str());
	double filter_threshold = atof(client::Console::getParam("-filter_threshold").c_str());
	
	//basm params
	int statistacal_kN = atof(client::Console::getParam("-statistacal_kN").c_str());
	double statistacal_threashold = atof(client::Console::getParam("-statistacal_threashold").c_str());
	int min_segm_points = atof(client::Console::getParam("-min_segm_points").c_str());
	double max_dist_diff = atof(client::Console::getParam("-max_dist_diff").c_str());
	int reduce_median_window = atof(client::Console::getParam("-reduce_median_window").c_str());
	//double min_rdp_eps = atof(client::Console::getParam("-min_rdp_eps").c_str());
	int min_rdp_size = atof(client::Console::getParam("-min_rdp_size").c_str());

	int ls_order = atof(client::Console::getParam("-ls_order").c_str());
	double ls_step = atof(client::Console::getParam("-ls_step").c_str());

	int minPts = atof(client::Console::getParam("-min_pts").c_str());
	double eps = atof(client::Console::getParam("-eps").c_str());

	std::vector<double> data;

	
	try {

		readXYDScene(file, data);
		std::cout << "Read points : " << data.size()  << std::endl;

		double **output;
		int clusters_number = 0;
		
		double amount_time = 0;
		clock_t amount_clock;
		amount_clock = clock();
		
		//casmLineExtractor(&data[0], clustering_eps, clustering_minPts, min_rdp_eps, max_dist, min_part_size, merge_dist, merge_angle, filter_kN, filter_threshold, output, clusters_number);
		basmLineExtractor(&data[0], statistacal_kN, statistacal_threashold, min_segm_points, max_dist_diff,
									reduce_median_window,
									min_rdp_eps, min_rdp_size, output, clusters_number);

		//rdpMinimization(&data[0], min_rdp_eps, output, clusters_number);
		//lsLineApproximation(&data[0], output, clusters_number);
		//statisticalDistanceFilter(&data[0], statistacal_kN, statistacal_threashold, output, clusters_number);
		//statisticalFilter(&data[0], statistacal_kN, statistacal_threashold, output, clusters_number);
		//reduceMedianFilter(&data[0], reduce_median_window,  output, clusters_number);
		//lsRDPApproximation(&data[0], ls_order, ls_step, min_rdp_eps, output, clusters_number);  
		//naiveBreakpointDetector(&data[0], max_dist_diff, min_segm_points, output, clusters_number);
		//euclideanClusterExctraction(&data[0], eps, minPts, 9999, output, clusters_number);
		//adaptiveRDPStD(&data[0], min_rdp_eps, min_rdp_size, output, clusters_number);
		
		amount_time = ((float)(clock() - amount_clock)) / CLOCKS_PER_SEC;
		std::cout << "Amount time :" << amount_time  << "sec" << std::endl;
		
		//printOutput(output, clusters_number);
		
		//clearMemory(output, clusters_number);
		
		std::vector<PointCloud> lines_cluster;
		
		for(int i=0; i < clusters_number; i++){
			PointCloud lc(output[i], pointSize());
			std::cout << "Cluster " << i << std::endl;
			for(auto jt = lc.begin(); jt != lc.end(); jt++){
				std::cout << jt.x() << " " << jt.y() << std::endl;
			}
			lines_cluster.push_back(lc);
		}	

		PointCloud source_cloud(&data[0], pointSize());

		Vizualizer::init(&argc, argv);
		Vizualizer v1;
		v1.createWindow("Lines", 600, 600, 2, 2);	

		v1.addClouds(lines_cluster, client::LINES, 1.0f);	
		v1.addCloud(source_cloud, client::POINTS, 1.0f, 1.0f, 1.0f, 0.5f);		
				
		Vizualizer::start();		

	}
	catch(RDAException& e){
		std::cout << e.what() << std::endl; 
		return -1;
	}

	return 0;	
}

