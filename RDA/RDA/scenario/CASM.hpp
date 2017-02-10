
#ifndef ASM_H
#define AMS_H

#include <time.h>

#include <scenario\Scenario.h>

#include <rda\common.h>
#include <rda\approximied_cloud_part.h>
#include <rda\filtering.h>
#include <rda\curve.h>
#include <rda\segmentation.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>


class CASM : public Scanario {
public :
	void start(int argc, char* argv[]){
		std::cout << std::endl;

		std::vector<double> distances;			
		
		rda::CloudPtr dist_cloud (new rda::Cloud);
		rda::CloudPtr ls_dist_cloud (new rda::Cloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> );	
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZ> );	
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> );

		rda::CloudPtr filtered_dist_cloud (new rda::Cloud);
		
		rda::Console console;
		console.readArgs(argc, argv);	
		
		//Params		
		int kN = atof(console.getParam("-filter_kN").c_str());
		double threshold = atof(console.getParam("-filter_threashold").c_str());
		double eps = atof(console.getParam("-eps").c_str());
		int minPts = atoi(console.getParam("-minPts").c_str());
		double max_dist = atof(console.getParam("-max_dist").c_str());					
		double merge_dist = atof(console.getParam("-merge_dist").c_str());
		double merge_angle = atof(console.getParam("-merge_angle").c_str());
		int min_part_size = atoi(console.getParam("-min_part_size").c_str());
		double min_rdp_eps = atof(console.getParam("-min_rdp_eps").c_str());


		try{
			cloud = rda::readScene(console.getParam("-file"), distances);
			std::cout << "Input points number : " << cloud->size() << std::endl;			

			for(int i = 0; i < distances.size(); i++){
				dist_cloud->push_back(rda::Point(i, distances[i], 0));					
			}
			
			#pragma region Filteration			

			clock_t filter_clock;
			filter_clock = clock();

			std::vector<int> filtered_dists_indexes;
			rda::statisticalDistanceFilter(distances, kN, threshold, filtered_dists_indexes); 					

			for(int i = 0; i < filtered_dists_indexes.size(); i++){
				filtered_dist_cloud->push_back(rda::Point(filtered_dists_indexes[i], distances[filtered_dists_indexes[i]], 0));
				//for x,y format
				cloud_filtered->push_back(cloud->at(filtered_dists_indexes[i]));
			}
			
			std::cout << "Input size " << cloud->size() << std::endl;
			std::cout << "Filtered size " << cloud_filtered->size() << std::endl;			

			std::cout << "Filtration time :" <<  ((float)(clock() - filter_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
			std::cout << "\t k-Neighbours : " << kN << std::endl;
			std::cout << "\t threshold : " << threshold << std::endl;

			#pragma endregion

			#pragma region Clustering Euclidean
			
			clock_t segm_clock;
			segm_clock = clock();

			std::vector<rda::CloudPtr> clusters;
			rda::euclideanClusterExctraction(cloud_filtered, clusters, eps, minPts, 999999);		

			std::cout << "Clustering Euclidean time :" <<  ((float)(clock() - segm_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
			std::cout << "\t clusters : " << clusters.size() <<  std::endl;
			
			#pragma endregion 
			
			#pragma region Partitioning

			clock_t partitioning_clock;
			partitioning_clock = clock();

			std::vector<rda::CloudPart> clusters_patrs;		

			std::vector<rda::Line> min_clusters_patrs;
			std::vector<rda::CloudPart> min_clusters_parts_lines;

			std::vector<std::vector<rda::ApproximiedCloudPart>> approximied_parts;
			std::vector<std::vector<rda::ApproximiedCloudPart>> merged_cluster;
			
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

				for(auto i = 0; i < parts.size();i++)
					clusters_patrs.push_back(parts[i]);

				for(auto i = 0; i < min_parts.size();i++)
					min_clusters_patrs.push_back(rda::Line(min_parts[i].first_point(), min_parts[i].last_point()));

				approximied_parts.push_back(approximied);
				merged_cluster.push_back(merged);
			}


			std::cout << "Partitioning time :" <<  ((float)(clock() - partitioning_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;

			std::cout << "Approximation : " << std::endl;
			std::cout << "\t Lengths : "  << std::endl;
			for(int i = 0; i < approximied_parts.size(); i++){
				std::cout << "\t cluster " << i  << " :" << std::endl;
				for(int j = 0; j < approximied_parts[i].size(); j++ ){
					std::cout << "\t " << approximied_parts[i][j].approx_line().length()  << " mm |";
					std::cout << " ( " << approximied_parts[i][j].approx_line().start().x  << " " << approximied_parts[i][j].approx_line().start().y << ")";
					std::cout << " ( " << approximied_parts[i][j].approx_line().end().x  << " " << approximied_parts[i][j].approx_line().end().y << ")" << std::endl;
				}
			}

			std::cout << "Merging : " << std::endl;
			std::cout << "\t Lengths : "  << std::endl;
			for(int i = 0; i < merged_cluster.size(); i++){
				std::cout << "\t cluster " << i  << " :" << std::endl;
				for(int j = 0; j < merged_cluster[i].size(); j++ ){
					std::cout << "\t " << merged_cluster[i][j].approx_line().length()  << " mm |";
					std::cout << " ( " << merged_cluster[i][j].approx_line().start().x  << " " << merged_cluster[i][j].approx_line().start().y << ")";
					std::cout << " ( " << merged_cluster[i][j].approx_line().end().x  << " " << merged_cluster[i][j].approx_line().end().y << ")" << std::endl;
				}
			}

			#pragma endregion		

			#pragma region Vizualization 

			rda::Vizualizer::init(&argc, argv);	
			rda::Vizualizer v;	
			rda::Vizualizer v2;	
			rda::Vizualizer v3;	
			rda::Vizualizer v4;	
			rda::Vizualizer v5;	
			rda::Vizualizer v6;
			rda::Vizualizer v7;
			rda::Vizualizer v8;

			rda::Vizualizer v9;

			v.createWindow("raw", 700, 700, 20, 20);		
			v2.createWindow("minimized_ramer", 700, 700, 640, 20);
			v3.createWindow("approximied_rumer", 700, 700, 640, 20);
			v4.createWindow("merged", 700, 700, 20, 20);
			v5.createWindow("distances", 700, 700, 20, 20);
			v6.createWindow("filtered", 700, 700, 640, 20);
			v7.createWindow("euclidean_segmentation", 700, 700, 640, 20);
			v8.createWindow("monotone_segmentation", 700, 700, 640, 20);		


			v5.addCloud(dist_cloud, rda::LINE_STRIP, 1.0, 0.0, 1.0, 1.0);			
			v5.addCloud(dist_cloud, rda::POINTS, 1.0, 1.0, 1.0, 1.0, 1.0f);
			v5.addCloud(ls_dist_cloud, rda::POINTS, 0.0, 1.0, 0.0, 1.0);
			
			v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);					
						
			v2.addClouds(min_clusters_patrs, rda::LINE_STRIP, 1.0f, 15.0f);
			v2.addCloud(cloud_filtered, rda::CIRCLES, 0.4f, 0.4f, 0.4f, 0.5f);	

			v3.addCloud(cloud, rda::CIRCLES, 0.3f, 0.3f, 0.3f, 0.3f);	
			for(int i = 0; i < approximied_parts.size(); i++)
				v3.addClouds(approximied_parts.at(i), rda::LINES, 1.0f, 3.0f);

			v4.addCloud(cloud, rda::CIRCLES, 0.3f, 0.3f, 0.3f, 0.3f);		
			for(int i = 0; i < merged_cluster.size(); i++)
				v4.addClouds(merged_cluster.at(i), rda::LINES, 1.0f);

			v5.addCloud(dist_cloud, rda::POINTS, 0.0f, 0.0f, 0.0f, 1.0f, 4.0f);	
			v5.addCloud(filtered_dist_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);	

			v6.addCloud(cloud_filtered, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.5f);	

			v7.addClouds(clusters, rda::CIRCLES, 1.0f);

			v8.addClouds(clusters_patrs, rda::CIRCLES, 0.5f);
			v8.addClouds(clusters_patrs, rda::LINE_STRIP, 0.5f);		

			rda::Vizualizer::start();	
			
		#pragma endregion 

		}
		catch(rda::RdaException& e){

			std::cout << e.what() << std:: endl;
		}
	}
};

#endif