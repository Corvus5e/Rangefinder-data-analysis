
#ifndef ASM_2F_H
#define ASM_2F_H

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

#include <time.h>

#include <rda\common.h>
#include <rda\filtering.h>
#include <rda\rangefinder.h>
#include <rda\curve.h>
#include <rda\rda_exception.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>

#include <scenario\Scenario.h>

class BASM : public Scanario {

public: 

	void start(int argc, char* argv[]){
		try{
			std::vector<double> distances;
			std::vector<rda::RPoint> rob_points;
			std::vector<rda::Range> part_ranges;
	
			rda::CloudPtr raw_cloud(new rda::Cloud);
			std::vector<rda::CloudPtr> reduce_median_clouds;
			
			rda::Console console;			
			console.readArgs(argc, argv);
	
			int sensor_id = 0;
			rda::readScene(console.getParam("-file"), distances, rob_points, part_ranges, sensor_id);	
	
			int statistacal_kN = atof(console.getParam("-statistacal_kN").c_str());
			double statistacal_threashold = atof(console.getParam("-statistacal_threashold").c_str());
			int min_segm_points = atof(console.getParam("-min_segm_points").c_str());
			double max_dist_diff = atof(console.getParam("-max_dist_diff").c_str());
			int reduce_median_window = atof(console.getParam("-reduce_median_window").c_str());
			double min_rdp_eps = atof(console.getParam("-min_rdp_eps").c_str());
			int min_rdp_size = atof(console.getParam("-min_rdp_size").c_str());

			clock_t partitioning_clock;
			partitioning_clock = clock();
	
			rda::computePointCloud(rob_points, distances, raw_cloud, sensor_id);
	
			//statistical filter
			std::vector<std::vector<int>> stat_filtered_indexes(part_ranges.size());		
			for(auto i = 0; i < part_ranges.size(); i++){			
				rda::statisticalDistanceFilter(distances, part_ranges[i], statistacal_kN, statistacal_threashold, stat_filtered_indexes[i]);			
			}				
	
	
			// naive breakponit detector
			std::vector<std::vector<int>> break_indexes;
			for(auto i = 0; i < stat_filtered_indexes.size(); i++){
				//rda::naive_beakpoint_detector(distances, stat_filtered_indexes[i], max_dist_diff, min_segm_points, break_indexes);  			
				rda::naiveBreakpointDetector(raw_cloud, stat_filtered_indexes[i], max_dist_diff, min_segm_points, break_indexes);
			}
	
			// reduce median filter	
			std::vector<std::vector<int>> reduce_median_indexes(break_indexes.size());		
			for(auto i = 0; i < break_indexes.size(); i++){
				if(break_indexes[i].size() < reduce_median_window){
					reduce_median_indexes[i] = break_indexes[i];
				}
				rda::reduceMedianFilter(distances, break_indexes[i], reduce_median_window, reduce_median_indexes[i]);						
	
				// count filtered into world coordinate system)
				rda::CloudPtr rm_cloud(new rda::Cloud);
				rda::computePointCloud(rob_points, distances, reduce_median_indexes[i], rm_cloud, sensor_id);
				reduce_median_clouds.push_back(rm_cloud);
			}
	
			// Adaptive Split&Merge
			std::vector<std::vector<rda::CloudPart>> min_cloud_parts;
			for(auto it = reduce_median_clouds.begin(); it != reduce_median_clouds.end(); ++it){
				std::vector<rda::CloudPart> min_parts;
				rda::adaptiveRdp(rda::CloudPart(*it), min_rdp_size, min_rdp_eps, min_parts);
				min_cloud_parts.push_back(min_parts);
			}
	
			//Least Squares
			std::vector<std::vector<rda::ApproximiedCloudPart>> ls_cloud_parts(min_cloud_parts.size());
			std::vector<rda::Line> ls_lines;
			for(auto i = 0; i < min_cloud_parts.size(); i++){
				rda::lsLineApproximation(min_cloud_parts[i], ls_cloud_parts[i]);
				for(auto j = 0; j < ls_cloud_parts[i].size(); j++){
					ls_lines.push_back(ls_cloud_parts[i][j].approx_line());
				}
			}
	
			std::cout << "Wasted time :" <<  ((float)(clock() - partitioning_clock)) / CLOCKS_PER_SEC << "sec" << std::endl;
	
			#pragma region Vizualization
		
			std::cout << "Input points number: " << raw_cloud->size() << std::endl;
	
			//dists clouds
			std::vector<rda::CloudPtr> dists_clouds;
			int dists_number = 0;
			for(auto it = part_ranges.begin(); it != part_ranges.end(); ++it){
				dists_clouds.push_back(rda::CloudPtr(new rda::Cloud));
				for(auto i = it->start; i <= it->end; i++){
					dists_clouds.back()->push_back(rda::Point(dists_number++, distances[i], 1));
				}
			}
	
			// statistical filtering dist cloud and 2d cloud
			int stat_number = 0;
			std::vector<rda::CloudPtr> sf_dists_clouds(stat_filtered_indexes.size());
			std::vector<rda::CloudPtr> sf_clouds(stat_filtered_indexes.size());
			for(auto i = 0; i < stat_filtered_indexes.size(); i++){
				sf_dists_clouds[i] = rda::CloudPtr(new rda::Cloud);
				sf_clouds[i] = rda::CloudPtr(new rda::Cloud);
				stat_number += stat_filtered_indexes[i].size();
				for(auto j = 0; j < stat_filtered_indexes[i].size(); j++){
					sf_dists_clouds[i]->push_back(rda::Point(stat_filtered_indexes[i][j], distances[stat_filtered_indexes[i][j]], 1));
					sf_clouds[i]->push_back(rda::computePoint(rob_points[stat_filtered_indexes[i][j]], distances[stat_filtered_indexes[i][j]], sensor_id));
				}
			}
			std::cout << "Points after statistical filter: " << stat_number << std::endl;
		
			// naive breakpoint detector
			int naive_number = 0;
			std::vector<rda::CloudPtr> break_clouds(break_indexes.size());
			for(auto i = 0 ; i < break_indexes.size(); i++){			
				break_clouds[i] = rda::CloudPtr(new rda::Cloud);
				naive_number += break_indexes[i].size();
				for(auto j = 0; j < break_indexes[i].size(); j++){				
					break_clouds[i]->push_back(rda::computePoint(rob_points[break_indexes[i][j]], distances[break_indexes[i][j]], sensor_id));
				}
			}
			std::cout << "Points after naive: " << naive_number << std::endl;
	
			//reduce median filtered dists clouds
			int filtered_dist_number = 0;
			std::vector<rda::CloudPtr> filtered_dists_clouds;
			for(auto it = reduce_median_indexes.begin(); it != reduce_median_indexes.end(); ++it){
				filtered_dists_clouds.push_back(rda::CloudPtr(new rda::Cloud));
				filtered_dist_number += it->size();
				for(auto i = it->begin(); i != it->end(); ++i){
					filtered_dists_clouds.back()->push_back(rda::Point(*i, distances[*i], 1));
				}
			}
			std::cout << "Points after reduce median filter: " << filtered_dist_number << std::endl;
	
			std::vector<rda::Line> min_cloud_parts_lines;
			for(auto it = min_cloud_parts.begin(); it != min_cloud_parts.end(); ++it){
				for(auto jt = it->begin(); jt != it->end(); ++jt){
					min_cloud_parts_lines.push_back(jt->line());
				}
			}			
			std::cout << "Points after approximation: " << ls_lines.size() * 2 << std::endl; 
	
			// Vizualizer
			rda::Vizualizer::init(&argc, argv);	
			rda::Vizualizer v;
			rda::Vizualizer v_1;
			rda::Vizualizer v_2;
			rda::Vizualizer v_3;
			rda::Vizualizer v_4;
			rda::Vizualizer v_5;
			rda::Vizualizer v_6;
			rda::Vizualizer v_7;
			rda::Vizualizer v_8;
	
			v.createWindow("distances", 700, 700, 20, 20);
			v_1.createWindow("raw", 700, 700, 720, 20);
			v_4.createWindow("statistical filter", 700, 700, 20, 20);
			v_7.createWindow("statistical distances", 700, 700, 20, 20);
			v_5.createWindow("naive breakpoint detector", 700, 700, 20, 20);
			v_2.createWindow("reduce median filter", 700, 700, 720, 20);
			v_8.createWindow("reduce median filter distances", 700, 700, 20, 20);
			v_3.createWindow("split&merge", 700, 700, 720, 20);		
			v_6.createWindow("least squares", 700, 700, 720, 20);
	
			
			for(auto i = 0; i < dists_clouds.size(); i++){
				//v.addCloud(dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
				v.addCloud(dists_clouds[i], rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f);			
				v.addCloud(dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);			
			}
	
	
			for(auto i = 0; i < sf_dists_clouds.size(); i++){
				//v_7.addCloud(sf_dists_clouds[i], rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f);			
				v_7.addCloud(sf_dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);			
				//v_8.addCloud(sf_dists_clouds[i], rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f);
				//v.addCloud(sf_dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);	
			}
	
			// dists sum test
			//v.addCloud(sum_dist_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f);
			//v.addCloud(sum_dist_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			//v.addCloud(sum_dist_av_cloud, rda::LINES, 0.0f, 0.0f, 0.0f, 1.0f);
			//v.addCloud(sum_dist_std_cloud, rda::LINES, 0.0f, 0.0f, 1.0f, 1.0f);
			
			v_1.addCloud(raw_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			//v_2.addCloud(raw_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			//v_4.addCloud(raw_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			
			
			for(auto i = 0; i < sf_clouds.size(); i++){
				v_4.addCloud(sf_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
				//v_2.addCloud(sf_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 1.0f, 1.0f);			
			}
			
			for(auto i = 0; i < filtered_dists_clouds.size(); i++){
				v_8.addCloud(filtered_dists_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
				//v_8.addCloud(filtered_dists_clouds[i], rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f);			
			}
	
			for(auto i = 0; i < reduce_median_clouds.size(); i++){
				v_2.addCloud(reduce_median_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
				v_3.addCloud(reduce_median_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.1f);
			}
	
			for(auto i = 0; i < min_cloud_parts_lines.size(); i++){
				v_3.addCloud(min_cloud_parts_lines[i], rda::LINES, 0.0f, 0.0f, 0.0f, 1.0f, 5.0f);		
			}
	
			for(auto i = 0; i < break_clouds.size(); i++){
				v_5.addCloud(break_clouds[i], rda::LINE_STRIP, 0.0f, 0.0f, 0.0f, 1.0f);
				v_5.addCloud(break_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 0.0f, 1.0f);
			}
	
			//v_5.addClouds(break_clouds, rda::CIRCLES, 1.0f);
	
			v_6.addCloud(raw_cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.1f);
			for(auto i = 0; i < ls_lines.size(); i++){			
				v_6.addCloud(ls_lines[i], rda::LINES, 0.0f, 0.0f, 0.0f, 1.0f, 5.0f);
			}
	
			rda::Vizualizer::start();
	
			#pragma endregion
		}
		catch(rda::RdaException& e){
			std::cout << e.what() << std::endl;
		}
	}
};

#endif