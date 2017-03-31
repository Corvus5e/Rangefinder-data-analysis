#ifndef RDA_DISTERRORRESEARCH_H
#define RDA_DISTERRORRESEARCH_H


#include <scenario\Scenario.h>
#include <rda\common.h>
#include <rda\function\LeastSquares.h>
#include <rda\io\io.h>
#include <rda\io\vizualization.h>
#include <rda\io\console.h>

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

class DistErrorResearch : public Scanario {
private:	
	double maxDeviationFromEndLine(rda::CloudPart& cloud, function::LeastSquares& error_function, int& index)
	{
		index = 0;

		double max_dev = 0;
		double curr_dev(max_dev);
		double dist_hyp = 0;
		rda::Line line_hyp(cloud.first_point(), cloud.last_point());				
		double max_diff = 0;

		std::vector<double> devs;
		for(auto i = cloud.begin(); i <= cloud.end(); i++){
			dist_hyp = line_hyp.k() * cloud.at(i).x + line_hyp.b();
			curr_dev = std::abs(cloud.at(i).y - dist_hyp) / (error_function.value(dist_hyp));	
			devs.push_back(curr_dev);
			if(curr_dev > max_dev){
				max_dev = curr_dev;
				index = i;
				max_diff = std::abs(cloud.at(i).y - dist_hyp);
			}
		}
		std::cout << "error: " << max_diff << std::endl;
		return  max_dev;		
	}

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

	void rdp(rda::CloudPart& cloud, double error, function::LeastSquares& error_function, int min_segm_size, std::vector<int>& indexes)
	{
		int mid_index = 0;
		double max_dev = maxDeviationFromEndLine(cloud, error_function, mid_index);

		//system("cls");
		std::cout << "Max dev: " << max_dev << std::endl;		
		//std::cin.get();	

		if(max_dev > error){
			std::vector<int> indexes_left;
			std::vector<int> indexes_right;

			rdp(rda::CloudPart(cloud.cloud(), rda::Range(cloud.begin(), mid_index)), error, error_function, min_segm_size, indexes_left);
			rdp(rda::CloudPart(cloud.cloud(), rda::Range(mid_index, cloud.end())), error, error_function, min_segm_size, indexes_right);

			indexes.resize(indexes_left.size() + indexes_right.size());
			std::copy(indexes_left.begin(), indexes_left.end(), indexes.begin());
			std::copy(indexes_right.begin(), indexes_right.end(), indexes.begin() + indexes_left.size());
		}
		else{
			indexes.push_back(cloud.begin());
			indexes.push_back(cloud.end());
		}
	}

	void rdp2d(rda::CloudPart& cloud, std::vector<double>& distances, double error, function::LeastSquares& error_function, int min_segm_size, std::vector<int>& indexes)
	{
		int mid_index = 0;
		double max_dev = maxDeviationFromEndLine2d(cloud, distances, error_function, mid_index);			

		if(max_dev > error){
			std::vector<int> indexes_left;
			std::vector<int> indexes_right;

			rdp2d(rda::CloudPart(cloud.cloud(), rda::Range(cloud.begin(), mid_index)), distances, error, error_function, min_segm_size, indexes_left);
			rdp2d(rda::CloudPart(cloud.cloud(), rda::Range(mid_index, cloud.end())), distances, error, error_function, min_segm_size, indexes_right);

			indexes.resize(indexes_left.size() + indexes_right.size());
			std::copy(indexes_left.begin(), indexes_left.end(), indexes.begin());
			std::copy(indexes_right.begin(), indexes_right.end(), indexes.begin() + indexes_left.size());
		}
		else{
			indexes.push_back(cloud.begin());
			indexes.push_back(cloud.end());
		}
	}

	double dl(rda::CloudPart cloud, double min_error, int min_size, std::vector<int>& indexes, function::LeastSquares& error_function)
	{
		int mid_index;
		double error = std::max(maxDeviationFromEndLine(cloud, error_function, mid_index), 1.0);
		double significance = rda::distancePointToPoint(cloud.first_point(), cloud.last_point()) / error;
		
		if(cloud.range().size() >= min_size){
			if(error > min_error){
				std::vector<int> left_indexes;
				std::vector<int> right_indexes;
				double left_significance = dl(rda::CloudPart(cloud.cloud(),rda::Range(cloud.range().start, mid_index)), min_error, min_size, left_indexes, error_function);
				double right_significance = dl(rda::CloudPart(cloud.cloud(), rda::Range(mid_index, cloud.range().end)), min_error, min_size, right_indexes, error_function);

				if(significance < left_significance || significance < right_significance){
					indexes.insert(indexes.end(), left_indexes.begin(), left_indexes.end());
					indexes.insert(indexes.end(), right_indexes.begin(), right_indexes.end());
					return std::max(left_significance, right_significance);
				}			
			}
		}
		else{		
			significance = -1;
		}

		indexes.push_back(cloud.begin());
		indexes.push_back(cloud.end());

		return significance;
	}	

	void filter(rda::CloudPart cloud, int window_size, function::LeastSquares& error_function, double error, std::vector<int>& indexes)
	{
		int w = window_size / 2;		

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

public:
	void start(int argc, char* argv[])
	{										

		rda::CloudPtr raw_dist_cloud(new rda::Cloud);
		rda::CloudPtr raw_cloud(new rda::Cloud);
		rda::CloudPtr rdp_dist_cloud(new rda::Cloud);		
		rda::CloudPtr error_cloud(new rda::Cloud);
		rda::CloudPtr approx_error_cloud(new rda::Cloud);
				

		rda::Console args;
		args.readArgs(argc, argv);

		// Scan data
		std::vector<double> raw_distances;
		std::vector<rda::Range> raw_part_ranges;
		raw_cloud = rda::readScene(args.getParam("-raw_file"), raw_distances, raw_part_ranges);
		
		for(auto i = 0; i < raw_distances.size(); i++){
			raw_dist_cloud->push_back(rda::Point(i, raw_distances[i], 1.0f));
		}

		readErrorFile(args.getParam("-error_file"), error_cloud);
		function::LeastSquares ls(8);
		ls.init(error_cloud, 0, error_cloud->size() - 1);
		ls.approximate();
		for(auto i = error_cloud->front().x - 135; i < error_cloud->back().x + 135; i++){
			approx_error_cloud->push_back(rda::Point(i, ls.value(i), 1.0f));
		}

		int filter_window = atof(args.getParam("-filter_window").c_str());
		double filter_error = atof(args.getParam("-filter_error").c_str());	
		int min_segm_points = atof(args.getParam("-min_segm_points").c_str());
		double breakpoint_error = atof(args.getParam("-breakpoint_error").c_str());

		std::vector<std::vector<rda::Range>> filtered_ranges(raw_part_ranges.size());
		std::vector<std::vector<int>> indexes(raw_part_ranges.size());
		std::vector<rda::CloudPtr> dist_filtered_cloud;
		std::vector<std::vector<double>> dists_filtered((raw_part_ranges.size()));
		std::vector<rda::CloudPtr> filtered_clouds;

		for(auto i = 0; i < raw_part_ranges.size(); i++){
			adaptiveBreakpointDetector(rda::CloudPart(raw_dist_cloud, raw_part_ranges[i]), ls, breakpoint_error, min_segm_points, filtered_ranges[i]);
			//filter(rda::CloudPart(raw_dist_cloud, raw_part_ranges[i]), filter_window, ls, filter_error, indexes[i]);			
			dist_filtered_cloud.push_back(rda::CloudPtr(new rda::Cloud));
			filtered_clouds.push_back(rda::CloudPtr(new rda::Cloud));
			for(auto it = filtered_ranges[i].begin(); it != filtered_ranges[i].end(); ++it){
				for(auto jt = it->start; jt <= it->end; jt++){
					dist_filtered_cloud.back()->push_back(raw_dist_cloud->at(jt));
					dists_filtered[i].push_back(raw_distances[jt]);
					filtered_clouds.back()->push_back(raw_cloud->at(jt));
				}
												
			}
		}

		double rdp_error = atof(args.getParam("-rdp_error").c_str());
		readErrorFile(args.getParam("-error_file"), error_cloud);

		std::vector<std::vector<int>> rdp_indexes(filtered_ranges.size());
		std::vector<rda::CloudPtr> rdp_clouds;

		//rdp(rda::CloudPart(raw_dist_cloud), rdp_error, ls, 1, indexes);
		for(auto i = 0; i < filtered_clouds.size(); i++){
			if(filtered_clouds[i]->size() > 0){
				rdp2d(rda::CloudPart(filtered_clouds[i]), dists_filtered[i], rdp_error, ls, 1, rdp_indexes[i]);
				rdp_clouds.push_back(rda::CloudPtr(new rda::Cloud));
				for(auto j = 0; j < rdp_indexes[i].size(); j++){
					//rdp_dist_cloud->push_back(raw_dist_cloud->at(rdp_indexes[j]));
					rdp_clouds.back()->push_back(filtered_clouds[i]->at(rdp_indexes[i][j]));
				}
			}
		}



		//Vizualization;

		rda::Vizualizer::init(&argc, argv);			
		rda::Vizualizer v_1;	
		rda::Vizualizer v_2;	
		rda::Vizualizer v_3;
		rda::Vizualizer v_4;		
		v_1.createWindow("standart deviation", 720, 720, 722, 2);				
		v_3.createWindow("raw distances", 720, 720, 2, 2);
		v_4.createWindow("raw cloud 2d", 720, 720, 722, 2);

		v_1.addCloud(error_cloud, rda::CIRCLES, 0.0f, 0.0f, 1.0f, 1.0f);
		v_1.addCloud(approx_error_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 1.0f, 1.0f);		

		v_3.addCloud(raw_dist_cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 1.0f);
		v_3.addCloud(raw_dist_cloud, rda::LINE_STRIP, 1.0f, 1.0f, 1.0f, 0.4f);		
		v_3.addClouds(dist_filtered_cloud, rda::CIRCLES, 1.0f);
		v_3.addClouds(dist_filtered_cloud, rda::LINE_STRIP, 1.0f);
		
		//v_3.addCloud(rdp_dist_cloud, rda::LINE_STRIP, 1.0f, 1.0f, 0.0f, 1.0f);
		//v_3.addCloud(rdp_dist_cloud, rda::CIRCLES, 1.0f, 0.0f, 0.0f, 1.0f);		

		v_4.addCloud(raw_cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 1.0f);
		for(auto i = 0; i < filtered_clouds.size(); i++){
			v_4.addCloud(filtered_clouds[i], rda::CIRCLES, 0.0f, 0.0f, 1.0f, 1.0f);
		}
		for(auto i = 0; i < rdp_clouds.size(); i++){
			v_4.addCloud(rdp_clouds[i], rda::LINE_STRIP, 1.0f, 1.0f, 0.0f, 1.0f);
			v_4.addCloud(rdp_clouds[i], rda::CIRCLES, 1.0f, 0.0f, 0.0f, 1.0f);
		}

		rda::Vizualizer::start();

	}
};

#endif