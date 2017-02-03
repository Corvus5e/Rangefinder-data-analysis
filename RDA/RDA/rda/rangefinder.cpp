
#include <list>
#include <iostream>
#include <map>

#include <rda\common.h>
#include <rda\rangefinder.h>
#include <rda\vector.h>
#include <rda\approximied_cloud_part.h>

namespace {

	double left_matrix[4][4] = {{ 0, -1, 0, -120 }, 
								{ 1, 0, 0, 241.0 },
								{ 0, 0, 1, 229.5 },
								{ 0, 0, 0, 1 }};

	double front_matrix[4][4] = {{ 1, 0, 0, 119 }, 
								 { 0, 1, 0, 0.0 },
								 { 0, 0, 1, 181.0 },
								 { 0, 0, 0, 1 }};
	
	double right_matrix[4][4] = {{ 0, 1, 0, -65.0 }, 
								 { -1, 1, 0, -223.0 },
								 { 0, 0, 1, 265.0 },
								 { 0, 0, 0, 1 }};

	double maxDistError(double distance, std::vector<std::pair<double, double>> errors)
	{
		if(distance < errors.front().first)
			return errors.front().second;
	
		if(distance > errors.back().first)
			return errors.back().second;
	
		for(auto i = 0; i < errors.size() - 1; i++){
			if(distance >= errors[i].first && distance <= errors[i+1].first)
				return errors[i].second + (distance - errors[i].first) * (errors[i+1].second - errors[i].second) / (errors[i+1].first - errors[i].first);
		}
	}
}

void rda::computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, rda::CloudPtr cloud, int sensor)
{
	for(int i = 0; i < distances.size(); i++){
		cloud->push_back(rda::computePoint(rob_point[i], distances[i], sensor));
	}	
}

void rda::computePointCloud(std::vector<rda::RPoint> rob_point, std::vector<double> distances, std::vector<int>& indexes, rda::CloudPtr cloud, int sensor)
{
	for(int i = 0; i < indexes.size(); i++){
		cloud->push_back(rda::computePoint(rob_point[indexes[i]], distances[indexes[i]], sensor));
	}	
}

rda::Point rda::computePoint(RPoint& rob_point, double distance, int sensor_id)
{	
		const int size = 4;

		double sensor_vec[4] = {distance, 0.0, 0.0, 1.0 };
		std::vector<double> sensor_vector(&sensor_vec[0], &sensor_vec[size]);
		// count coordinates in robot cordinates from sensor coordinates
		std::vector<double> robot_vector; 
		switch(sensor_id){		
			case FrontSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(front_matrix, sensor_vector); break;
			case LeftSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(left_matrix, sensor_vector); break;
			case RightSensor: 
				robot_vector = rda::mulMatrixOnVector<size>(right_matrix, sensor_vector); break;
		}
		
		double robot_matrix[size][size] = 
		{{ std::cos(rob_point.angle_robot * pi/180.0), -std::sin(rob_point.angle_robot * pi/180.0), 0, rob_point.x_robot }, 
		 { std::sin(rob_point.angle_robot * pi/180.0), std::cos(rob_point.angle_robot * pi/180.0), 0,  rob_point.y_robot },
		 { 0, 0, 1, 0 },
		 { 0, 0, 0, 1 }};

		// count coordinates in global cordinates from robot coordinates
		std::vector<double> gcs_vector = rda::mulMatrixOnVector<size>(robot_matrix, robot_vector);

		return rda::Point(gcs_vector[0], gcs_vector[1], 0.0/*gcs_vector[2]*/);
}

void rda::naive_beakpoint_detector(std::vector<double>& distances, double max_diff, int min_points, std::vector<rda::Range>& indexes)
{
	int last = 0;	

	for(auto i = 0; i < distances.size() - 1; i++){		
		if( std::abs( distances[i] - distances[i+1]) > max_diff ){
			if( (i - last + 1) >= min_points)
				indexes.push_back(rda::Range(last, i));
			last = i + 1;
		}
	}

	if(distances.size() - last >= min_points) // close last segment
		indexes.push_back(rda::Range(last, distances.size() - 1));
}

void rda::naive_beakpoint_detector(std::vector<double>& distances, std::vector<int>& v_indexes, double max_diff, int min_points, std::vector<std::vector<int>>& indexes)
{
	std::vector<int> tmp;
	
	for(auto i = 0; i < v_indexes.size() - 1; i++){

		tmp.push_back(v_indexes[i]);

		if( std::abs( distances[v_indexes[i]] - distances[v_indexes[i+1]]) > max_diff ){
			if( tmp.size() >= min_points)
				indexes.push_back(tmp);	
			tmp.clear();
		}		
	}

	tmp.push_back(v_indexes.back());
	if(tmp.size() >= min_points)
		indexes.push_back(tmp);	
}

void rda::adaptiveNaiveDetector(std::vector<double>& distances, std::vector<std::pair<double, double>>& errors, int min_points, std::vector<rda::Range>& indexes)
{
	int last = 0;	
	std::cout << "Breakpoints:" << std::endl;
	for(auto i = 0; i < distances.size() - 1; i++){		
		if( std::abs( distances[i] - distances[i+1]) > maxDistError(std::min(distances[i],distances[i+1]) , errors) ){
			if( (i - last + 1) >= min_points){
				indexes.push_back(rda::Range(last, i));
				std::cout << distances[i] << " " << distances[i+1] << " error ";
				std::cout << std::abs( distances[i] - distances[i+1]) << " maxError " << maxDistError(std::min(distances[i],distances[i+1]) , errors) << std::endl;
			}
			last = i + 1;
		}
	}

	if(distances.size() - last >= min_points) // close last segment
		indexes.push_back(rda::Range(last, distances.size() - 1));
}