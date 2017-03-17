#ifndef RDA_DISTERRORRESEARCH_H
#define RDA_DISTERRORRESEARCH_H


#include <scenario\Scenario.h>
#include <rda\common.h>
#include <rda\io\io.h>
#include <rda\io\vizualization.h>

#include <string>
#include <vector>

class DistErrorResearch : public Scanario {
public:
	void start(int argc, char* argv[])
	{
		std::string folder(argv[1]);
		std::vector<std::vector<double>> distances(argc - 2);
		rda::CloudPtr known_dist_cloud(new rda::Cloud);
		std::vector<rda::CloudPtr> clouds;
		std::vector<rda::CloudPtr> dist_clouds;
		rda::CloudPtr st_dev_cloud(new rda::Cloud);
		rda::CloudPtr mean_cloud(new rda::Cloud);
		rda::CloudPtr medians_cloud(new rda::Cloud);
		rda::CloudPtr error_cloud(new rda::Cloud);
		std::vector<std::vector<rda::Range>> part_ranges(argc - 2);

		const int start_dist = 25;
		const int dist_step = 35;
		const double measurement_diff = 17;		

		for(auto i = 2; i < argc; i++){
			std::string file = "D:\\git\\Rangefinder_Data_Analysis\\scans\\dist_error_measurements\\" + std::string(argv[i]);
			clouds.push_back(rda::readScene(file, distances[i-2], part_ranges[i-2]));
			known_dist_cloud->push_back(rda::Point(start_dist + (i-2)*dist_step, 10*atof(argv[i]) + measurement_diff, 1.0f));
		}

		std::vector<double> st_devs;
		std::vector<double> means(distances.size());
		std::vector<double> medians;
		
		std::cout << "known dist	|	 mean	|	median" << std::endl;
		for(auto i = 0; i < distances.size(); i++){
			st_devs.push_back(3 * rda::standartDeviation(distances[i].begin(), distances[i].end(), means[i]));
			medians.push_back(rda::medianValue<double>(distances[i].begin(), distances[i].end()));
			std::cout <<  10*(start_dist + i*5) + measurement_diff << " | " << means[i] << " | " << medians[i] << std::endl;
		}


		for(auto i = 0; i < known_dist_cloud->size(); i++){
			/*error_cloud->push_back(rda::Point(start_dist + i*dist_step, known_dist_cloud->at(i).y + st_devs[i], 1.0f));
			error_cloud->push_back(rda::Point(start_dist + i*dist_step, known_dist_cloud->at(i).y - st_devs[i], 1.0f));*/
			mean_cloud->push_back(rda::Point(start_dist + i*dist_step, means[i], 1.0f));
			medians_cloud->push_back(rda::Point(start_dist + i*dist_step, medians[i], 1.0f));
			error_cloud->push_back(rda::Point(start_dist + i*dist_step, means[i] + st_devs[i], 1.0f));
			error_cloud->push_back(rda::Point(start_dist + i*dist_step, means[i] - st_devs[i], 1.0f));
		}

		for(auto i = 0; i < distances.size(); i++){
			rda::CloudPtr c(new rda::Cloud);			
			for(auto j = 0; j < distances[i].size(); j++){
				c->push_back(rda::Point(start_dist + i*dist_step, distances[i][j], 1.0f));
			}
			dist_clouds.push_back(c);
			double mean = 0;
			st_dev_cloud->push_back(rda::Point(start_dist + i*dist_step, st_devs[i], 1.0f)); 
		}

		//Vizualization;

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		rda::Vizualizer v_1;	
		rda::Vizualizer v_2;	
		v.createWindow("raw", 720, 720, 2, 2);		
		v_1.createWindow("standart deviation", 720, 720, 722, 2);		
		v_2.createWindow("measurements and 3 standart deviations", 720, 720, 2, 2);

		for(int i = 0; i < dist_clouds.size(); i++){
			v.addCloud(dist_clouds[i], rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.1f);
			v_2.addCloud(dist_clouds[i], rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.1f);
		}
		v.addCloud(mean_cloud, rda::POINTS, 0.0f, 1.0f, 0.0f, 1.0f, 5.0f);
		v.addCloud(mean_cloud, rda::LINE_STRIP, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f);
		v.addCloud(medians_cloud, rda::LINE_STRIP, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
		v.addCloud(known_dist_cloud, rda::POINTS, 1.0f, 0.0f, 0.0f, 1.0f, 5.0f);
		v.addCloud(known_dist_cloud, rda::LINE_STRIP, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f);
		v_1.addCloud(st_dev_cloud, rda::CIRCLES, 0.0f, 0.0f, 1.0f, 1.0f);
		v_2.addCloud(error_cloud, rda::LINES, 1.0f, 0.0f, 0.0f, 1.0f);

		rda::Vizualizer::start();

	}
};

#endif