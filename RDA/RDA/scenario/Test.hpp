
#ifndef RDA_TEST_H
#define RDA_TEST_H

#include <scenario\Scenario.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>

#include <rda\curve.h>
#include <time.h>

class Test : public Scanario{
public:

	void start(int argc, char* argv[])
	{
		rda::Console console;
		console.readArgs(argc, argv);	

		std::vector<double> distances;		
		std::vector<rda::Range> part_ranges;
		rda::CloudPtr cloud (new rda::Cloud);		
		std::vector<rda::Line> lines;
		std::vector<rda::CloudPart> rdp_clouds;
		std::vector<rda::Line> rdp_lines;

		std::vector<rda::CloudPart> nrdp_clouds;
		std::vector<rda::Line> nrdp_lines;

		cloud = rda::readScene(console.getParam("-file"), distances, part_ranges);
		int points_increment = atof(console.getParam("-points_increment").c_str());
		int stable_angles = atof(console.getParam("-stable_angles").c_str());
		double angle_threashold = atof(console.getParam("-angle_threashold").c_str());

		double min_error = atof(console.getParam("-min_error").c_str());
		int min_size = atof(console.getParam("-min_size").c_str());

		std::vector<rda::CloudPtr> angles_clouds; 
		std::vector<rda::CloudPtr> derivative_clouds; 
		
		clock_t partitioning_clock;
		std::cout << "Points :" <<  cloud->size() << std::endl;	
		partitioning_clock = clock();
		
		for(auto i = 0; i < part_ranges.size(); i++){						
			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, rdp_clouds);
			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, nrdp_clouds, rda::stDevSignificanceEstimator);
		}

		for(auto i = 0; i < rdp_clouds.size(); i++){
			rdp_lines.push_back(rdp_clouds[i].line());
		}

		for(auto i = 0; i < nrdp_clouds.size(); i++){
			nrdp_lines.push_back(nrdp_clouds[i].line());
		}


		double time = ((float)(clock() - partitioning_clock)) / CLOCKS_PER_SEC;
		std::cout << "Wasted time :" <<  time << "sec" << std::endl;					

		// Vizualization
		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		rda::Vizualizer v_1;	
		v.createWindow("new rdp test", 720, 720, 2, 2);
		v_1.createWindow("adaptive rdp", 720, 720, 720, 2);

		v_1.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);
		v_1.addClouds(rdp_lines, rda::LINES, 1.0f);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);		
		v.addClouds(nrdp_lines, rda::LINES, 1.0f);


		rda::Vizualizer::start();	
	}
};

#endif