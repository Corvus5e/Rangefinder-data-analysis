
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

		std::vector<rda::CloudPart> mrdp_clouds;
		std::vector<rda::Line> mrdp_lines;

		std::vector<rda::CloudPart> srdp_clouds;
		std::vector<rda::Line> srdp_lines;

		std::vector<rda::CloudPart> sdrdp_clouds;
		std::vector<rda::Line> sdrdp_lines;

		cloud = rda::readScene(console.getParam("-file"), distances, part_ranges);

		double min_error = atof(console.getParam("-min_rdp_eps").c_str());
		int min_size = atof(console.getParam("-min_rdp_size").c_str());

		std::vector<rda::CloudPtr> angles_clouds; 
		std::vector<rda::CloudPtr> derivative_clouds; 
		
		clock_t partitioning_clock;
		std::cout << "Points :" <<  cloud->size() << std::endl;	
		partitioning_clock = clock();
		
		for(auto i = 0; i < part_ranges.size(); i++){						
			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, rdp_clouds);
			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, mrdp_clouds, rda::meanSignificanceEstimator);
			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, srdp_clouds, rda::stDevSignificanceEstimator);

			rda::adaptiveRDP(rda::CloudPart(cloud, part_ranges[i]), min_error, min_size, sdrdp_clouds, rda::stDevDirSignificanceEstimator);

		}

		for(auto i = 0; i < rdp_clouds.size(); i++){
			rdp_lines.push_back(rdp_clouds[i].line());
		}

		for(auto i = 0; i < mrdp_clouds.size(); i++){
			mrdp_lines.push_back(mrdp_clouds[i].line());
		}

		for(auto i = 0; i < srdp_clouds.size(); i++){
			srdp_lines.push_back(srdp_clouds[i].line());
		}

		for(auto i = 0; i < sdrdp_clouds.size(); i++){
			sdrdp_lines.push_back(sdrdp_clouds[i].line());
		}
		double time = ((float)(clock() - partitioning_clock)) / CLOCKS_PER_SEC;
		std::cout << "Wasted time :" <<  time << "sec" << std::endl;					

		// Vizualization
		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		rda::Vizualizer v_1;	
		rda::Vizualizer v_2;	
		rda::Vizualizer v_3;
		v.createWindow("mean rdp test", 720, 720, 2, 2);
		v_1.createWindow("adaptive rdp", 720, 720, 720, 2);
		v_2.createWindow("stddev rdp", 720, 720, 720, 2);
		v_3.createWindow("stddev dir rdp", 720, 720, 720, 2);

		v_1.addCloud(cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.3f);
		v_1.addClouds(rdp_lines, rda::LINES, 1.0f);
		v.addCloud(cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.3f);		
		v.addClouds(mrdp_lines, rda::LINES, 1.0f);
		v_2.addCloud(cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.3f);
		v_2.addClouds(srdp_lines, rda::LINES, 1.0f);
		v_3.addCloud(cloud, rda::CIRCLES, 1.0f, 1.0f, 1.0f, 0.3f);
		v_3.addClouds(sdrdp_lines, rda::LINES, 1.0f);


		rda::Vizualizer::start();	
	}
};

#endif