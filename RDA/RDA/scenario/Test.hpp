
#ifndef RDA_TEST_H
#define RDA_TEST_H

#include <scenario\Scenario.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>

#include <rda\curve.h>

class Test : public Scanario{
public:

	void start(int argc, char* argv[])
	{
		rda::Console console;
		console.readArgs(argc, argv);	

		std::vector<double> distances;		
		std::vector<rda::Range> part_ranges;
		rda::CloudPtr cloud (new rda::Cloud);
		std::vector<std::vector<rda::CloudPart>> lines_clusters;

		cloud = rda::readScene(console.getParam("-file"), distances, part_ranges);

		//Params		
		double min_rdp_eps = atof(console.getParam("-min_rdp_eps").c_str());
		int min_rdp_size = atof(console.getParam("-min_rdp_size").c_str());
		
		std::vector<rda::CloudPart> cloud_parts;
		for(auto i = 0; i < part_ranges.size(); i++){
			cloud_parts.push_back(rda::CloudPart(cloud, rda::Range(part_ranges[i])));			
		}				

		std::vector<rda::CloudPart> minimized_parts;
		rda::adaptiveLineSegmentation(cloud_parts, min_rdp_size, min_rdp_eps, minimized_parts);

		std::vector<rda::Line> min_cloud_parts_lines;
		for(auto i = minimized_parts.begin(); i != minimized_parts.end(); ++i){
			min_cloud_parts_lines.push_back(i->line());
		}
		// Vizualization

		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		v.createWindow("adaptiveRDP new test", 720, 720, 2, 2);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);		
		v.addClouds(min_cloud_parts_lines, rda::LINES, 1.0f);

		rda::Vizualizer::start();	
	}
};

#endif