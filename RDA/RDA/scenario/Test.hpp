
#ifndef RDA_TEST_H
#define RDA_TEST_H

#include <scenario\Scenario.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>

#include <rda\line_extraction.h>

class Test : public Scanario {
public:
	void start(int argc, char* argv[]){

		rda::Console console;
		console.readArgs(argc, argv);	

		std::vector<double> distances;			
		rda::CloudPtr cloud (new rda::Cloud);
		std::vector<std::vector<rda::ApproximiedCloudPart>> lines_clusters;

		cloud = rda::readScene(console.getParam("-file"), distances);

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

		rda::casmExtractor(cloud, distances, eps, minPts, max_dist,
							min_rdp_eps, min_part_size, merge_dist,
							merge_angle, kN, threshold, lines_clusters);


		// Vizualization


		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		v.createWindow("casmExtractor", 720, 720, 2, 2);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);
		for(int i = 0; i < lines_clusters.size(); i++)
			v.addClouds(lines_clusters.at(i), rda::LINES, 1.0f);
		
		rda::Vizualizer::start();	
	}
};

#endif