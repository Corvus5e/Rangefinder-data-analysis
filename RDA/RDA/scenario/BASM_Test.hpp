
#ifndef BASM_TEST_H
#define BASM_TEST_H


#include <scenario\Scenario.h>

#include <rda\io\io.h>
#include <rda\io\console.h>
#include <rda\io\vizualization.h>

#include <rda\line_extraction.h>

class BASM_Test : public Scanario{
public:

	void start(int argc, char* argv[])
	{
		rda::Console console;
		console.readArgs(argc, argv);	

		std::vector<double> distances;		
		std::vector<rda::Range> part_ranges;
		rda::CloudPtr cloud (new rda::Cloud);
		std::vector<std::vector<rda::ApproximiedCloudPart>> lines_clusters;

		cloud = rda::readScene(console.getParam("-file"), distances, part_ranges);

		//Params		
		int statistacal_kN = atof(console.getParam("-statistacal_kN").c_str());
		double statistacal_threashold = atof(console.getParam("-statistacal_threashold").c_str());
		int min_segm_points = atof(console.getParam("-min_segm_points").c_str());
		double max_dist_diff = atof(console.getParam("-max_dist_diff").c_str());
		int reduce_median_window = atof(console.getParam("-reduce_median_window").c_str());
		double min_rdp_eps = atof(console.getParam("-min_rdp_eps").c_str());
		int min_rdp_size = atof(console.getParam("-min_rdp_size").c_str());

		rda::basmExtractor(cloud, distances, part_ranges, statistacal_kN, statistacal_threashold,
							min_segm_points, max_dist_diff, reduce_median_window, min_rdp_eps,
							min_rdp_size, lines_clusters);


		// Vizualization


		rda::Vizualizer::init(&argc, argv);	
		rda::Vizualizer v;	
		v.createWindow("basmExtractor", 720, 720, 2, 2);
		v.addCloud(cloud, rda::CIRCLES, 0.0f, 0.0f, 0.0f, 0.3f);
		for(int i = 0; i < lines_clusters.size(); i++)
			v.addClouds(lines_clusters.at(i), rda::LINES, 1.0f);
		
		rda::Vizualizer::start();	
	}
};

#endif