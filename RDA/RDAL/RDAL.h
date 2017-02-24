
#ifndef RDAL_H
#define RDAL_H

	/*
	 * Protocol :
	 *  
	 * input { SIZE, x, y, *, *, distance, x, y, *, *, distance, ....} (* - is any number(not used))
	 * 
	*/

extern "C" {
	
	
	__declspec(dllexport) void __cdecl casmLineExtractor(double* input,			// 
														double clustering_eps,  //80
														int clustering_minPts,  //3
														double min_rdp_eps,		//30
														double max_dist,		//100
														int min_part_size,		//8
														double merge_dist,		// 50 (if not used -> -10)
														double merge_angle,		// 20
														int filter_kN,			// 5
														double filter_treshold,	// 0.9 (if model data -> 5.0)
														double**& output,		//
														int& clusters_size);	// 


	__declspec(dllexport) void __cdecl basmLineExtractor(double* input,					// 
														int statistical_kn,				//
														double statistical_threashold,	//
														int min_segm_points,			//
														double max_dist_diff,			//
														int rmed_window_size,			//
														int min_rdp_eps,				//
														int min_rdp_size,				//
														double**& output,				//
														int& clusters_size);			// 

	/*
	 * Ramer-Douglas-Peucker		 
	*/
	__declspec(dllexport) void __cdecl rdpMinimization(double* input, double threshold, double**& output, int& clusters_size);

	/*	 
	 * Least Squares 1st order
	*/	
	__declspec(dllexport) void __cdecl lsLineApproximation(double* input, double**& output, int& clusters_size);		

	/*	 
	 * Least Squares function approximied by RDP
	*/
	__declspec(dllexport) void __cdecl lsRDPApproximation(double* input, int order, double step, double threashold, double**& output, int& clusters_size);		

	__declspec(dllexport) void __cdecl statisticalDistanceFilter(double* input, int neighbours_number, double threshold, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl statisticalFilter(double* input, int neighbours_number, double threshold, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl reduceMedianFilter(double* input, int window_size, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl naiveBreakpointDetector(double* input, double max_diff, int min_points, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl euclideanClusterExctraction(double* input, double eps, int min_points,int max_points, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl adaptiveRDP(double* input, double min_error, int min_size, double**& output, int& clusters_size);

	__declspec(dllexport) void __cdecl adaptiveRDPStD(double* input, double min_error, int min_size, double**& output, int& clusters_size);

	// Returns the number of elements needed to store point in array
	__declspec(dllexport) int __cdecl pointSize();	

	__declspec(dllexport) void __cdecl clearMemory(double**& ptr, int size);

}

#endif