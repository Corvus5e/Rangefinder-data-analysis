
#ifndef RDAL_H
#define RDAL_H

	/*
	 * Protocol :
	 *  
	 * input { SIZE, x, y, *, *, distance, x, y, *, *, distance, ....} (* - is any number(not used))
	 * 
	*/

extern "C" {
	
	__declspec(dllexport) void __cdecl extractLines(double* input,			// 
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

	// Returns the number of elements needed to store point in array
	__declspec(dllexport) int __cdecl pointSize();

	//__declspec(dllexport) void __cdecl clearMemory(double*& ptr);

	__declspec(dllexport) void __cdecl clearMemory(double**& ptr, int size);

}

#endif