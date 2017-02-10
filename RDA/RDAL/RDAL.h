
#ifndef RDAL_H
#define RDAL_H

	/*
	 * Protocol :
	 *  
	 * input { SIZE, x, y, *, *, distance, x, y, *, *, distance, ....} (* - is any number(not used))
	 * 
	*/

extern "C" {
	
	__declspec(dllexport) void __cdecl casmExtractor(double* input, double clustering_eps, int clustering_minPts, double min_rdp_eps, double max_dist, int min_part_size, double merge_dist, double merge_angle, int filter_kN, double filter_treshold, double**& output, int& clusters_size);

	// Returns the number of elements needed to store point in array
	__declspec(dllexport) int __cdecl pointSize();

	//__declspec(dllexport) void __cdecl clearMemory(double*& ptr);

	__declspec(dllexport) void __cdecl clearMemory(double**& ptr, int size);

}

#endif