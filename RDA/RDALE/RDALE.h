
#ifndef RDALE_H
#define RDALE_H


extern "C" {
		
	__declspec(dllexport) void __cdecl extractLines(double* input,			 // 
													std::string& error_file, //
													double rdp_eps,			 // 3
													int filter_window_size,  // 11 
													double filter_error,	 // 1
													int min_segment_points,  // 5 
													double breakpoint_error, // 3
													double merge_distance,
													double merge_angle,
													double**& output,		 //
													int& clusters_size);	 //

		// Returns the number of elements needed to store point in array
	__declspec(dllexport) int __cdecl pointSize();	

	__declspec(dllexport) void __cdecl clearMemory(double**& ptr, int size);
}

#endif