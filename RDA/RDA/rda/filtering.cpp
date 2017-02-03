
#include <list>
#include <algorithm>

#include <pcl\filters\statistical_outlier_removal.h>

#include <rda\filtering.h>
#include <rda\common.h>

void rda::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ> &filtered_cloud, int k, double thr )
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(k);
	sor.setStddevMulThresh(thr);
	sor.filter(filtered_cloud);	
}

void rda::medianFilter(std::vector<double>& values, int wsize, std::vector<double>& output)
{
	output.resize(values.size());

	std::vector<double> buf(wsize);
	int n = values.size(); // to int 
	for(int i = 0; i < values.size(); i++){
		for(int j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = values.front();
			if(k >= n)
				buf[j] = values.back();
			if( k >= 0 && k < values.size())
				buf[j] = values[k];
		}
		std::sort(buf.begin(), buf.end());
		output[i] = buf[wsize/2];
	}
}

void rda::reduceMedianFilter(std::vector<double>& values, rda::Range bounds, int wsize, std::vector<int>& indexes)
{	
	if(bounds.size() < wsize){
		wsize = bounds.size();
	}

	std::vector<double> buf(wsize);
	int n = values.size(); // to int 

	for(auto i = bounds.start; i <= bounds.end; i++){
		for(auto j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = values[bounds.start];
			if(k >= n)
				buf[j] = values[bounds.end];
			if( k >= 0 && k < values.size())
				buf[j] = values[k];
		}

		std::sort(buf.begin(), buf.end());
		if(values[i] == buf[wsize/2])
			indexes.push_back(i);		
	}		
}

void rda::reduceMedianFilter(std::vector<double>& values, std::vector<int>& v_indexes, int wsize, std::vector<int>& indexes)
{
	std::vector<double> buf(wsize);
	int n = v_indexes.size(); // to int 

	for(auto i = 0; i < v_indexes.size(); i++){
		for(auto j = 0; j < buf.size(); j++){
			int k = i + j - wsize/2;
			//if(k < 0)
				buf[j] = values[v_indexes.front()];
			if(k >= n)
				buf[j] = values[v_indexes.back()];
			if( k >= 0 && k < n)
				buf[j] = values[v_indexes[k]];
		}

		std::sort(buf.begin(), buf.end());
		if(values[v_indexes[i]] == buf[wsize/2])
			indexes.push_back(v_indexes[i]);		
	}
}

void rda::kuwaharaFilter(const std::vector<double>& values, int w_size, std::vector<double>& output)
{
	output.resize(values.size());
	std::copy(values.begin(), values.end(), output.begin());

	if(values.size() > w_size){
		for(auto i = w_size; i < output.size() - w_size - 1; i++){
			double mean_left = 0.0;
			double mean_right = 0.0;
			double st_dev_left = rda::standartDeviation(&output[i-w_size], &output[i], mean_left);
			double st_dev_right = rda::standartDeviation(&output[i+1], &output[i+w_size+1], mean_right);			

			if(st_dev_left < st_dev_right)
				output[i] = mean_left;
			else
				output[i] = mean_right;
		}
	}		
}

void rda::statisticalDistanceFilter(std::vector<double>& distances, int k, double coef, std::vector<int>& indexes)
{
	rda::statisticalDistanceFilter(distances, rda::Range(0, distances.size() - 1), k, coef, indexes);
}

void rda::statisticalDistanceFilter(std::vector<double>& distances, rda::Range range, int k, double coef, std::vector<int>& indexes)
{
	std::vector<double> dists_sum; //values
	std::vector<int> dist_indexes; //values
 
	int half = k/2;

	// Begin
	for(int i = range.start; i < range.start + half ; i++){
		double sumDist = 0;
		for(int j = range.start; j < range.start + k ; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//Middle
	for(int i = range.start + half; i <= range.end - half; i++){
		double sumDist = 0;
		for(int j = i - half; j <= i + half; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}

		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//End
	for(int i = range.end - half + 1; i <= range.end ; i++){
		double sumDist = 0;
		for(int j = range.end - k + 1; j <= range.end; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	double av = 0;
	double st_d = rda::standartDeviation(dists_sum.begin(), dists_sum.end(), av);
	double threshold = st_d * coef;

	for(int i = 0; i < dists_sum.size(); i++){
		if(dists_sum[i] <= av + threshold)
			indexes.push_back(dist_indexes[i]);
	}
}

void rda::statisticalDistanceFilterDebug(std::vector<double>& distances, rda::Range range, int k, double coef, std::vector<int>& indexes, std::vector<double>& dists_sum)
{	
	std::vector<int> dist_indexes; //values
 
	int half = k/2;

	// Begin
	for(int i = range.start; i < range.start + half ; i++){
		double sumDist = 0;
		for(int j = range.start; j < range.start + k ; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//Middle
	for(int i = range.start + half; i <= range.end - half; i++){
		double sumDist = 0;
		for(int j = i - half; j <= i + half; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}

		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	//End
	for(int i = range.end - half + 1; i <= range.end ; i++){
		double sumDist = 0;
		for(int j = range.end - k + 1; j <= range.end; j++){
			if(i != j){
				sumDist += std::abs(distances[i] - distances[j]);
			}
		}
		dist_indexes.push_back(i);
		dists_sum.push_back(sumDist);
	}

	double av = 0;
	double st_d = rda::standartDeviation(dists_sum.begin(), dists_sum.end(), av);
	double threshold = st_d * coef;

	for(int i = 0; i < dists_sum.size(); i++){
		if(dists_sum[i] <= av + threshold)
			indexes.push_back(dist_indexes[i]);
	}
}