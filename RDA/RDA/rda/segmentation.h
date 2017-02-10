
#ifndef EUCLIDEAN_H
#define EUCLIDEAN_H

#include <vector>

#include <rda\common.h>

namespace rda {

	void euclideanClusterExctraction(rda::CloudPtr cloud, std::vector<rda::CloudPtr>& clusters, double tolerance, int minSize, int maxSize);
}

#endif