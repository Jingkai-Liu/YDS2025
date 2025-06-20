// include/velodyne_distance/compute_distance.hpp
#ifndef VELODYNEDISTANCE_COMPUTE_DISTANCE_HPP
#define VELODYNEDISTANCE_COMPUTE_DISTANCE_HPP

#include <cstddef>  // for size_t

void compute_distance(const float* x, const float* y, float* r, std::size_t N);

#endif  // VELODYNEDISTANCE_COMPUTE_DISTANCE_HPP
