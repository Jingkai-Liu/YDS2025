#include "velodyne_distance/compute_distance.hpp"

#include <cmath>

void compute_distance(const float* x, const float* y, float* r, std::size_t N) {
  for (std::size_t i = 0; i < N; ++i) {
    r[i] = std::sqrt(x[i] * x[i] + y[i] * y[i]);
  }
}
