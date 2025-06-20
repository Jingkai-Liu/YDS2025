#include "velodyne_distance/compute_distance.hpp"
#include <arm_neon.h>
#include <cmath>

void compute_distance(const float* x, const float* y, float* r, size_t N) {
  size_t i = 0;
  for (; i + 3 < N; i += 4) {
    float32x4_t x_vec = vld1q_f32(x + i);
    float32x4_t y_vec = vld1q_f32(y + i);
    float32x4_t x2 = vmulq_f32(x_vec, x_vec);
    float32x4_t y2 = vmulq_f32(y_vec, y_vec);
    float32x4_t sum = vaddq_f32(x2, y2);
    float32x4_t r_vec = vsqrtq_f32(sum);
    vst1q_f32(r + i, r_vec);
  }
  for (; i < N; ++i) {
    r[i] = std::sqrt(x[i] * x[i] + y[i] * y[i]);
  }
}
