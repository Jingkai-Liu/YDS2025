#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include "velodyne_distance/compute_distance.hpp"


class DistanceNode : public rclcpp::Node {
public:
  DistanceNode() : Node("distance_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10,
      std::bind(&DistanceNode::cloud_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  size_t N = msg->width * msg->height;
  Eigen::Matrix<float, 3, Eigen::Dynamic> cloud(3, N);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  size_t count = 0;
  for (size_t i = 0; i < N; ++i, ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
    cloud(0, count) = *iter_x;
    cloud(1, count) = *iter_y;
    cloud(2, count) = *iter_z;
    count++;
  }
  cloud.conservativeResize(3, count);

  // 手动筛选符合条件的点，放入 std::vector
  std::vector<float> x_filtered_vec, y_filtered_vec;
  for (size_t i = 0; i < count; ++i) {
    float z_val = cloud(2, i);
    if (z_val > -0.5f && z_val < 1.5f) {
      x_filtered_vec.push_back(cloud(0, i));
      y_filtered_vec.push_back(cloud(1, i));
    }
  }

  size_t M = x_filtered_vec.size();
  if (M == 0) {
    RCLCPP_WARN(this->get_logger(), "No points passed the filter.");
    return;
  }

  // 调用 compute_distance
  std::vector<float> distances(M);
  compute_distance(x_filtered_vec.data(), y_filtered_vec.data(), distances.data(), M);

  // 计算平均距离
  float sum = 0.0f;
  for (auto d : distances) sum += d;
  float avg_dist = sum / M;

  RCLCPP_INFO(this->get_logger(), "[Distance] Avg filtered distance: %.2f m", avg_dist);
}

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceNode>());
  rclcpp::shutdown();
  return 0;
}
