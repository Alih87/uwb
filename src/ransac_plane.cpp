#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std::chrono_literals;

class PlaneRANSAC : public rclcpp::Node
{
public:
  PlaneRANSAC() : Node("ransac_plane_node")
  {
    this->declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");

    this->declare_parameter("ransac_distance_threshold", 0.03);
    this->declare_parameter("ransac_max_iterations", 100);
    this->declare_parameter("min_inliers", 500);

    // Pixel ROI ratios: bottom-center region
    this->declare_parameter("roi_u_min_ratio", 0.20);
    this->declare_parameter("roi_u_max_ratio", 0.80);
    this->declare_parameter("roi_v_min_ratio", 0.55);
    this->declare_parameter("roi_v_max_ratio", 0.95);

    // Depth limits in meters
    this->declare_parameter("z_min", 0.3);
    this->declare_parameter("z_max", 6.0);

    // Downsample pixels for speed: 1 = every pixel, 2 = every second pixel, etc.
    this->declare_parameter("pixel_step", 2);

    depth_topic_ = this->get_parameter("depth_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();

    distance_threshold_ = this->get_parameter("ransac_distance_threshold").as_double();
    max_iterations_ = this->get_parameter("ransac_max_iterations").as_int();
    min_inliers_ = this->get_parameter("min_inliers").as_int();

    roi_u_min_ratio_ = this->get_parameter("roi_u_min_ratio").as_double();
    roi_u_max_ratio_ = this->get_parameter("roi_u_max_ratio").as_double();
    roi_v_min_ratio_ = this->get_parameter("roi_v_min_ratio").as_double();
    roi_v_max_ratio_ = this->get_parameter("roi_v_max_ratio").as_double();

    z_min_ = this->get_parameter("z_min").as_double();
    z_max_ = this->get_parameter("z_max").as_double();

    pixel_step_ = this->get_parameter("pixel_step").as_int();
    if (pixel_step_ < 1) {
      pixel_step_ = 1;
    }

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PlaneRANSAC::depthCallback, this, std::placeholders::_1)
    );

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      10,
      std::bind(&PlaneRANSAC::cameraInfoCallback, this, std::placeholders::_1)
    );

    coeff_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ground_plane/coefficients", 10);

    counts_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/ground_plane/counts", 10);

    roi_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ground_plane/roi_cloud", 10);

    inlier_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ground_plane/inlier_cloud", 10);

    outlier_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ground_plane/outlier_cloud", 10);

    normal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/ground_plane/normal", 10);

    centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/ground_plane/centroid", 10);

    roi_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/ground_plane/pixel_roi", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/ground_plane/normal_marker", 10);

    timer_ = this->create_wall_timer(
      200ms,
      std::bind(&PlaneRANSAC::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Pixel-ROI RANSAC plane node started.");
    RCLCPP_INFO(this->get_logger(), "Depth topic      : %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "CameraInfo topic : %s", camera_info_topic_.c_str());
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    camera_frame_id_ = msg->header.frame_id;
    has_camera_info_ = true;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_depth_msg_ = msg;
    has_depth_ = true;
  }

  float getDepthMeters(const cv::Mat & depth_img, int u, int v, const std::string & encoding)
  {
    if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        encoding == "16UC1") {
      uint16_t depth_mm = depth_img.at<uint16_t>(v, u);

      if (depth_mm == 0) {
        return std::numeric_limits<float>::quiet_NaN();
      }

      return static_cast<float>(depth_mm) * 0.001f;
    }

    if (encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
        encoding == "32FC1") {
      float depth_m = depth_img.at<float>(v, u);

      if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
      }

      return depth_m;
    }

    return std::numeric_limits<float>::quiet_NaN();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildCloudFromPixelROI(
    const sensor_msgs::msg::Image::SharedPtr & depth_msg,
    int & u_min,
    int & u_max,
    int & v_min,
    int & v_max)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    }
    catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return nullptr;
    }

    const cv::Mat & depth_img = cv_ptr->image;

    int width = depth_msg->width;
    int height = depth_msg->height;

    u_min = static_cast<int>(width * roi_u_min_ratio_);
    u_max = static_cast<int>(width * roi_u_max_ratio_);
    v_min = static_cast<int>(height * roi_v_min_ratio_);
    v_max = static_cast<int>(height * roi_v_max_ratio_);

    u_min = std::max(0, std::min(u_min, width - 1));
    u_max = std::max(0, std::min(u_max, width));
    v_min = std::max(0, std::min(v_min, height - 1));
    v_max = std::max(0, std::min(v_max, height));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->header.frame_id = depth_msg->header.frame_id;
    cloud->is_dense = false;

    for (int v = v_min; v < v_max; v += pixel_step_) {
      for (int u = u_min; u < u_max; u += pixel_step_) {
        float Z = getDepthMeters(depth_img, u, v, depth_msg->encoding);

        if (!std::isfinite(Z)) {
          continue;
        }

        if (Z < z_min_ || Z > z_max_) {
          continue;
        }

        float X = static_cast<float>((u - cx_) * Z / fx_);
        float Y = static_cast<float>((v - cy_) * Z / fy_);

        if (!std::isfinite(X) || !std::isfinite(Y)) {
          continue;
        }

        cloud->points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
  }

  void timerCallback()
  {
    sensor_msgs::msg::Image::SharedPtr depth_msg;

    double fx, fy, cx, cy;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!has_depth_) {
        return;
      }

      if (!has_camera_info_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Waiting for camera_info..."
        );
        return;
      }

      depth_msg = latest_depth_msg_;

      fx = fx_;
      fy = fy_;
      cx = cx_;
      cy = cy_;
    }

    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Invalid camera intrinsics."
      );
      return;
    }

    int u_min = 0;
    int u_max = 0;
    int v_min = 0;
    int v_max = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      buildCloudFromPixelROI(depth_msg, u_min, u_max, v_min, v_max);

    if (!cloud) {
      return;
    }

    publishPixelROI(depth_msg->header, u_min, u_max, v_min, v_max);

    publishCloud(
      cloud,
      depth_msg->header,
      roi_cloud_pub_
    );

    if (cloud->points.size() < static_cast<size_t>(min_inliers_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Not enough valid ROI points: %zu",
        cloud->points.size()
      );
      return;
    }

    runRANSAC(depth_msg->header, cloud);
  }

  void runRANSAC(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(distance_threshold_);
    seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "RANSAC failed: no plane inliers found."
      );
      return;
    }

    if (inliers->indices.size() < static_cast<size_t>(min_inliers_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Plane found but too few inliers: %zu",
        inliers->indices.size()
      );
      return;
    }

    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];

    double norm = std::sqrt(a * a + b * b + c * c);

    if (norm < 1e-6) {
      RCLCPP_WARN(this->get_logger(), "Invalid plane normal.");
      return;
    }

    a /= norm;
    b /= norm;
    c /= norm;
    d /= norm;

    publishPlaneData(header, cloud, inliers, a, b, c, d);
  }

  void publishPixelROI(
    const std_msgs::msg::Header &,
    int u_min,
    int u_max,
    int v_min,
    int v_max)
  {
    std_msgs::msg::Int32MultiArray msg;

    // [u_min, u_max, v_min, v_max]
    msg.data = {u_min, u_max, v_min, v_max};

    roi_pub_->publish(msg);
  }

  void publishCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(*cloud, msg);
    msg.header = header;

    pub->publish(msg);
  }

  void publishPlaneData(
    const std_msgs::msg::Header & header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointIndices::Ptr & inliers,
    double a,
    double b,
    double c,
    double d)
  {
    std_msgs::msg::Float64MultiArray coeff_msg;

    // Plane equation:
    // ax + by + cz + d = 0
    coeff_msg.data = {a, b, c, d};

    coeff_pub_->publish(coeff_msg);

    std_msgs::msg::Int32MultiArray counts_msg;

    // [total_roi_points, inlier_points, outlier_points]
    counts_msg.data = {
      static_cast<int32_t>(cloud->points.size()),
      static_cast<int32_t>(inliers->indices.size()),
      static_cast<int32_t>(cloud->points.size() - inliers->indices.size())
    };

    counts_pub_->publish(counts_msg);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    extract.setNegative(false);
    extract.filter(*inlier_cloud);

    extract.setNegative(true);
    extract.filter(*outlier_cloud);

    publishCloud(inlier_cloud, header, inlier_cloud_pub_);
    publishCloud(outlier_cloud, header, outlier_cloud_pub_);

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double centroid_z = 0.0;

    for (const auto & p : inlier_cloud->points) {
      centroid_x += p.x;
      centroid_y += p.y;
      centroid_z += p.z;
    }

    centroid_x /= static_cast<double>(inlier_cloud->points.size());
    centroid_y /= static_cast<double>(inlier_cloud->points.size());
    centroid_z /= static_cast<double>(inlier_cloud->points.size());

    geometry_msgs::msg::Vector3Stamped normal_msg;

    normal_msg.header = header;
    normal_msg.vector.x = a;
    normal_msg.vector.y = b;
    normal_msg.vector.z = c;

    normal_pub_->publish(normal_msg);

    geometry_msgs::msg::PointStamped centroid_msg;

    centroid_msg.header = header;
    centroid_msg.point.x = centroid_x;
    centroid_msg.point.y = centroid_y;
    centroid_msg.point.z = centroid_z;

    centroid_pub_->publish(centroid_msg);

    publishNormalMarker(header, centroid_x, centroid_y, centroid_z, a, b, c);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Plane: %.4fx + %.4fy + %.4fz + %.4f = 0 | inliers: %zu / %zu",
      a, b, c, d,
      inliers->indices.size(),
      cloud->points.size()
    );
  }

  void publishNormalMarker(
    const std_msgs::msg::Header & header,
    double cx,
    double cy,
    double cz,
    double nx,
    double ny,
    double nz)
  {
    visualization_msgs::msg::Marker marker;

    marker.header = header;
    marker.ns = "ground_plane";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start;
    start.x = cx;
    start.y = cy;
    start.z = cz;

    geometry_msgs::msg::Point end;
    end.x = cx + nx * 0.5;
    end.y = cy + ny * 0.5;
    end.z = cz + nz * 0.5;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.03;
    marker.scale.y = 0.06;
    marker.scale.z = 0.10;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_pub_->publish(marker);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr coeff_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr counts_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr roi_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr outlier_cloud_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr normal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  sensor_msgs::msg::Image::SharedPtr latest_depth_msg_;

  bool has_depth_{false};
  bool has_camera_info_{false};

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_id_;

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};

  double distance_threshold_{0.03};
  int max_iterations_{100};
  int min_inliers_{500};

  double roi_u_min_ratio_{0.20};
  double roi_u_max_ratio_{0.80};
  double roi_v_min_ratio_{0.55};
  double roi_v_max_ratio_{0.95};

  double z_min_{0.3};
  double z_max_{6.0};

  int pixel_step_{2};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PlaneRANSAC>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
