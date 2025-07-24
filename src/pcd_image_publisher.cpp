#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>               // for pcl::toPCLPointCloud2
#include <pcl_conversions/pcl_conversions.h> // for pcl_conversions::fromPCL
#include <pcl/common/io.h>                // for pcl::toROSMsg
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class PcdImagePublisher : public rclcpp::Node {
public:
  PcdImagePublisher()
  : Node("pcd_image_publisher")
  {
    // 参数
    auto pcd_path  = declare_parameter<std::string>("pcd_path", "test.pcd");
    auto img_path  = declare_parameter<std::string>("image_path", "test.jpg");
    auto pcd_frame = declare_parameter<std::string>("pcd_frame", "lidar");
    auto img_frame = declare_parameter<std::string>("image_frame", "camera");

    // 1. 读取 PCD 并转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(get_logger(), "无法读取 PCD 文件: %s", pcd_path.c_str());
      rclcpp::shutdown();
      return;
    }
    // 转成 PointCloud2 消息
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, pcl_pc2_msg_);
    pcl_pc2_msg_.header.frame_id = pcd_frame;

    // 2. 读取图像并转换
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty()) {
      RCLCPP_ERROR(get_logger(), "无法读取图像文件: %s", img_path.c_str());
      rclcpp::shutdown();
      return;
    }
    cv_bridge::CvImage cv_img;
    cv_img.header.frame_id = img_frame;
    cv_img.encoding        = "bgr8";
    cv_img.image           = img;
    img_msg_ = *cv_img.toImageMsg();

    // 3. 创建发布器
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points_raw", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10);

    // 4. 定时器：10Hz 发布
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PcdImagePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto now = this->get_clock()->now();
    // 更新时间戳
    pcl_pc2_msg_.header.stamp = now;
    img_msg_.header.stamp     = now;
    // **打印 frame_id 到控制台**
    RCLCPP_INFO(this->get_logger(),
                "Publishing PointCloud frame_id: '%s', Image frame_id: '%s'",
                pcl_pc2_msg_.header.frame_id.c_str(),
                img_msg_.header.frame_id.c_str());

    // 发布
    cloud_pub_->publish(pcl_pc2_msg_);
    image_pub_->publish(img_msg_);
  }

  // 成员变量
  sensor_msgs::msg::PointCloud2 pcl_pc2_msg_;
  sensor_msgs::msg::Image img_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
