#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "laser_geometry/laser_geometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

using std::placeholders::_1;

class ScanFilter : public rclcpp::Node
{
private:
    const std::string referece_frame = "r_sole";
    const char* scan_topic = "/scan";
    const char* pub_topic = "/filtered_pc2";
    laser_geometry::LaserProjection projector_;

    const float filter_z_low = 0.2;
    const float filter_z_high = 3.0;
    const float sensor_height = 1.5;
    const char* right_sole_frame = "r_sole";
    const char* left_sole_frame = "l_sole";

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_in;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in)
    {
        std::string transform_error;
        if (tf_buffer_in->canTransform(
            scan_in->header.frame_id,
            referece_frame,
            tf2_ros::fromMsg(scan_in->header.stamp) + tf2::durationFromSec(scan_in->ranges.size() * scan_in->time_increment),
            tf2::durationFromSec(0.5),
            & transform_error ))
        {
            // Converts the scans into cartesian space points
            sensor_msgs::msg::PointCloud2 cloud;
            projector_.transformLaserScanToPointCloud(referece_frame, *scan_in, cloud, *tf_buffer_in);
            // Filter cloud.
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
            // Pass data from Pointcloud2 to pcl
            pcl::fromROSMsg(cloud, *pcl_cloud);
            RCLCPP_INFO(get_logger(), "Original cloud size: %li \n", pcl_cloud->size());
            // FILTER 1 - PASSTHROUGH
            pcl::PassThrough<pcl::PointXYZ> pass;       // Create the passthrough filtering object
            pass.setInputCloud (pcl_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits(filter_z_low, filter_z_high);
            // Filtering
            pass.filter (*cloud_filtered1);
            RCLCPP_INFO(get_logger(), "Cloud size after filter 1: %li \n", cloud_filtered1->size());
            // FILTER 2 - PLANE PROJECTOR
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());   // Create a set of planar coefficients with X=Y=0,Z=1, d= 1.5
            coefficients->values.resize(4);
            coefficients->values[0] = coefficients->values[1] = 0;
            coefficients->values[2] = 1;
            coefficients->values[3] = 0;
            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud_filtered1);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_filtered2);
            RCLCPP_INFO(get_logger(), "Cloud size after filter 2: %li \n", cloud_filtered2->size());
            // Publish PC2
            sensor_msgs::msg::PointCloud2 ros_cloud;
            ros_cloud.header.frame_id = referece_frame;
            ros_cloud.header.stamp = now();
            pcl::toROSMsg(*cloud_filtered2, ros_cloud);
            pointcloud_pub->publish(ros_cloud);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not transform message: %s", transform_error.c_str());
        }
    };

public:
    ScanFilter() : Node("scan_filtering_node")
    {
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic, 10);
        tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_in);

        raw_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan> (
            scan_topic,
            10,
            std::bind(&ScanFilter::scan_callback, this, _1)
        );
    }
};  // End of class ScanFilter node


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<ScanFilter>();
        std::cout << "Starting up node. \n";
        rclcpp::spin(node);
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    return 0;
}

