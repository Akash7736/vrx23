#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/filters/passthrough.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <functional>

using std::placeholders::_1;
using namespace std::chrono_literals;


class LidarSubscriber : public rclcpp::Node
{

  public:
    LidarSubscriber()
    : Node("lidar_subscriber")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/myvoxel", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10, std::bind(&LidarSubscriber::lidar_callback, this, _1));
    }


  private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msgPtr) const
    {
  // Container for Output data
      sensor_msgs::msg::PointCloud2 output;
      sensor_msgs::msg::PointCloud2 cloudmsg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);  //stores filtered
      
      // RCLCPP_INFO(this->get_logger(), "Data Received : '%d'", msg.data[2]);

      pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud2);
      pcl::PCLPointCloud2 cloud_filtered2;
      



      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2);
      // pcl_conversions::toPCL(*msgPtr, *cloudPtr);

      pcl::fromROSMsg(*msgPtr, *cloud);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(-15, 15);

      pass.setFilterFieldName("y");
      pass.setFilterLimits(-15, 15);

      // pass.setFilterFieldName("z");
      // pass.setFilterLimits(-1, 2);


      pass.filter(*cloud_filtered);

      pcl::toROSMsg(*cloud_filtered, cloudmsg);
      pcl_conversions::toPCL(cloudmsg, *cloud2);
    
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   
      // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_filtered);
      
      sor.setInputCloud(cloudPtr2);
      // sor.setInputCloud(*cloud_filtered);
      sor.setLeafSize(0.01, 0.01, 0.01);
      
      sor.filter(cloud_filtered2);
      // sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_filtered2, output);
      publisher_->publish(output);
      // pub.publish(output);

      // pcl::toROSMsg(*cloud_filtered, output);

    //   printf(msg.header.frame_id);
    //   RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg->data.c_str()); )
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}


// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <pcl/filters/passthrough.h>


// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>

// using std::placeholders::_1;


// class LidarSubscriber : public rclcpp::Node
// {
//   public:
//     LidarSubscriber()
//     : Node("lidar_subscriber")
//     {
//       subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//       "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10, std::bind(&LidarSubscriber::lidar_callback, this, _1));
//     }

//   private:
//     void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "Data Received : '%d'", msg.data[2]);

//     //   printf(msg.header.frame_id);
//     //   RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg->data.c_str()); )
//     }
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<LidarSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }
