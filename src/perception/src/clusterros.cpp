#include <memory>
#include <iostream>
#include <filesystem>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
typedef pcl::PointXYZ PointT;

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
      

      pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud2);
      pcl::PCLPointCloud2 cloud_filtered2;
      pcl::PointCloud<PointT>::Ptr voxel_cloud  (new pcl::PointCloud<PointT>);



      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2);


    // pcl::PointCloud<PointT>::Ptr cloud        (new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr voxel_cloud  (new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr  coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr       inliers      (new pcl::PointIndices);
    pcl::PCDReader               cloud_reader;

    // Normal Extraction Objects
    pcl::ModelCoefficients::Ptr       cylinder_co    (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr            cylinder_in    (new pcl::PointIndices);
    pcl::search::KdTree<PointT>::Ptr  tree           (new pcl::search::KdTree<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals  (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr      cylinder_cloud (new pcl::PointCloud<PointT> ());

    // Normals computation objects
    pcl::NormalEstimation<PointT,pcl::Normal>            normals_estimator;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
    pcl::ExtractIndices<PointT>                          cylinder_indices_extractor;
    pcl::ExtractIndices<pcl::Normal>                     cylinder_indices_extractor_temp;


      pcl::fromROSMsg(*msgPtr, *cloud);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.0, 50);

      pass.filter(*cloud_filtered);

    //   pcl::toROSMsg(*cloud_filtered, cloudmsg);
    //   pcl_conversions::toPCL(cloudmsg, *cloud2);
    
    //   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        pcl::VoxelGrid<PointT> sor;

      
      sor.setInputCloud(cloud_filtered);
     
      sor.setLeafSize(0.1, 0.1, 0.1);
      
      sor.filter(*voxel_cloud);

          // Performing estimation of normals
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(voxel_cloud);
    normals_estimator.setKSearch(90);
    normals_estimator.compute(*cloud_normals);


    // Parameters for segmentation
    cylinder_segmentor.setOptimizeCoefficients(true);
	cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
	cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
	cylinder_segmentor.setNormalDistanceWeight(0.5);
	cylinder_segmentor.setMaxIterations(10000);
	cylinder_segmentor.setDistanceThreshold(0.05);
	cylinder_segmentor.setRadiusLimits(0.1, 0.4);
    int l=0;


        // Appplying segmentation
    cylinder_segmentor.setInputCloud(voxel_cloud);
	cylinder_segmentor.setInputNormals(cloud_normals);
	cylinder_segmentor.segment(*cylinder_in,*cylinder_co);

    // extracting indices
    cylinder_indices_extractor.setInputCloud(voxel_cloud);
    cylinder_indices_extractor.setIndices(cylinder_in);
    cylinder_indices_extractor.setNegative(false);
    cylinder_indices_extractor.filter(*cylinder_cloud);
    //*********************************************************************8
    // pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2::Ptr pc2_cloud_ptr (new pcl::PCLPointCloud2);
    // pcl::PointCloudpcl::PointXYZ::Ptr xyzrgb_cloud_ptr (new pcl::PointCloudpcl::PointXYZ);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new pcl::PointCloud<pcl::PointXYZ>);

        if(!cylinder_cloud->points.empty()){
        // std::stringstream ss ;ss<< "ex_cylinder_"<<l<<".pcd";
        // std::cout<<"Cloud Contains " <<cylinder_cloud->points.size()<<std::endl;
        if(cylinder_cloud->points.size() > 2){
            pcl::toROSMsg(*cylinder_cloud, cloudmsg);
            pcl::toPCLPointCloud2(*cylinder_cloud, *pc2_cloud_ptr);
            pcl_conversions::fromPCL(*pc2_cloud_ptr, output);
            // cloud_saver(ss.str(),path,cylinder_cloud);
            publisher_->publish(output);

            l++;
        }}

        cylinder_indices_extractor.setNegative(true);
        cylinder_indices_extractor.filter(*voxel_cloud);


        // processing normals
        cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
        cylinder_indices_extractor_temp.setIndices(cylinder_in);
        cylinder_indices_extractor_temp.setNegative(true);
        cylinder_indices_extractor_temp.filter(*cloud_normals);
//*********************************************************************************
    //   pcl_conversions::fromPCL(cloud_filtered2, output);
    //   publisher_->publish(output);

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


