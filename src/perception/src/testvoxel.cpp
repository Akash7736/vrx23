#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pyproj/pyproj.h>
#include <cmath>

class FrameListener : public rclcpp::Node {
public:
    FrameListener() : Node("Licamfuser") {
        xp = 0.0;
        yp = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        clocktime = 0;
        heightthresh = 1;

        cam_matrix = (cv::Mat_<double>(3, 3) << 762.7223205566406, 0.0, 640.0,
                                                 0.0, 762.7223110198975, 360.0,
                                                 0.0, 0.0, 1.0);
        K = (cv::Mat_<double>(3, 3) << 762.7223205566406, 0.0, 640.0,
                                       0.0, 762.7223110198975, 360.0,
                                       0.0, 0.0, 1.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        filpublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
        subscriptionvoxel_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/myvoxel", 10, std::bind(&FrameListener::voxel_callback, this, std::placeholders::_1));
        subscriptiongps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/wamv/sensors/gps/gps/fix", 10, std::bind(&FrameListener::gps_callback, this, std::placeholders::_1));
        subscriptionimu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/wamv/sensors/imu/imu/data", 10, std::bind(&FrameListener::imu_callback, this, std::placeholders::_1));
        subscriptionimg_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/wamv/sensors/cameras/far_left_camera_sensor/image_raw", 10, std::bind(&FrameListener::image_callback, this, std::placeholders::_1));
    }

private:
    double xp, yp, roll, pitch, yaw;
    int clocktime, heightthresh;
    cv::Mat cam_matrix, K;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filpublisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriptionvoxel_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriptiongps_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriptionimu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionimg_;
    cv_bridge::CvImage cv_image;

    void gps2enu(double origin_lat, double origin_long, double goal_lat, double goal_long) {
        pyproj::Geod geodesic("WGS84");
        double azimuth, back_azimuth, distance;
        geodesic.inv(origin_long, origin_lat, goal_long, goal_lat, azimuth, back_azimuth, distance);
        azimuth = azimuth * M_PI / 180.0;

        double y = std::cos(azimuth) * distance;
        double x = std::sin(azimuth) * distance;
        xp = x;
        yp = y;
    }

    std::vector<double> transform_point(geometry_msgs::msg::TransformStamped transformation,
                                         geometry_msgs::msg::Point point_wrt_source) {
        geometry_msgs::msg::PointStamped point_wrt_source_stamped;
        point_wrt_source_stamped.point = point_wrt_source;
        geometry_msgs::msg::PointStamped point_wrt_target;
        tf2::doTransform(point_wrt_source_stamped, point_wrt_target, transformation);
        return {point_wrt_target.point.x, point_wrt_target.point.y, point_wrt_target.point.z};
    }

    void voxel_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        clocktime = msg->header.stamp.sec;

        std::vector<std::vector<double>> pt_arr;
        std::vector<std::vector<double>> pointsincam;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            geometry_msgs::msg::Point point_wrt_lidar;
            point_wrt_lidar.x = *iter_x;
            point_wrt_lidar.y = *iter_y;
            point_wrt_lidar.z = *iter_z;

            geometry_msgs::msg::TransformStamped t1;
            try {
                t1 = tf_buffer_->lookupTransform("base_link", "lidar_wamv_link", rclcpp::Time(0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                return;
            }
            auto pt2 = transform_point(t1, point_wrt_lidar);

            geometry_msgs::msg::PointStamped point_wrt_base_stamped;
            point_wrt_base_stamped.point.x = pt2[0];
            point_wrt_base_stamped.point.y = pt2[1];
            point_wrt_base_stamped.point.z = pt2[2];
            point_wrt_base_stamped.header.frame_id = "base_link";

            geometry_msgs::msg::TransformStamped t2;
            try {
                t2 = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                return;
            }
            auto point_wrt_world = transform_point(t2, point_wrt_base_stamped.point);

            if (point_wrt_world[2] > heightthresh && point_wrt_world[2] < 1.5 && pt2[0] <
