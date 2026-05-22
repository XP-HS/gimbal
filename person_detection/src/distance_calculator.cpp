#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>
#include <tuple>

class DistanceCalculator {
private:
    ros::Subscriber cloud_sub_;
    ros::Subscriber detection_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher realtime_marker_pub_;
    ros::Publisher person_points_pub_;  // 发布人物位置给轨迹过滤器
    
    // Camera intrinsics
    const double fx_ = 690.0;
    const double fy_ = 920.0;
    const double cx_ = 320.0;
    const double cy_ = 240.0;
    const double YAW_CALIBRATION = 0.637;  // 相机偏转角校准
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    bool has_cloud_;
    bool has_odom_;
    
    // FAST-LIO2 Odometry
    double odom_x_, odom_y_, odom_z_;
    double odom_qx_, odom_qy_, odom_qz_, odom_qw_;
    
    std::string target_frame_;
    
public:
    DistanceCalculator() : has_cloud_(false), has_odom_(false) {
        ros::NodeHandle nh;
        
        nh.param<std::string>("target_frame", target_frame_, "camera_init");
        
        cloud_sub_ = nh.subscribe("/cloud_registered", 1, &DistanceCalculator::cloudCallback, this);
        detection_sub_ = nh.subscribe("/detections", 1, &DistanceCalculator::detectionCallback, this);
        odom_sub_ = nh.subscribe("/Odometry", 1, &DistanceCalculator::odomCallback, this);
        
        realtime_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/realtime_persons", 10);
        person_points_pub_ = nh.advertise<geometry_msgs::PointStamped>("/person_points_raw", 100);
        
        ROS_INFO("Distance Calculator Node Started");
        ROS_INFO("  - Publishes: /realtime_persons (markers), /person_points_raw (raw points)");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        has_odom_ = true;
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;
        odom_qx_ = msg->pose.pose.orientation.x;
        odom_qy_ = msg->pose.pose.orientation.y;
        odom_qz_ = msg->pose.pose.orientation.z;
        odom_qw_ = msg->pose.pose.orientation.w;
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *latest_cloud_);
        has_cloud_ = true;
    }
    
    void transformBodyToCamera(double x_body, double y_body, double z_body,
                                double& x_cam, double& y_cam, double& z_cam) {
        if (!has_odom_) {
            x_cam = x_body; y_cam = y_body; z_cam = z_body;
            return;
        }
        
        double qx = odom_qx_, qy = odom_qy_, qz = odom_qz_, qw = odom_qw_;
        
        double x_rot = (1 - 2*qy*qy - 2*qz*qz) * x_body 
                     + (2*qx*qy - 2*qz*qw) * y_body 
                     + (2*qx*qz + 2*qy*qw) * z_body;
        double y_rot = (2*qx*qy + 2*qz*qw) * x_body 
                     + (1 - 2*qx*qx - 2*qz*qz) * y_body 
                     + (2*qy*qz - 2*qx*qw) * z_body;
        double z_rot = (2*qx*qz - 2*qy*qw) * x_body 
                     + (2*qy*qz + 2*qx*qw) * y_body 
                     + (1 - 2*qx*qx - 2*qy*qy) * z_body;
        
        x_cam = x_rot + odom_x_;
        y_cam = y_rot + odom_y_;
        z_cam = z_rot + odom_z_;
    }
    
    double getDepth(double u, double v) {
        if (!has_cloud_ || latest_cloud_->empty()) return 0.8;
        
        double min_dist = 15.0, best_z = 0.8;
        for (const auto& pt : latest_cloud_->points) {
            if (pt.z < 0.3 || pt.z > 10.0) continue;
            
            int uu = static_cast<int>((pt.x * fx_ / pt.z) + cx_);
            int vv = static_cast<int>((pt.y * fy_ / pt.z) + cy_);
            double dist = std::hypot(uu - u, vv - v);
            
            if (dist < min_dist) {
                min_dist = dist;
                best_z = pt.z;
            }
        }
        return best_z;
    }
    
    void detectionCallback(const vision_msgs::Detection2DArrayConstPtr& msg) {
        if (!has_cloud_) {
            ROS_WARN_THROTTLE(5, "Waiting for point cloud...");
            return;
        }
        
        std::vector<std::tuple<double, double, double, double>> current_persons;
        
        for (const auto& det : msg->detections) {
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;
            double confidence = det.results[0].score;
            
            double depth = getDepth(u, v);
            
            // 计算 body 坐标系下的位置
            double x_body = depth;
            double y_body = (cx_ - u) * depth / fx_ + tan(YAW_CALIBRATION) * depth;
            double z_body = (cy_ - v) * depth / fy_;
            
            double distance = std::sqrt(x_body*x_body + y_body*y_body + z_body*z_body);
            
            // 转换到 camera_init
            double x_cam, y_cam, z_cam;
            transformBodyToCamera(x_body, y_body, z_body, x_cam, y_cam, z_cam);
            
            current_persons.push_back({x_cam, y_cam, z_cam, distance});
            
            // 发布原始点给轨迹过滤器
            geometry_msgs::PointStamped point_msg;
            point_msg.header = msg->header;
            point_msg.header.frame_id = target_frame_;
            point_msg.point.x = x_cam;
            point_msg.point.y = y_cam;
            point_msg.point.z = z_cam;
            person_points_pub_.publish(point_msg);
        }
        
        // 发布实时标记
        if (!current_persons.empty()) {
            publishRealtimeMarkers(current_persons, msg->header);
        }
    }
    
    void publishRealtimeMarkers(const std::vector<std::tuple<double, double, double, double>>& persons,
                                 const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < persons.size(); ++i) {
            double x = std::get<0>(persons[i]);
            double y = std::get<1>(persons[i]);
            double z = std::get<2>(persons[i]);
            double distance = std::get<3>(persons[i]);
            
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header = header;
            sphere_marker.header.frame_id = target_frame_;
            sphere_marker.ns = "realtime_persons";
            sphere_marker.id = i;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;
            sphere_marker.pose.position.x = x;
            sphere_marker.pose.position.y = y;
            sphere_marker.pose.position.z = z;
            sphere_marker.pose.orientation.w = 1.0;
            sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.25;
            sphere_marker.color.r = 0.0;
            sphere_marker.color.g = 1.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 0.9;
            sphere_marker.lifetime = ros::Duration(0.2);
            marker_array.markers.push_back(sphere_marker);
            
            visualization_msgs::Marker text_marker;
            text_marker.header = header;
            text_marker.header.frame_id = target_frame_;
            text_marker.ns = "realtime_labels";
            text_marker.id = i;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = x;
            text_marker.pose.position.y = y;
            text_marker.pose.position.z = z + 0.25;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.12;
            text_marker.color.r = 0.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
            char text[100];
            snprintf(text, sizeof(text), "%.2fm", distance);
            text_marker.text = text;
            text_marker.lifetime = ros::Duration(0.2);
            marker_array.markers.push_back(text_marker);
        }
        
        realtime_marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator calculator;
    ros::spin();
    return 0;
}