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
#include <tf/transform_listener.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        // 订阅
        cloud_sub = nh.subscribe("/livox/lidar", 1, &DistanceCalculator::cloudCallback, this);
        detection_sub = nh.subscribe("/detections", 1, &DistanceCalculator::detectionCallback, this);
        odom_sub = nh.subscribe("/Odometry", 1, &DistanceCalculator::odomCallback, this);

        // 发布
        person_pub = nh.advertise<geometry_msgs::PointStamped>("/person_points_world", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/realtime_persons", 10);

        ROS_INFO("人物距离计算节点启动: Livox机身系转到camera_init世界系");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub, detection_sub, odom_sub;
    ros::Publisher person_pub, marker_pub;
    tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ>::Ptr body_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    bool has_cloud = false;
    bool has_odom = false;

    // 相机内参
    const double fx = 690.0;
    const double fy = 920.0;
    const double cx = 320.0;
    const double cy = 240.0;
    const double yaw_calib = 0.637;

    // 里程计回调
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        has_odom = true;
    }

    // LIVOX 点云回调
    void cloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
    {
        body_cloud->clear();
        for (const auto& pt : msg->points)
        {
            // p.x p.y p.z = 相对无人机的坐标
            if (pt.x < 0.2 || pt.x > 12.0) continue;
            body_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
        has_cloud = true;
    }

    // 深度计算
    double getDepth(double u, double v)
    {
        if (!has_cloud || body_cloud->empty()) return -1;

        double min_dist = 1e9;
        double best_z = -1;

        for (const auto& p : body_cloud->points)
        {
            if (p.z < 0.3 || p.z > 10.0) continue;

            int uu = (p.x * fx / p.z) + cx;
            int vv = (p.y * fy / p.z) + cy;
            double d = hypot(uu - u, vv - v);

            if (d < min_dist)
            {
                min_dist = d;
                best_z = p.z;
            }
        }
        return best_z;
    }

    // 机身系转到世界系
    bool bodyToWorld(double xb, double yb, double zb, double& xw, double& yw, double& zw, ros::Time t)
    {
        try
        {
            tf::Stamped<tf::Point> p_in(tf::Point(xb, yb, zb), t, "body");
            tf::Stamped<tf::Point> p_out;
            tf_listener.waitForTransform("camera_init", "body", t, ros::Duration(0.05));
            tf_listener.transformPoint("camera_init", p_in, p_out);

            xw = p_out.x();
            yw = p_out.y();
            zw = p_out.z();
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // 检测回调 
    void detectionCallback(const vision_msgs::Detection2DArrayConstPtr& msg)
    {
        if (!has_cloud || !has_odom) return;

        int marker_id = 0;
        for (const auto& det : msg->detections)
        {
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;
            double depth = getDepth(u, v);

            if (depth < 0) continue;

            // 计算机身系坐标
            double xb = depth;
            double yb = (cx - u) * depth / fx + tan(yaw_calib) * depth;
            double zb = (cy - v) * depth / fy;

            // 转世界系
            double xw, yw, zw;
            if (!bodyToWorld(xb, yb, zb, xw, yw, zw, msg->header.stamp))
                continue;

            // 发布世界坐标
            geometry_msgs::PointStamped out;
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = "camera_init";
            out.point.x = xw;
            out.point.y = yw;
            out.point.z = zw;
            person_pub.publish(out);

            // 可视化
            publishMarker(xw, yw, zw, hypot(xb, hypot(yb, zb)), marker_id++);
        }
    }

    // 可视化 
    void publishMarker(double x, double y, double z, double dist, int id)
    {
        visualization_msgs::MarkerArray arr;

        // 球
        visualization_msgs::Marker m;
        m.header.frame_id = "camera_init";
        m.id = id;
        m.type = m.SPHERE;
        m.action = m.ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.scale.x = m.scale.y = m.scale.z = 0.3;
        m.color.g = 1.0;
        m.color.a = 0.8;
        arr.markers.push_back(m);

        // 文字
        m.id += 1000;
        m.type = m.TEXT_VIEW_FACING;
        m.pose.position.z += 0.4;
        m.scale.z = 0.2;
        m.text = std::to_string(dist).substr(0,4) + "m";
        arr.markers.push_back(m);

        marker_pub.publish(arr);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator node;
    ros::spin();
    return 0;
}
