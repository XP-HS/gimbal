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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;
using namespace livox_ros_driver2;

struct TrackedPerson
{
    double xw, yw, zw;
    double distance;
    ros::Time last_time;
    int id;
};

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        // 发布
        person_pub = nh.advertise<geometry_msgs::PointStamped>("/person_points_raw", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/realtime_persons", 10);

        // 同步订阅器初始化
        sub_lidar.subscribe(nh, "/livox/lidar", 10);
        sub_detections.subscribe(nh, "/detections", 10);
        sub_odom.subscribe(nh, "/Odometry", 10);

        // ROS Noetic 兼容版本同步策略
        sync.reset(new Sync(MySyncPolicy(10), sub_lidar, sub_detections, sub_odom));
        sync->registerCallback(boost::bind(&DistanceCalculator::syncCallback, this, _1, _2, _3));

        ROS_INFO("Distance calculator node started: Transform Livox body frame to camera_init world frame");
    }

private:
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    // 同步策略定义
    typedef sync_policies::ApproximateTime<CustomMsg, vision_msgs::Detection2DArray, nav_msgs::Odometry> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // 同步订阅者
    message_filters::Subscriber<livox_ros_driver2::CustomMsg> sub_lidar;
    message_filters::Subscriber<vision_msgs::Detection2DArray> sub_detections;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;

    ros::Publisher person_pub, marker_pub;

    // 相机内参
    const double fx = 690.0;
    const double fy = 920.0;
    const double cx = 320.0;
    const double cy = 240.0;
    const double yaw_calib = 0.637;

    // 相机相对于雷达的外参偏移
    const double camera_offset_x = 0.18;   // 前
    const double camera_offset_y = 0.34;   // 下
    const double camera_offset_z = 0.00;   // 左/右

    // 滤波系数
    //alpha 越大（趋近 1）：越相信当前原始数据，响应快、滤波弱，抖动抑制效果差；
    //alpha 越小（趋近 0）：越相信历史数据，平滑效果强、延迟变大
    const double alpha = 0.6; 
    double prev_x = 0, prev_y = 0, prev_z = 0;
    bool first_estimate = true;

    // 短时记忆跟踪
    std::vector<TrackedPerson> tracked_;
    const double max_memory_duration = 0.5;

    // 统一同步回调：雷达、检测、里程计时间对齐后才会调用
    void syncCallback(
        const CustomMsg::ConstPtr& lidar_msg,
        const vision_msgs::Detection2DArrayConstPtr& det_msg,
        const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        // 同步状态打印，1秒限制输出频率
        ROS_INFO_THROTTLE(1, "Sync callback triggered, detected persons: %d", (int)det_msg->detections.size());

        // 清除上一帧的所有可视化标记
        visualization_msgs::MarkerArray delete_array;
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_array.markers.push_back(delete_marker);
        marker_pub.publish(delete_array);

        // 解析当前同步的雷达点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto& pt : lidar_msg->points)
        {
            // 过滤近距离和远距离无效点
            if (pt.x < 0.2 || pt.x > 12.0) continue;
            cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }

        std::vector<TrackedPerson> new_tracked;
        ros::Time now = ros::Time::now();
        int marker_id = 0;

        for (const auto& det : det_msg->detections)
        {
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;
            double depth = getDepth(u, v, cloud);

            double xw = 0, yw = 0, zw = 0;
            double dist = 0;
            bool valid = false;

            if (depth > 0)
            {
                // 计算机身系坐标，并加入相机外参补偿
                double xb = depth + camera_offset_x;
                double yb = (cx - u) * depth / fx + tan(yaw_calib) * depth + camera_offset_y;
                double zb = (cy - v) * depth / fy + camera_offset_z;

                // 机身系转到世界系
                if (bodyToWorld(xb, yb, zb, xw, yw, zw, det_msg->header.stamp))
                {
                    dist = hypot(xb, hypot(yb, zb));
                    valid = true;
                }
            }

            if (!valid)
            {
                // 雷达点缺失，尝试使用历史记忆
                for (const auto& t : tracked_)
                {
                    if ((now - t.last_time).toSec() < max_memory_duration)
                    {
                        xw = t.xw;
                        yw = t.yw;
                        zw = t.zw;
                        dist = t.distance;
                        valid = true;
                        break;
                    }
                }
            }

            if (!valid) continue;

            // 一阶低通滤波，平滑抖动
            double fxw, fyw, fzw;
            if (first_estimate)
            {
                fxw = xw;
                fyw = yw;
                fzw = zw;
                first_estimate = false;
            }
            else
            {
                fxw = alpha * xw + (1 - alpha) * prev_x;
                fyw = alpha * yw + (1 - alpha) * prev_y;
                fzw = alpha * zw + (1 - alpha) * prev_z;
            }
            prev_x = fxw;
            prev_y = fyw;
            prev_z = fzw;

            // 保存到新跟踪列表
            TrackedPerson p;
            p.xw = fxw;
            p.yw = fyw;
            p.zw = fzw;
            p.distance = dist;
            p.last_time = now;
            p.id = marker_id;
            new_tracked.push_back(p);

            // 发布世界坐标
            geometry_msgs::PointStamped out;
            out.header.stamp = det_msg->header.stamp;
            out.header.frame_id = "camera_init";
            out.point.x = fxw;
            out.point.y = fyw;
            out.point.z = fzw;
            person_pub.publish(out);

            // 可视化标记
            publishMarker(fxw, fyw, fzw, dist, marker_id++);
        }

        // 更新跟踪列表
        tracked_ = new_tracked;
    }

    // 深度计算
    double getDepth(double u, double v, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double min_dist = 1e9;
        double best_z = -1;

        for (const auto& p : cloud->points)
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

    // 可视化发布
    void publishMarker(double x, double y, double z, double dist, int id)
    {
        visualization_msgs::MarkerArray arr;

        // 球体标记
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
        m.lifetime = ros::Duration(0.3);
        arr.markers.push_back(m);

        // 距离文字标记
        m.id += 1000;
        m.type = m.TEXT_VIEW_FACING;
        m.pose.position.z += 0.4;
        m.scale.z = 0.2;
        m.text = std::to_string(dist).substr(0,4) + "m";
        m.lifetime = ros::Duration(0.3);
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
