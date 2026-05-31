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
        sync.reset(new Sync(MySyncPolicy(10), sub_lidar, sub_detections, sub_odom));
        sync->registerCallback(boost::bind(&DistanceCalculator::syncCallback, this, _1, _2, _3));

        ROS_INFO("Distance calculator node started");
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

    // 相机内参（标定得到）
    const double fx = 2282.72;
    const double fy = 2288.64;
    const double cx = 966.55;
    const double cy = 519.82;

    // 相机相对于雷达的外参：前0.18m，下0.34m
    const double tx = 0.18;
    const double ty = 0.0;
    const double tz = 0.34;

    // 滤波参数
    const double alpha = 0.6;
    double last_x = 0, last_y = 0, last_z = 0;
    bool first_frame = true;

    // 短时记忆
    std::vector<TrackedPerson> tracked_;
    const double max_memory = 0.5;

    // 统一同步回调：雷达、检测、里程计时间对齐后才会调用
    void syncCallback(
        const CustomMsg::ConstPtr& lidar_msg,
        const vision_msgs::Detection2DArrayConstPtr& det_msg,
        const nav_msgs::OdometryConstPtr& odom_msg)
    {
        // 同步状态打印，1秒限制输出频率
        ROS_INFO_THROTTLE(1, "Sync callback, persons: %d", (int)det_msg->detections.size());

        // 解析雷达点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto& pt : lidar_msg->points)
        {
            if (pt.x < 0.2 || pt.x > 12.0) continue;
            cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }

        if (cloud->empty()) return;

        std::vector<TrackedPerson> new_tracked;
        ros::Time now = ros::Time::now();
        int marker_id = 0;

        for (const auto& det : det_msg->detections)
        {
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;
            double depth = getDepth(u, v, cloud);

            double xb = 0, yb = 0, zb = 0;
            double xw = 0, yw = 0, zw = 0;
            double dist = 0;
            bool valid = false;

            if (depth > 0)
            {
                // 像素到相机坐标系：右X，下Y，前Z
                double cx_cam = (u - cx) * depth / fx;
                double cy_cam = (v - cy) * depth / fy;
                double cz_cam = depth;

                // 相机坐标系到雷达坐标系【前X、左Y、上Z】
                double x_lidar = cz_cam + tx;
                double y_lidar = -cx_cam + ty;
                double z_lidar = -cy_cam + tz;

                xb = x_lidar;
                yb = y_lidar;
                zb = z_lidar;

                // 转到世界系
                if (bodyToWorld(xb, yb, zb, xw, yw, zw, det_msg->header.stamp))
                {
                    dist = sqrt(xb*xb + yb*yb + zb*zb);
                    valid = true;
                }
            }

            // 短时记忆
            if (!valid)
            {
                for (const auto& t : tracked_)
                {
                    if ((now - t.last_time).toSec() < max_memory)
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

            // 滤波
            double fxw, fyw, fzw;
            if (first_frame)
            {
                fxw = xw;
                fyw = yw;
                fzw = zw;
                first_frame = false;
            }
            else
            {
                fxw = alpha * xw + (1 - alpha) * last_x;
                fyw = alpha * yw + (1 - alpha) * last_y;
                fzw = alpha * zw + (1 - alpha) * last_z;
            }
            last_x = fxw;
            last_y = fyw;
            last_z = fzw;

            // 保存跟踪
            TrackedPerson p;
            p.xw = fxw;
            p.yw = fyw;
            p.zw = fzw;
            p.distance = dist;
            p.last_time = now;
            p.id = marker_id;
            new_tracked.push_back(p);

            // 发布
            geometry_msgs::PointStamped out;
            out.header.stamp = det_msg->header.stamp;
            out.header.frame_id = "camera_init";
            out.point.x = fxw;
            out.point.y = fyw;
            out.point.z = fzw;
            person_pub.publish(out);

            publishMarker(fxw, fyw, fzw, dist, marker_id++);
        }

        tracked_ = new_tracked;
    }

    // 深度计算：雷达点反向投影到像素
    double getDepth(double u, double v, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double min_dist = 1e9;
        double best_depth = -1;

        for (const auto& p : cloud->points)
        {
            if (p.x < 0.3 || p.x > 10.0)
                continue;

            // 雷达到相机坐标系正确映射
            double cx_cam = -p.y;
            double cy_cam = -p.z;
            double cz_cam = p.x;

            if (cz_cam < 0.1) continue;

            // 投影到像素
            int uu = cx_cam * fx / cz_cam + cx;
            int vv = cy_cam * fy / cz_cam + cy;

            double d = hypot(uu - u, vv - v);
            if (d < min_dist)
            {
                min_dist = d;
                best_depth = cz_cam;
            }
        }
        return best_depth;
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

        // 距离文字
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
