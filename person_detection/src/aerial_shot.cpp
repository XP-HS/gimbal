#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <tf/transform_listener.h>

// 消息同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;
using namespace message_filters;
using namespace visualization_msgs;
using namespace geometry_msgs;

struct TrackedPerson
{
    double xw, yw, zw;
    double distance;
    ros::Time last_time;
    int id;
};

class PersonLocalizer
{
public:
    PersonLocalizer()
    {
        // 发布者
        person_pub = nh.advertise<geometry_msgs::PointStamped>("/person_points_raw", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/realtime_persons", 10);

        // 同步订阅
        sub_detect.subscribe(nh, "/detections", 10);
        sub_odom.subscribe(nh, "/Odometry", 10);

        typedef sync_policies::ApproximateTime<vision_msgs::Detection2DArray, nav_msgs::Odometry> MySyncPolicy;
        sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_detect, sub_odom);
        sync->registerCallback(boost::bind(&PersonLocalizer::syncCallback, this, _1, _2));

        height_valid = false;
        ROS_INFO("Person localizer started");
    }

private:
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    // 同步
    message_filters::Subscriber<vision_msgs::Detection2DArray> sub_detect;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    Synchronizer<sync_policies::ApproximateTime<vision_msgs::Detection2DArray, nav_msgs::Odometry>>* sync;

    ros::Publisher person_pub;
    ros::Publisher marker_pub;

    // 内参
    const double fx = 2282.72;
    const double fy = 2288.64;
    const double cx = 966.55;
    const double cy = 519.82;

    // 外参
    const double tx = 0.18;
    const double tz = 0.34;

    // 滤波
    const double alpha = 0.6;
    double last_x = 0, last_y = 0, last_z = 0;
    bool first_frame = true;

    vector<TrackedPerson> tracked_;
    const double max_memory = 0.5;

    double current_height;
    bool height_valid;

    // 同步回调
    void syncCallback(const vision_msgs::Detection2DArrayConstPtr& detect_msg,
                      const nav_msgs::OdometryConstPtr& odom_msg)
    {
        current_height = odom_msg->pose.pose.position.z;
        height_valid = true;

        double H = fabs(current_height);
        if (H < 0.1) return;

        vector<TrackedPerson> new_tracked;
        ros::Time now = detect_msg->header.stamp;
        int marker_id = 0;

        for (const auto& det : detect_msg->detections)
        {
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;

            double dx = u - cx;
            double dy = v - cy;

            double cam_x = dx / fx;
            double cam_y = dy / fy;

            // ========== 你的核心公式 100% 完全保留 ==========
            double body_x = -cam_y * (H - tz);
            double body_y = -cam_x * (H - tz);
            double body_z = -H;

            body_x += tx;
            // =================================================

            double xw = 0, yw = 0, zw = 0;
            bool valid = false;

            if (bodyToWorld(body_x, body_y, body_z, xw, yw, zw, detect_msg->header.stamp))
                valid = true;

            if (!valid) continue;

            // 滤波
            double fxw, fyw, fzw;
            if (first_frame)
            {
                fxw = xw; fyw = yw; fzw = zw;
                first_frame = false;
            }
            else
            {
                fxw = alpha * xw + (1 - alpha) * last_x;
                fyw = alpha * yw + (1 - alpha) * last_y;
                fzw = alpha * zw + (1 - alpha) * last_z;
            }
            last_x = fxw; last_y = fyw; last_z = fzw;

            TrackedPerson p;
            p.xw = fxw;
            p.yw = fyw;
            p.zw = fzw;
            p.distance = hypot(body_x, body_y);
            p.last_time = now;
            p.id = marker_id;
            new_tracked.push_back(p);

            PointStamped out;
            out.header.stamp = now;
            out.header.frame_id = "camera_init";
            out.point.x = fxw;
            out.point.y = fyw;
            out.point.z = fzw;
            person_pub.publish(out);

            publishMarker(fxw, fyw, fzw, p.distance, marker_id++);
        }
        tracked_ = new_tracked;
    }

    bool bodyToWorld(double xb, double yb, double zb, double& xw, double& yw, double& zw, ros::Time t)
    {
        try
        {
            tf::Stamped<tf::Point> p_in(tf::Point(xb, yb, zb), t, "body");
            tf::Stamped<tf::Point> p_out;
            tf_listener.waitForTransform("camera_init", "body", t, ros::Duration(0.2));
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

    void publishMarker(double x, double y, double z, double dist, int id)
    {
        visualization_msgs::MarkerArray arr;

        // 球体标记
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "camera_init";
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.id = id;
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        
        sphere_marker.pose.position.x = x;
        sphere_marker.pose.position.y = y;
        sphere_marker.pose.position.z = z;
        
        sphere_marker.pose.orientation.x = 0.0;
        sphere_marker.pose.orientation.y = 0.0;
        sphere_marker.pose.orientation.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;
        
        sphere_marker.scale.x = 0.3;
        sphere_marker.scale.y = 0.3;
        sphere_marker.scale.z = 0.3;
        
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 0.0;
        sphere_marker.color.a = 0.8;
        
        sphere_marker.lifetime = ros::Duration(0.3);
        arr.markers.push_back(sphere_marker);

        // 文字标记
        visualization_msgs::Marker text_marker = sphere_marker;
        text_marker.id = id + 1000;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z += 0.4;
        text_marker.scale.z = 0.2;
        text_marker.text = std::to_string(dist).substr(0, 4) + "m";
        arr.markers.push_back(text_marker);

        marker_pub.publish(arr);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aerial_shot");
    PersonLocalizer node;
    ros::spin();
    return 0;
}
