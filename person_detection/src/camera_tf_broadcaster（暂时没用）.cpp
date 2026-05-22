#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TFManager {
private:
    tf::TransformBroadcaster br_;
    ros::Subscriber odom_sub_;
    
    bool has_odom_;
    double last_x_, last_y_, last_z_;
    double last_qx_, last_qy_, last_qz_, last_qw_;
    
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    
public:
    TFManager() : has_odom_(false) {
        ros::NodeHandle nh("~");
        
        nh.param<std::string>("map_frame", map_frame_, "map");
        nh.param<std::string>("odom_frame", odom_frame_, "odom");
        nh.param<std::string>("base_frame", base_frame_, "base_link");
        nh.param<std::string>("camera_frame", camera_frame_, "camera_init");
        
        // 订阅 FAST-LIO2 的 Odometry（不是 MAVROS 的）
        odom_sub_ = nh.subscribe("/Odometry", 10, &TFManager::odomCallback, this);
        
        ROS_INFO("TF Manager: connecting %s -> %s -> %s -> %s", 
                 map_frame_.c_str(), odom_frame_.c_str(), 
                 base_frame_.c_str(), camera_frame_.c_str());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        has_odom_ = true;
        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;
        last_z_ = msg->pose.pose.position.z;
        last_qx_ = msg->pose.pose.orientation.x;
        last_qy_ = msg->pose.pose.orientation.y;
        last_qz_ = msg->pose.pose.orientation.z;
        last_qw_ = msg->pose.pose.orientation.w;
    }
    
    void run() {
        ros::Rate rate(100);
        
        while (ros::ok()) {
            ros::Time now = ros::Time::now();
            
            // 1. map -> odom (静态)
            tf::Transform map_to_odom;
            map_to_odom.setIdentity();
            br_.sendTransform(tf::StampedTransform(map_to_odom, now, map_frame_, odom_frame_));
            
            // 2. odom -> base_link (动态，使用FAST-LIO2数据)
            if (has_odom_) {
                tf::Transform odom_to_base;
                odom_to_base.setOrigin(tf::Vector3(last_x_, last_y_, last_z_));
                tf::Quaternion q(last_qx_, last_qy_, last_qz_, last_qw_);
                odom_to_base.setRotation(q);
                br_.sendTransform(tf::StampedTransform(odom_to_base, now, odom_frame_, base_frame_));
            }
            
            // 3. base_link -> camera_init (静态)
            tf::Transform base_to_camera;
            base_to_camera.setOrigin(tf::Vector3(0.0, 0.0, -0.2));
            tf::Quaternion q_cam;
            q_cam.setRPY(0, 0, 0);
            base_to_camera.setRotation(q_cam);
            br_.sendTransform(tf::StampedTransform(base_to_camera, now, base_frame_, camera_frame_));
            
            rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_manager");
    TFManager tf_manager;
    tf_manager.run();
    return 0;
}