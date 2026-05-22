#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <map>
#include <deque>

class DistanceCalculator {
private:
    ros::Subscriber cloud_sub_;
    ros::Subscriber detection_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher realtime_marker_pub_;
    ros::Publisher history_marker_pub_;
    
    // Camera intrinsics
    const double fx_ = 690.0;
    const double fy_ = 920.0;
    const double cx_ = 320.0;
    const double cy_ = 240.0;

    const double YAW_CALIBRATION = 0.637; // 相机偏转角校准（弧度），正值为向右补偿
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    bool has_cloud_;
    int total_count_;
    
    // FAST-LIO2 的 Odometry 数据 (camera_init -> body)
    bool has_odom_;
    double odom_x_, odom_y_, odom_z_;
    double odom_qx_, odom_qy_, odom_qz_, odom_qw_;
    
    // 目标坐标系（发布到 camera_init）
    std::string target_frame_;
    
    // 历史轨迹管理
    struct TrajectoryPoint {
        double x, y, z;
        double timestamp;
        double distance;
    };
    
    struct TemporaryPoint {
        double x, y, z;
        double first_seen_time;
        double last_seen_time;
        int detection_count;
        bool is_confirmed;
    };
    
    struct PersonTrajectory {
        int id;
        std::deque<TrajectoryPoint> points;
        double last_seen_time;
        double last_x, last_y, last_z;
        std::vector<TemporaryPoint> temp_points;
    };
    
    std::map<int, PersonTrajectory> trajectories_;
    const int MAX_HISTORY_POINTS = 30;
    const double MIN_DISTANCE_BETWEEN_PERSONS = 0.4;
    const double MIN_DISTANCE_BETWEEN_POINTS = 0.5;
    const double STAY_RADIUS = 0.3;
    const double STAY_DURATION = 3.0;
    const int MIN_DETECTION_COUNT = 5;
    
    int next_person_id_;
    
    // Odometry 回调
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
    
    // 将 body 坐标系下的点转换到 camera_init 坐标系
    // 公式: P_camera_init = R * P_body + T
    // 其中 R 是 body -> camera_init 的旋转，T 是平移
    void transformBodyToCamera(double x_body, double y_body, double z_body,
                                double& x_cam, double& y_cam, double& z_cam) {
        if (!has_odom_) {
            x_cam = x_body;
            y_cam = y_body;
            z_cam = z_body;
            return;
        }
        
        // 1. 先用四元数旋转（body -> camera_init）
        double qx = odom_qx_;
        double qy = odom_qy_;
        double qz = odom_qz_;
        double qw = odom_qw_;
        
        double x_rot = (1 - 2*qy*qy - 2*qz*qz) * x_body 
                     + (2*qx*qy - 2*qz*qw) * y_body 
                     + (2*qx*qz + 2*qy*qw) * z_body;
        double y_rot = (2*qx*qy + 2*qz*qw) * x_body 
                     + (1 - 2*qx*qx - 2*qz*qz) * y_body 
                     + (2*qy*qz - 2*qx*qw) * z_body;
        double z_rot = (2*qx*qz - 2*qy*qw) * x_body 
                     + (2*qy*qz + 2*qx*qw) * y_body 
                     + (1 - 2*qx*qx - 2*qy*qy) * z_body;
        
        // 2. 加上平移
        x_cam = x_rot + odom_x_;
        y_cam = y_rot + odom_y_;
        z_cam = z_rot + odom_z_;
    }
    
    // 获取深度值（从点云中匹配）
    double getDepth(double u, double v) {
        if (!has_cloud_ || latest_cloud_->empty()) {
            return 0.8;
        }
        
        double min_dist = 15.0;
        double best_z = 0.8;
        
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
    
    double calculate3DDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
        return std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
    }
    
    int findExistingPerson(double x, double y, double z) {
        double min_dist = MIN_DISTANCE_BETWEEN_PERSONS;
        int matched_id = -1;
        
        for (const auto& traj_pair : trajectories_) {
            const auto& traj = traj_pair.second;
            if (traj.points.empty()) continue;
            
            const auto& last_point = traj.points.back();
            double dist = calculate3DDistance(x, y, z, last_point.x, last_point.y, last_point.z);
            
            if (dist < min_dist) {
                min_dist = dist;
                matched_id = traj.id;
            }
        }
        return matched_id;
    }
    
    int findOrCreateTemporaryPoint(PersonTrajectory& traj, double x, double y, double z, double current_time) {
        for (size_t i = 0; i < traj.temp_points.size(); ++i) {
            auto& temp = traj.temp_points[i];
            double dist = calculate3DDistance(x, y, z, temp.x, temp.y, temp.z);
            
            if (dist < STAY_RADIUS) {
                temp.last_seen_time = current_time;
                temp.detection_count++;
                return i;
            }
        }
        
        TemporaryPoint new_temp;
        new_temp.x = x;
        new_temp.y = y;
        new_temp.z = z;
        new_temp.first_seen_time = current_time;
        new_temp.last_seen_time = current_time;
        new_temp.detection_count = 1;
        new_temp.is_confirmed = false;
        traj.temp_points.push_back(new_temp);
        return traj.temp_points.size() - 1;
    }
    
    void checkAndConfirmStayedPoints(PersonTrajectory& traj, double current_time) {
        for (auto& temp : traj.temp_points) {
            if (temp.is_confirmed) continue;
            
            double stay_duration = current_time - temp.first_seen_time;
            
            bool should_confirm = (stay_duration >= STAY_DURATION) ||
                                  (temp.detection_count >= MIN_DETECTION_COUNT && stay_duration >= 1.0);
            
            if (should_confirm) {
                temp.is_confirmed = true;
                
                TrajectoryPoint point;
                point.x = temp.x;
                point.y = temp.y;
                point.z = temp.z;
                point.timestamp = current_time;
                point.distance = calculate3DDistance(temp.x, temp.y, temp.z, 0, 0, 0);
                
                traj.points.push_back(point);
                
                while (traj.points.size() > MAX_HISTORY_POINTS) {
                    traj.points.pop_front();
                }
                
                ROS_INFO("Person %d confirmed stay at (%.2f, %.2f, %.2f) - duration: %.1fs, detections: %d", 
                         traj.id, temp.x, temp.y, temp.z, stay_duration, temp.detection_count);
            }
        }
        
        auto it = traj.temp_points.begin();
        while (it != traj.temp_points.end()) {
            if (!it->is_confirmed && (current_time - it->last_seen_time) > 10.0) {
                it = traj.temp_points.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    bool shouldAddHistoryPoint(int person_id, double x, double y, double z, double current_time) {
        auto it = trajectories_.find(person_id);
        if (it == trajectories_.end()) {
            return true;
        }
        
        auto& traj = it->second;
        if (traj.points.empty()) {
            return true;
        }
        
        const auto& last_point = traj.points.back();
        
        double dist_diff = calculate3DDistance(x, y, z, last_point.x, last_point.y, last_point.z);
        
        if (dist_diff >= MIN_DISTANCE_BETWEEN_POINTS) {
            ROS_INFO("Person %d moved %.2fm (>=%.1fm) - adding history point", 
                     person_id, dist_diff, MIN_DISTANCE_BETWEEN_POINTS);
            return true;
        }
        
        return false;
    }
    
    void updateTrajectory(int person_id, double x, double y, double z, double distance, double current_time) {
        auto it = trajectories_.find(person_id);
        
        if (it == trajectories_.end()) {
            PersonTrajectory traj;
            traj.id = person_id;
            traj.last_seen_time = current_time;
            traj.last_x = x;
            traj.last_y = y;
            traj.last_z = z;
            
            TrajectoryPoint point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.timestamp = current_time;
            point.distance = distance;
            traj.points.push_back(point);
            
            trajectories_[person_id] = traj;
            ROS_INFO("Created new trajectory for person %d at (%.2f, %.2f, %.2f) dist=%.2fm", 
                     person_id, x, y, z, distance);
        } else {
            auto& traj = it->second;
            
            bool should_add = shouldAddHistoryPoint(person_id, x, y, z, current_time);
            
            if (should_add) {
                TrajectoryPoint point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.timestamp = current_time;
                point.distance = distance;
                
                traj.points.push_back(point);
                
                while (traj.points.size() > MAX_HISTORY_POINTS) {
                    traj.points.pop_front();
                }
                
                ROS_INFO("Person %d added history point, now %zu points", 
                         person_id, traj.points.size());
                
                traj.temp_points.clear();
            } else {
                findOrCreateTemporaryPoint(traj, x, y, z, current_time);
            }
            
            checkAndConfirmStayedPoints(traj, current_time);
            
            traj.last_seen_time = current_time;
            traj.last_x = x;
            traj.last_y = y;
            traj.last_z = z;
        }
    }
    
    void publishRealtimeMarkers(const std::vector<std::tuple<double, double, double, double, double, int>>& persons, 
                                 const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        
        for (const auto& p : persons) {
            double x = std::get<0>(p);
            double y = std::get<1>(p);
            double z = std::get<2>(p);
            double distance = std::get<3>(p);
            int id = std::get<5>(p);
            
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header = header;
            sphere_marker.header.frame_id = target_frame_;
            sphere_marker.ns = "realtime_persons";
            sphere_marker.id = id;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;
            
            sphere_marker.pose.position.x = x;
            sphere_marker.pose.position.y = y;
            sphere_marker.pose.position.z = z;
            sphere_marker.pose.orientation.w = 1.0;
            
            sphere_marker.scale.x = 0.25;
            sphere_marker.scale.y = 0.25;
            sphere_marker.scale.z = 0.25;
            
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
            text_marker.id = id;
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
            snprintf(text, sizeof(text), "P%d: %.2fm", id, distance);
            text_marker.text = text;
            
            text_marker.lifetime = ros::Duration(0.2);
            
            marker_array.markers.push_back(text_marker);
        }
        
        realtime_marker_pub_.publish(marker_array);
    }
    
    void publishHistoryMarkers(double current_time) {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        
        for (const auto& traj_pair : trajectories_) {
            const auto& traj = traj_pair.second;
            
            for (size_t i = 0; i < traj.points.size(); ++i) {
                const auto& point = traj.points[i];
                
                visualization_msgs::Marker history_marker;
                history_marker.header.frame_id = target_frame_;
                history_marker.header.stamp = ros::Time(point.timestamp);
                history_marker.ns = "history_points";
                history_marker.id = marker_id++;
                history_marker.type = visualization_msgs::Marker::SPHERE;
                history_marker.action = visualization_msgs::Marker::ADD;
                
                history_marker.pose.position.x = point.x;
                history_marker.pose.position.y = point.y;
                history_marker.pose.position.z = point.z;
                history_marker.pose.orientation.w = 1.0;
                
                history_marker.scale.x = 0.1;
                history_marker.scale.y = 0.1;
                history_marker.scale.z = 0.1;
                
                history_marker.color.r = 1.0;
                history_marker.color.g = 0.5;
                history_marker.color.b = 0.0;
                history_marker.color.a = 0.8;
                
                history_marker.lifetime = ros::Duration(0);
                
                marker_array.markers.push_back(history_marker);
            }
        }
        
        history_marker_pub_.publish(marker_array);
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *latest_cloud_);
        has_cloud_ = true;
    }
    
    void detectionCallback(const vision_msgs::Detection2DArrayConstPtr& msg) {
        if (!has_cloud_) {
            ROS_WARN_THROTTLE(5, "Waiting for point cloud data...");
            return;
        }
        
        if (!has_odom_) {
            ROS_WARN_THROTTLE(5, "Waiting for Odometry data...");
        }
        
        int num_people = msg->detections.size();
        std::vector<std::tuple<double, double, double, double, double, int>> current_persons;
        double current_time = ros::Time::now().toSec();
        
        for (size_t i = 0; i < msg->detections.size(); ++i) {
            const auto& det = msg->detections[i];
            double u = det.bbox.center.x;
            double v = det.bbox.center.y;
            double confidence = det.results[0].score;
            
            // 1. 获取深度（这就是人在 body 坐标系下的 Z 坐标？）
            double depth = getDepth(u, v);
            
            // 2. 计算人在 body 坐标系下的位置
            // 假设相机朝前，那么：
            //   X_body = 深度（前方距离）
            //   Y_body = 横向偏移（根据像素 u 计算）
            //   Z_body = 高度（根据像素 v 计算）
            double x_body = depth;  // 前方距离
            double y_body_raw = (cx_ - u) * depth / fx_;  // 左右偏移（左正右负）
            double y_body = y_body_raw + tan(YAW_CALIBRATION) * depth;
            double z_body = (cy_ - v) * depth / fy_;  // 高度偏移
            
            // 3. 转换到 camera_init 坐标系
            double x_cam, y_cam, z_cam;
            transformBodyToCamera(x_body, y_body, z_body, x_cam, y_cam, z_cam);
            
            // 4. 计算距离（欧氏距离）
            double distance = std::sqrt(x_body*x_body + y_body*y_body + z_body*z_body);
            
            total_count_++;
            
            // 5. 匹配已有轨迹（使用 camera_init 坐标）
            int person_id = findExistingPerson(x_cam, y_cam, z_cam);
            
            if (person_id == -1) {
                person_id = next_person_id_++;
                ROS_INFO("NEW PERSON #%d: body=(%.2f, %.2f, %.2f) cam=(%.2f, %.2f, %.2f) dist=%.2fm", 
                         person_id, x_body, y_body, z_body, x_cam, y_cam, z_cam, distance);
            }
            
            // 6. 发布到 camera_init 坐标系
            current_persons.push_back({x_cam, y_cam, z_cam, distance, confidence, person_id});
            updateTrajectory(person_id, x_cam, y_cam, z_cam, distance, current_time);
        }
        
        if (num_people > 0) {
            int total_history_points = 0;
            for (const auto& traj : trajectories_) {
                total_history_points += traj.second.points.size();
            }
            ROS_INFO("Frame: %d person(s), tracking %zu persons, %d history points", 
                     num_people, trajectories_.size(), total_history_points);
        }
        
        if (!current_persons.empty()) {
            publishRealtimeMarkers(current_persons, msg->header);
        }
        publishHistoryMarkers(current_time);
    }
    
public:
    DistanceCalculator() : has_cloud_(false), has_odom_(false), total_count_(0), next_person_id_(0),
                           odom_x_(0), odom_y_(0), odom_z_(0),
                           odom_qx_(0), odom_qy_(0), odom_qz_(0), odom_qw_(1) {
        ros::NodeHandle nh;
        
        // 发布到 camera_init 坐标系
        nh.param<std::string>("target_frame", target_frame_, "camera_init");
        
        ROS_INFO("========================================");
        ROS_INFO("Distance Calculator Started");
        ROS_INFO("  - Target frame: %s (publishing in camera_init)", target_frame_.c_str());
        ROS_INFO("  - RViz Fixed Frame: body");
        ROS_INFO("========================================");
        
        cloud_sub_ = nh.subscribe("/cloud_registered", 1, 
                                   &DistanceCalculator::cloudCallback, this);
        detection_sub_ = nh.subscribe("/detections", 1, 
                                       &DistanceCalculator::detectionCallback, this);
        odom_sub_ = nh.subscribe("/Odometry", 1, 
                                  &DistanceCalculator::odomCallback, this);
        realtime_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/realtime_persons", 10);
        history_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/history_trajectories", 10);
        
        ROS_INFO("Subscribed to: /cloud_registered, /detections, /Odometry");
        ROS_INFO("Published: /realtime_persons (in %s frame)", target_frame_.c_str());
        ROS_INFO("========================================");
        ROS_INFO("RViz Setup:");
        ROS_INFO("  - Fixed Frame: body");
        ROS_INFO("  - Add MarkerArray: /realtime_persons");
        ROS_INFO("========================================");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator calculator;
    ros::spin();
    return 0;
}