#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <cmath>

class PersonClustering {
private:
    ros::Subscriber point_sub_;
    ros::Publisher person_list_pub_;     // 发布去重后的人员坐标列表
    
    struct Person {
        int id;
        double x, y, z;
        double last_seen_time;
        int detection_count;
    };
    
    std::map<int, Person> persons_;
    int next_id_;
    
    // 参数
    double cluster_radius_;      // 距离阈值（仅大于该值才存储/发布）
    double cleanup_timeout_;     // 清理超时（秒，可选保留：超时移除久未更新的点）
    
public:
    PersonClustering() : next_id_(0) {
        ros::NodeHandle nh("~");
        
        // 阈值改为1.5米（可通过参数配置）
        nh.param<double>("cluster_radius", cluster_radius_, 3);
        nh.param<double>("cleanup_timeout", cleanup_timeout_, 60.0);
        
        point_sub_ = nh.subscribe("/person_points_raw", 10, 
                                   &PersonClustering::pointCallback, this);
        person_list_pub_ = nh.advertise<geometry_msgs::PoseArray>("/person_list", 10);
        
        ROS_INFO("Person Clustering Node Started");
        ROS_INFO("  - Distance threshold: %.2f m", cluster_radius_);
        ROS_INFO("  - Cleanup timeout: %.1f s", cleanup_timeout_);
        ROS_INFO("  - Publishing: /person_list (PoseArray)");
    }
    
    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double current_time = msg->header.stamp.toSec();
        double x = msg->point.x;
        double y = msg->point.y;
        double z = msg->point.z;
        
        // 核心逻辑：检查新点是否与所有已存储点距离都大于阈值
        bool is_new_person = true;
        for (const auto& pair : persons_) {
            const auto& person = pair.second;
            double dist = std::hypot(x - person.x, y - person.y);
            
            // 只要有一个已存储点距离小于阈值，就不存储/发布
            if (dist < cluster_radius_) {
                is_new_person = false;
                break;
            }
        }
        
        // 仅当满足「所有已存储点距离都大于阈值」时，才创建新点
        if (is_new_person) {
            createNewPerson(x, y, z, current_time);
            // 清理过期点（可选逻辑，若不需要超时清理可删除）
            cleanupOldPersons(current_time);
            // 发布更新后的列表
            publishPersonList(msg->header.frame_id);
        }
        // 否则：忽略该点，不存储、不发布
    }
    
    // 新增：单独封装创建新人员的逻辑
    void createNewPerson(double x, double y, double z, double current_time) {
        Person new_person;
        new_person.id = next_id_++;
        new_person.x = x;
        new_person.y = y;
        new_person.z = z;
        new_person.last_seen_time = current_time;
        new_person.detection_count = 1;
        
        persons_[new_person.id] = new_person;
        ROS_INFO("[NEW] Person %d at (%.2f, %.2f, %.2f)", 
                 new_person.id, x, y, z);
    }
    
    // 保留清理逻辑（可选：若不需要超时移除已存储点，可删除此函数及调用）
    void cleanupOldPersons(double current_time) {
        auto it = persons_.begin();
        while (it != persons_.end()) {
            if (current_time - it->second.last_seen_time > cleanup_timeout_) {
                ROS_INFO("[REMOVED] Person %d (timeout)", it->first);
                it = persons_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void publishPersonList(const std::string& frame_id) {
        geometry_msgs::PoseArray msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;
        
        for (const auto& pair : persons_) {
            geometry_msgs::Pose pose;
            pose.position.x = pair.second.x;
            pose.position.y = pair.second.y;
            pose.position.z = pair.second.z;
            pose.orientation.w = 1.0;  // 无效值，仅占位
            
            msg.poses.push_back(pose);
        }
        
        person_list_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_clustering");
    PersonClustering clustering;
    ros::spin();
    return 0;
}
