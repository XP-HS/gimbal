/**
 * rtsp_image_publisher.cpp - 使用 Jetson 硬件解码器
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

class RTSPPublisher {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    
    std::string rtsp_url_;
    std::string ros_topic_;
    cv::VideoCapture cap_;
    bool is_streaming_;
    int frame_width_;
    int frame_height_;
    
    cv_bridge::CvImage cv_image_;
    cv::Mat frame_;
    
    int frame_count_;
    double total_latency_;
    ros::Time last_log_time_;

public:
    RTSPPublisher(const std::string& rtsp_url, const std::string& ros_topic) 
        : it_(nh_), rtsp_url_(rtsp_url), ros_topic_(ros_topic),
          is_streaming_(false), frame_count_(0), total_latency_(0) {
        
        image_pub_ = it_.advertise(ros_topic_, 1);
        cv_image_.encoding = sensor_msgs::image_encodings::BGR8;
        
        ROS_INFO("========================================");
        ROS_INFO("RTSP URL: %s", rtsp_url_.c_str());
        ROS_INFO("Image Topic: %s", ros_topic_.c_str());
        ROS_INFO("Decoder: NVIDIA Jetson Hardware (H.264)");
        ROS_INFO("========================================");
    }

    bool initialize() {
        ROS_INFO("Connecting to RTSP stream...");
        
        // ========== 方案1：使用 NVIDIA Jetson 硬件解码器 ==========
        // nvv4l2decoder 是 Jetson 的硬件解码器
        std::string gst_pipeline = 
            "rtspsrc location=" + rtsp_url_ + 
            " latency=0"
            " drop-on-latency=true"
            " ! rtph264depay"
            " ! h264parse"
            " ! nvv4l2decoder"  // Jetson 硬件解码器
            " ! nvvidconv"
            " ! video/x-raw,format=BGRx"
            " ! videoconvert"
            " ! video/x-raw,format=BGR"
            " ! appsink max-buffers=1 drop=true";
        
        ROS_INFO("Using NVIDIA hardware decoder...");
        cap_.open(gst_pipeline, cv::CAP_GSTREAMER);
        
        if (!cap_.isOpened()) {
            ROS_WARN("Hardware decoder failed, trying software decoder...");
            
            // ========== 方案2：使用 decodebin（自动选择） ==========
            std::string gst_pipeline2 = 
                "rtspsrc location=" + rtsp_url_ + 
                " latency=0 drop-on-latency=true ! "
                "decodebin ! "
                "videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink max-buffers=1 drop=true";
            
            cap_.open(gst_pipeline2, cv::CAP_GSTREAMER);
        }
        
        if (!cap_.isOpened()) {
            ROS_ERROR("GStreamer failed, trying FFMPEG...");
            
            // ========== 方案3：回退到 FFMPEG ==========
            cap_.open(rtsp_url_, cv::CAP_FFMPEG);
            if (cap_.isOpened()) {
                cap_.set(cv::CAP_PROP_BUFFERSIZE, 0);
            }
        }
        
        if (!cap_.isOpened()) {
            ROS_ERROR("Cannot open RTSP stream");
            return false;
        }
        
        // 等待第一帧
        ros::Duration(0.5).sleep();
        
        // 获取视频属性
        frame_width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        frame_height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        
        if (frame_width_ <= 0) frame_width_ = 640;
        if (frame_height_ <= 0) frame_height_ = 480;
        
        ROS_INFO("========================================");
        ROS_INFO("RTSP Connected Successfully!");
        ROS_INFO("Resolution: %dx%d", frame_width_, frame_height_);
        ROS_INFO("========================================");
        
        is_streaming_ = true;
        return true;
    }

    void run() {
        if (!initialize()) return;
        
        ros::Rate rate(30);
        last_log_time_ = ros::Time::now();
        
        ROS_INFO("Starting stream publishing...");
        
        while (ros::ok() && is_streaming_) {
            ros::Time frame_start = ros::Time::now();
            
            if (!cap_.read(frame_)) {
                ROS_WARN_THROTTLE(1, "Failed to read frame");
                ros::Duration(0.01).sleep();
                continue;
            }
            
            if (frame_.empty()) continue;
            
            ros::Time current_time = ros::Time::now();
            double latency = (current_time - frame_start).toSec() * 1000;
            total_latency_ += latency;
            frame_count_++;
            
            try {
                cv_image_.header.stamp = current_time;
                cv_image_.header.frame_id = "camera_frame";
                cv_image_.image = frame_;
                image_pub_.publish(cv_image_.toImageMsg());
                
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge error: %s", e.what());
            }
            
            if (frame_count_ % 100 == 0 && frame_count_ > 0) {
                double avg_latency = total_latency_ / frame_count_;
                ROS_INFO("Frame %d - Avg latency: %.1f ms", frame_count_, avg_latency);
            }
            
            if ((ros::Time::now() - last_log_time_).toSec() >= 10.0) {
                ROS_INFO("Published %d frames, FPS: %.1f", 
                         frame_count_, frame_count_ / 10.0);
                last_log_time_ = ros::Time::now();
                frame_count_ = 0;
                total_latency_ = 0;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    std::string rtsp_url = "rtsp://192.168.144.108";
    std::string ros_topic = "/camera/image_raw";
    
    if (argc >= 2) rtsp_url = argv[1];
    if (argc >= 3) ros_topic = argv[2];
    
    ros::init(argc, argv, "rtsp_image_publisher");
    
    try {
        RTSPPublisher publisher(rtsp_url, ros_topic);
        publisher.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}