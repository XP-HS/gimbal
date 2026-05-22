#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
import warnings
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

warnings.filterwarnings('ignore')

class PureDetector:
    def __init__(self):
        rospy.init_node('person_detector', anonymous=True)
        self.bridge = CvBridge()

        # ===================== 跳帧降频 =====================
        self.frame_count = 0
        self.process_every_n_frames = 2  # 每2帧检测1次
        # ====================================================

        # YOLOv5 本地加载（完全按你的路径，不动）
        self.model = torch.hub.load(
            '/home/jetson/yolov5',
            'custom',
            path='/home/jetson/yolov5/yolov5s.pt',
            source='local',
            force_reload=False
        )
        self.model = self.model.cuda()
        self.model.classes = [0]  # 只检测人
        self.model.conf = 0.4

        # 发布者
        self.detection_pub = rospy.Publisher('/detections', Detection2DArray, queue_size=10)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)

        # 订阅者
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        rospy.loginfo("✅ 人体检测节点启动 ｜ 跳帧降频：每2帧检测1次")

    def image_callback(self, msg):
        # 跳帧降频（不检测直接返回）
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        # 转换图像
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO 检测
        with torch.no_grad():
            results = self.model(img, size=640)

        boxes = results.xyxy[0]

        # 构建检测消息
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for *xyxy, conf, cls in boxes:
            x1, y1, x2, y2 = map(int, xyxy)

            detection = Detection2D()
            detection.bbox.center.x = (x1 + x2) / 2.0
            detection.bbox.center.y = (y1 + y2) / 2.0
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

            # 画框
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 发布检测结果
        if len(boxes) > 0:
            self.detection_pub.publish(detections_msg)

        # 发布图像
        image_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        image_msg.header = msg.header
        self.image_pub.publish(image_msg)

if __name__ == '__main__':
    try:
        detector = PureDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
