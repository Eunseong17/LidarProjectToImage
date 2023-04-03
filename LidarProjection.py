# -*- coding: utf-8 -*-
import rospy
import cv2
import open3d as o3d
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.point_cloud2 import read_points
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header
import pdb


class Projection:
    def __init__(self):
        rospy.init_node('reprojection_node', anonymous=True)

        self.bridge = CvBridge()

        #self.image_sub = rospy.Subscriber("/front_cam/image_raw", Image, self.image_callback)
        #self.pcd_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.pcd_callback)
        #self.image_pub = rospy.Publisher('/reprojected_image', Image, queue_size=1)
        self.image_sub = Subscriber("/front_cam/image_raw", Image)
        self.pcd_sub = Subscriber("/ouster/points", PointCloud2)
        self.image_pub = rospy.Publisher('/reprojected_image', Image, queue_size=1)

        # Add the synchronizer
        sync = ApproximateTimeSynchronizer([self.image_sub, self.pcd_sub], queue_size=30, slop=0.033)
        sync.registerCallback(self.sync_callback)
        #self.latest_img = None
        #self.latest_pcd = None

        rospy.spin()

    def sync_callback(self, img_msg, pcd_msg):
        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        pc_data = read_points(pcd_msg, field_names=("x", "y", "z"), skip_nans=True)
        pcd_points = np.array(list(pc_data))  # pcd_points's shape is (N,3)
        self.process_projection(img, pcd_points)

    def process_projection(self, img, pcd):
        self.project_lidar_to_camera(img, pcd)
        self.visualize_projection(img)
        
        
    def project_lidar_to_camera(self, img, pcd):
        # self.X_c = R*X_L + T
        self.X_c = np.dot(R, pcd.T).T + t  # N x 3 . camera 좌표계로 변경
        self.X_c = self.X_c[0 <= self.X_c[:, 2]]  # z 값이 0이상인 배열을 새로 만듬
        # self.X_c = self.X_c[self.X_c[:,2]]
        # pdb.set_trace()
        self.range_value = np.sqrt(self.X_c[:,0]**2+self.X_c[:,1]**2+self.X_c[:,2])
        # print(max(self.range_value))

        # X_depth = self.X_c[:, 2]
        self.max_depth = max(self.range_value)

        # a, b 는 normalized image, r은 principle axis와 포인트까지의 거리. theta는 입사각.
        self.X_c[:, 0] /= self.X_c[:, 2]  # a
        self.X_c[:, 1] /= self.X_c[:, 2]  # b
        self.X_c[:, 2] = np.sqrt(pow(self.X_c[:, 0], 2) + pow(self.X_c[:, 1], 2))  # r
        theta = np.arctan2(self.X_c[:, 2], 1)

        theta_d = theta * (1 + (D[0] * theta**2) + (D[1] * theta**4) + (D[2] * theta**6) + (D[3] * theta**8))

        # normalize & distortion points x, y
        x = (theta_d / self.X_c[:, 2]) * self.X_c[:, 0]
        y = (theta_d / self.X_c[:, 2]) * self.X_c[:, 1]

        # image distortion points u, v
        u = K[0][0] * x + K[0][2]
        v = K[1][1] * y + K[1][2]
        u = u.reshape((-1, 1))
        v = v.reshape((-1, 1))
        self.uv = np.concatenate((u, v), axis=1)

    def visualize_projection(self, img):
        # max_depth = self.max_depth # 지하주차장의 경우 0.05 왜 이러는진 모름
        # Visualization
        min_val = 0
        max_val = 15 # 삼차원상에서의 거리가 맥스가 거의 20정도 나왔다.
        for i, point in enumerate(self.uv):
            if self.X_c[i,2] <= 2.2:# z축으로 필터링해야 이상한 점들이 나오지 않음.
                out = int((self.range_value[i] - min_val) * (255 / (max_val - min_val)))
                color = (255-out, 0, out*2)  # 변경된 부분
                cv2.circle(img, (int(point[0]), int(point[1])), 2, color, -1)
        
        self.publish_image(img)

    def publish_image(self, img):
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.image_pub.publish(img_msg)

if __name__ == '__main__':
    # Define intrinsic and extrinsic parameters of the camera
    K = np.array([[602.4980536915167 * 0.98, 0.0, 1000.8923620425339 * 1.02],
                  [0.0, 599.73087785615, 587.7585563875641 * 1.045],
                  [0.0, 0.0, 1.0]])

    D = np.array([[-0.009951590108888648], [-0.13250497280054477], [0.35194285152672844], [-0.19495781377972635]])

    R = np.array([[-0.01340237, -0.9998429, 0.01159974],
                  [-0.00385881, -0.01154898, -0.99992586],
                  [0.99990274, -0.01344614, -0.00370342]])

    t = np.array([-0.0098, -0.2011, -0.2335])

    try:
        projection = Projection()
    except rospy.ROSInterruptException:
        pass

