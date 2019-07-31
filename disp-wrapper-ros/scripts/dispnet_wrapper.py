#!/usr/bin/env python

import numpy as np
import rospy
import time
import netdef_slim as nd
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError


class DispnetWrapper:
    def __init__(self):
        c = nd.load_module('/home/netdef/netdef_models/DispNet3/css/controller.py').Controller()
        self.disp_net = c.net_actions(net_dir=c.base_path)
        self.bridge = CvBridge()

        self.images_queue = []
        self.disparity_image_publisher = rospy.Publisher('/nn/debug', Image)
        self.disparity_publisher = rospy.Publisher('/nn/depth', DisparityImage)
        self.right_camera_info_subscriber = rospy.Subscriber('/camera/infra2/camera_info', CameraInfo, self.on_camera_info_received)
        self.right_camera_info_publisher = rospy.Publisher('/camera/infra2/modified_camera_info', CameraInfo)

    def on_images_received(self, left_image, right_image):
        if len(self.images_queue) < 1:
            self.images_queue.append((left_image, right_image))

    def on_camera_info_received(self, camera_info):
        modified_camera_info = camera_info
        modified_camera_info.header.frame_id = 'camera_infra1_optical_frame'
        projection_matrix = list(modified_camera_info.P)
        projection_matrix[3] = -projection_matrix[0] * .0499728  # As we're switching frames, modify the baseline as well
        modified_camera_info.P = projection_matrix

        self.right_camera_info_publisher.publish(modified_camera_info)

    def analyze_images(self):
        left_image, right_image = self.images_queue.pop()

        # Convert from sensor_msgs/Image to numpy
        # uint8 rgb
        # 480, 640, 3
        left_cv_image = self.bridge.imgmsg_to_cv2(left_image, "bgr8")
        right_cv_image = self.bridge.imgmsg_to_cv2(right_image, "bgr8")

        start = time.time()
        net_output = self.disp_net.eval(left_cv_image.transpose(2, 0, 1)[np.newaxis, :, :, :],
                                        right_cv_image.transpose(2, 0, 1)[np.newaxis, :, :, :])
        # keys(['disp.L', 'occ.L', 'db.L', 'db_soft.L', 'occ_soft.L'])
        print('Analysis time : ', time.time() - start)

        disparity_image = net_output['disp.L']
        print('Max and min : ', disparity_image.max(), disparity_image.min())

        # Disparity image shape = (1, 1, 480, 640)
        disparity_image = disparity_image[0, 0, :, :]

        # Rescale for image view
        debug_disparity_image = disparity_image + -75  # disparity_image.min()
        debug_disparity_image = debug_disparity_image / 80  # debug_disparity_image.max()

        debug_image_message = self.bridge.cv2_to_imgmsg(debug_disparity_image)
        self.disparity_image_publisher.publish(debug_image_message)

        image_message = self.bridge.cv2_to_imgmsg(disparity_image)

        disparity_message = DisparityImage()
        disparity_message.image = image_message
        disparity_message.header = left_image.header
        disparity_message.T = .0499728  # meters
        disparity_message.f = 382.9233093261719  # pixels

        disparity_message.min_disparity = -300
        disparity_message.max_disparity = 600
        disparity_message.delta_d = .001

        self.disparity_publisher.publish(disparity_message)


if __name__ == '__main__':
    rospy.init_node('dispnet_wrapper')
    dispnet_wrapper = DispnetWrapper()

    # left_image_sub = message_filters.Subscriber('/stereo/left/image_raw', Image)
    # right_image_sub = message_filters.Subscriber('/stereo/right/image_raw', Image)
    left_image_sub = message_filters.Subscriber('/camera/infra1/image_rect_raw', Image)
    right_image_sub = message_filters.Subscriber('/camera/infra2/image_rect_raw', Image)
    time_sync = message_filters.TimeSynchronizer([left_image_sub,
                                                  right_image_sub], 1)

    time_sync.registerCallback(dispnet_wrapper.on_images_received)
    while not rospy.is_shutdown():
        if len(dispnet_wrapper.images_queue) > 0:
            dispnet_wrapper.analyze_images()
