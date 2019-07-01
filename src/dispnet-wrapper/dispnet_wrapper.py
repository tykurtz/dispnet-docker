#!/usr/bin/env python

import numpy as np
import rospy
import netdef_slim as nd
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DispnetWrapper:
    def __init__(self):
        c = nd.load_module('/home/netdef/netdef_models/DispNet3/CSS/controller.py').Controller()
        self.disp_net = c.net_actions(net_dir=c.base_path)
        self.bridge = CvBridge()

        self.images_queue = []

    def on_images_received(self, left_image, right_image):
        # Convert from sensor_msgs/Image to numpy
        # uint8 rgb
        # 480, 640, 3
        left_cv_image = self.bridge.imgmsg_to_cv2(left_image, "bgr8")
        right_cv_image = self.bridge.imgmsg_to_cv2(right_image, "bgr8")

        if len(self.images_queue) < 2:
            self.images_queue.append((left_cv_image, right_cv_image))

    def analyze_images(self):
        left_cv_image, right_cv_image = self.images_queue.pop()
        net_output = self.disp_net.eval(left_cv_image[np.newaxis, :, :, :],
                                        right_cv_image[np.newaxis, :, :, :])
        disparity_image = net_output['disp[0].fwd']

        self.publish_disparity(disparity_image)


    def publish_disparity(self, disparity_image):
        raise NotImplementedError


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
        if len(dispnet_wrapper.images_queue) > 1:
            dispnet_wrapper.analyze_images()
