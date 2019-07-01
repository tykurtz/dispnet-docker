#!/usr/bin/env python

import numpy as np
import rospy
import time
import netdef_slim as nd
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DispnetWrapper:
    def __init__(self):
        c = nd.load_module('/home/netdef/netdef_models/DispNet3/css/controller.py').Controller()
        self.disp_net = c.net_actions(net_dir=c.base_path)
        self.bridge = CvBridge()

        self.images_queue = []
        self.disparity_publisher = rospy.Publisher('/nn_disp_out', Image)

    def on_images_received(self, left_image, right_image):
        # Convert from sensor_msgs/Image to numpy
        # uint8 rgb
        # 480, 640, 3
        left_cv_image = self.bridge.imgmsg_to_cv2(left_image, "bgr8")
        right_cv_image = self.bridge.imgmsg_to_cv2(right_image, "bgr8")

        if len(self.images_queue) < 1:
            self.images_queue.append((left_cv_image, right_cv_image))

    def analyze_images(self):
        left_cv_image, right_cv_image = self.images_queue.pop()

        start = time.time()
        net_output = self.disp_net.eval(left_cv_image.transpose(2, 0, 1)[np.newaxis, :, :, :],
                                        right_cv_image.transpose(2, 0, 1)[np.newaxis, :, :, :])
        # keys(['disp.L', 'occ.L', 'db.L', 'db_soft.L', 'occ_soft.L'])
        print('Analysis time : ', time.time() - start)

        disparity_image = net_output['disp.L']
        # Disparity image shape = (1, 1, 480, 640)
        print('Max and min : ', disparity_image.max(), disparity_image.min())

        # Rescale for image view
        disparity_image = disparity_image[0, 0, :, :]
        disparity_image = disparity_image + disparity_image.min()
        disparity_image = disparity_image / disparity_image.max()

        image_message = self.bridge.cv2_to_imgmsg(disparity_image)
        self.disparity_publisher.publish(image_message)


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
