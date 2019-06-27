#!/usr/bin/env python

import rospy
import netdef_slim as nd
import message_filters
from sensor_msgs.msg import Image


class DispnetWrapper:
    def __init__(self):
        c = nd.load_module('/home/netdef/netdef_models/DispNet3/CSS/controller.py').Controller()
        self.disp_net = c.net_actions(net_dir=c.base_path)


    def analyze_images(self, left_image, right_image):
        net_output = self.disp_net.eval([left_image, right_image])
        disparity_image = net_output['disp[0].fwd']


    def publish_disparity(self, disparity_image):
        raise NotImplementedError


if __name__ == '__main__':
    rospy.init_node('dispnet_wrapper')
    dispnet_wrapper = DispnetWrapper()

    left_image_sub = message_filters.Subscriber('/stereo/left/image_raw', Image)
    right_image_sub = message_filters.Subscriber('/stereo/right/image_raw', Image)
    time_sync = message_filters.TimeSynchronizer([left_image_sub,
                                                  right_image_sub], 1)

    time_sync.registerCallback(dispnet_wrapper.analyze_images)
    rospy.spin()
