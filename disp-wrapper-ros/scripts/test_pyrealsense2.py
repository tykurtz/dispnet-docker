#!/usr/bin/env python

import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.infrared, 0)
# config.enable_stream(rs.stream.infrared, 1)
config.enable_all_streams()
pipe_profile = pipeline.start(config)
streams = pipe_profile.get_streams()

ir1_stream = streams[1]
ir2_stream = streams[2]

print(ir1_stream.get_extrinsics_to(ir2_stream))
