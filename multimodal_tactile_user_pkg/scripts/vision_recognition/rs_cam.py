import pyrealsense2 as rs
import numpy as np
import cv2

class rs_cam:
    def __init__(self, depth):
        # Create a pipeline
        self.pipeline = rs.pipeline()
        #Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if depth:
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start streaming
        profile = self.pipeline.start(config)

        if depth:
            # Getting the depth sensor's depth scale (see rs-align example for explanation)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_sensor.set_option(rs.option.visual_preset, 2)
            self.depth_scale = depth_sensor.get_depth_scale()
            print("Depth Scale is: " , self.depth_scale)
            # We will be removing the background of objects more than
            #  clipping_distance_in_meters meters away
            #clipping_distance_in_meters = 0.5 #1 meter
            #clipping_distance = clipping_distance_in_meters / depth_scale
            # Create an align object
            # rs.align allows us to perform alignment of depth frames to others frames
            # The "align_to" is the stream type to which we plan to align depth frames.
            align_to = rs.stream.color
            self.align = rs.align(align_to)

            self.depth_intrinsics = None

    def colour_frames(self, frames):
        color_frame = frames.get_color_frame()
        # Validate that both frames are valid
        if not color_frame:
            return
        color_image = np.asanyarray(color_frame.get_data())

        return color_image

    def depth_frames(self, frames):
        #frames.get_depth_frame() is a 640x360 depth image
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        self.depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return
        depth_image = np.asanyarray(aligned_depth_frame.get_data())*self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=100), cv2.COLORMAP_JET)

        return color_image, depth_colormap, depth_image

    def scale(self, image): # zooms on image
        height, width, channels = image.shape
        scale = 2  # x? digital zoom
        centerX, centerY = int(height / 2), int(width / 2)
        radiusX, radiusY = int(height / (scale * 2)), int(width / (scale * 2))
        minX, maxX = centerX - radiusX, centerX + radiusX
        minY, maxY = centerY - radiusY, centerY + radiusY
        image = cv2.resize(image[minX:maxX, minY:maxY], (width, height), interpolation=cv2.INTER_LINEAR)
        return image
