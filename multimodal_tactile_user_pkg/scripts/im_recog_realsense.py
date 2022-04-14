#!/usr/bin/env python3.7
# Must be run from scripts folder
# Import Statements
import pyrealsense2 as rs
import numpy as np
import traceback
import argparse
import sys, os
import time
import cv2
from vision_recognition.detect import classifier
import torch
import matplotlib
matplotlib.use( 'tkagg' )
import matplotlib.pyplot as plt
from vision_recognition.finger_count import skinmask, getcnthull
from vision_recognition.shape_detector import rectangle_detector
from vision_recognition.blob_detector import detect_blobs
from vision_recognition.rs_cam import rs_cam

try:
    from pub_classes import diag_class, obj_class
    import rospy
    test=False
except ModuleNotFoundError:
    print(f"rospy module not found, proceeding in test mode")
    test = True
    # Hacky way of avoiding errors
    class ROS():
        def is_shutdown(self):
            return False
    rospy = ROS()

os.chdir(os.path.expanduser("~/catkin_ws/src/visuoTactileHRC/"))
sys.path.insert(0, "./visuoTactileHRC/scripts/vision_recognition") # Need to add path to "models" parent dir for pickler

def realsense_run():
    # ROS node setup
    frame_id = 'Realsense_node'

    if not test:
        rospy.init_node(frame_id, anonymous=True)
        diag_obj = diag_class(frame_id=frame_id, user_id=args.user_id, user_name=args.user_name, queue=1)
        rate = rospy.Rate(10)
    
    if args.classify:
        try:
            frames = cam.pipeline.wait_for_frames()
            if args.depth:
                color_image, depth_colormap, depth_image = cam.depth_frames(frames)
            else:
                color_image = cam.colour_frames(frames)

            im_classifier = classifier(args.comp_device, args.weights, args.img_size, color_image, args.conf_thres, args.iou_thres)
            if not test:
                obj_obj = obj_class(frame_id=frame_id, names=im_classifier.names, queue=1)
        
        except Exception as e:
            print("**Classifier Load Error**")
            traceback.print_exc(file=sys.stdout)
            if not test:
                diag_obj.publish(2, f"load classifier error: {e}")
            raise

    diag_timer = time.time()
    while (not rospy.is_shutdown()) or test:
        try:
            # Get frameset of color and depth
            frames = cam.pipeline.wait_for_frames()

            if args.depth:
                color_image_raw, depth_colormap, depth_image = cam.depth_frames(frames)
            else:
                color_image_raw = cam.colour_frames(frames)

            if args.classify:
                try:
                    # Run YOLOv3 classifier on image
                    if args.depth:
                        color_image, det = im_classifier.detect(color_image_raw, depth_image, depth_histogram=False)
                    else:
                        color_image, det = im_classifier.detect(color_image_raw, None)

                    try:
                        mask_img = skinmask(color_image_raw)
                        contours, hull = getcnthull(mask_img)
                        if cv2.contourArea(contours) > 1500:
                            det = torch.cat((det, torch.tensor([[0, 0, 0, 0, 1, 10, 0]])))
                            #cv2.drawContours(img, [contours], -1, (255,255,0), 2)
                            cv2.drawContours(color_image, [hull], -1, (0, 255, 255), 2)
                    except Exception as e:
                        print("hand mask error: ", e)
                        pass

                    try:
                        approx, thrash = rectangle_detector(color_image_raw)
                        if approx is not None:
                            cv2.drawContours(color_image, [approx], 0, (0, 255, 0), 5)
                            if not test:
                                obj_obj.publish(det)

                    except Exception as e:
                        print("rectangle error: ", e)
                        pass

                    try:
                        detect_blobs(thrash)
                    except Exception as e:
                        print("blob error: ", e)
                        pass

                    # if not test:
                    #     obj_obj.publish(det)
                except Exception as e:
                    print("**Classifier Detection Error**")
                    traceback.print_exc(file=sys.stdout)
                    if not test:
                        diag_obj.publish(2, f"load classifier error: {e}")

            # Remove background - Set pixels further than clipping_distance to grey
            #grey_color = 153
            #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            #bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            #image = scale(bg_removed)

            if  (time.time()-diag_timer) > 1:
                if not test:
                    diag_obj.publish(0, f"Running")
                diag_timer = time.time()

        except TypeError as e:
            time.sleep(1)
            if not test:
                diag_obj.publish(2, f"TypeError")
        except Exception as e:
            print("**Get Image Error**")
            if not test:
                diag_obj.publish(2, f"Realsense image error: {e}")
            traceback.print_exc(file=sys.stdout)
            break

        #im_screw_states, tally = im_screw_detect.detect_screws(image, args.disp)
        #im_screw_states = im_screw_states.tolist()
        

        if args.disp:
            if args.depth:
                disp_im = np.hstack((color_image, depth_colormap))
            else:
                disp_im = color_image
            
            cv2.namedWindow('Realsense viewer', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Realsense viewer', disp_im)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            
        if not test:
            rate.sleep()
        else:
            #time.sleep(1)
            pass


## Argument parsing
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Run realsense vision recognition ROS node')
    parser.add_argument('--disp', '-V',
                        help='Enable displaying of camera image',
                        default=True,
                        action="store")
    parser.add_argument('--depth', '-D',
                        help='Depth active',
                        default=True,
                        action="store_true")
    parser.add_argument('--user_name', '-N',
                    help='Set name of user, default: unknown',
                    default='N/A',
                    action="store_true")
    parser.add_argument('--user_id', '-I',
                    help='Set id of user, default: None',
                    default=0,
                    action="store_true")
    parser.add_argument('--classify', '-C',
                    help='Classify image',
                    default=True,
                    action="store_true")
    parser.add_argument('--comp_device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--img_size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--weights', nargs='+', type=str, default='best.pt', help='model.pt path(s)')
    parser.add_argument('--conf_thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou_thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--test', default=test, help='Test mode for visual recognition without ROS')
    
    args = parser.parse_known_args()[0]

    cam = rs_cam(args.disp)

    try:
        realsense_run()
    except rospy.ROSInterruptException:
        print("realsense_run ROS exception")
    except Exception as e:
        print("**Image Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        cam.pipeline.stop()
        cv2.destroyAllWindows()
        plt.close('all')
