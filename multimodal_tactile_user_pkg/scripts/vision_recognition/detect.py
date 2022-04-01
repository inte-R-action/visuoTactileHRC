# Adapted from https://github.com/ultralytics/yolov3#pretrained-checkpoints
#!/usr/bin/env python3
import argparse
import time
from pathlib import Path
import tensorflow as tf
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import os
from vision_recognition.models.experimental import attempt_load
from vision_recognition.utils.datasets import LoadStreams, LoadImages
from vision_recognition.utils.general import check_img_size, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, \
    strip_optimizer, set_logging, increment_path
from vision_recognition.utils.plots import plot_one_box
from vision_recognition.utils.torch_utils import select_device, load_classifier, time_synchronized
import numpy as np
import math
from scipy import stats
import matplotlib
matplotlib.use( 'tkagg' )
import matplotlib.pyplot as plt

class classifier():
    def __init__(self, comp_device, weights, imgsz, img, conf_thres, iou_thres):
        #img = tf.make_ndarray(img)

        #source, weights, view_img, save_txt, imgsz = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
        #webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        #   ('rtsp://', 'rtmp://', 'http://')) or source.endswith('rs')

        # Directories
        #save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
        #(save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Initialize
        #set_logging()
        self. conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.device = select_device(comp_device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        self.img_size = imgsz
        # Load model
        #result = []
        #for root, dirs, files in os.walk(path):
        #    if weights in files:
        #        result.append(os.path.join(root, name))
        dir = os.path.dirname(__file__)
        print(dir+'/'+weights)
        self.model = attempt_load(dir+'/'+weights, map_location=self.device)  # load FP32 model
        imgsz = check_img_size(imgsz, s=self.model.stride.max())  # check img_size
        if self.half:
            self.model.half()  # to FP16

        # Set Dataloader
        vid_path, vid_writer = None, None
        self.view_img = False
        cudnn.benchmark = True  # set True to speed up constant image size inference
        #dataset = LoadStreams(source, img_size=imgsz)


        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        print(self.names)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        self.t0 = time.time()
        #img = torch.zeros((1, 3, imgsz, imgsz), device=self.device)  # init img
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        
        self.imgs = [None] * 1
        self.imgs[0] = img
        # check for common shapes
        s = np.stack([self.letterbox(x, new_shape=self.img_size)[0].shape for x in self.imgs], 0)  # inference shapes
        self.rect = np.unique(s, axis=0).shape[0] == 1  # rect inference if all shapes equal
        if not self.rect:
            print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')

    def detect(self, img, depth_image=None, depth_histogram=False):
        im0s = [img.copy()]
        #if cv2.waitKey(1) == ord('q'):  # q to quit
        #    cv2.destroyAllWindows()
        #    raise StopIteration

        # Letterbox
        img = [self.letterbox(x, new_shape=self.img_size, auto=self.rect)[0] for x in im0s]
        # Stack
        img = np.stack(img, 0)
        # Convert
        img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to bsx3x416x416
        img = np.ascontiguousarray(img)

        #path = self.sources

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment=None)[0]#opt.augment)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=None, agnostic=None)
        t2 = time_synchronized()

        # Apply Classifier
        #if classify:
        #    pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections
        save_txt = False
        save_img = False

        for i, det in enumerate(pred):  # detections per image
            s, im0 = '%g: ' % i, im0s[i].copy()

            #save_path = str(save_dir / p.name)
            #txt_path = str(save_dir / 'labels' / p.stem) + ('_%g' % dataset.frame if dataset.mode == 'video' else '')
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            dist = []
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()                    
                    
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, self.names[int(c)])  # add to string
                
                
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    # Get average distance to object bounding box
                    if depth_image is not None:
                        dist_mat = np.around(np.nan_to_num(depth_image[xyxy[1].int():xyxy[3].int(), xyxy[0].int():xyxy[2].int()]), decimals=3)
                        dist_mat = np.reshape(dist_mat, (-1,))
                        dist_mat = np.ma.masked_equal(dist_mat, 0)
                        #dist.append(np.nanmean(dist_mat))
                        #dist.append(np.mean(dist_mat))
                        #dist.append(np.ma.mean(dist_mat))
                        dist.append(dist_mat.mean())
                    else:
                        dist.append(0)

                    #print(self.names[int(cls)], conf, dist[-1])
                    if np.ma.is_masked(dist[-1]) == False:
                        if math.isnan(dist[-1]) == False:
                            dist[-1] = round(dist[-1], 2)
                    else:      
                        dist[-1] = float("nan")
                    label = '%s %.2f %s m' % (self.names[int(cls)], conf, dist[-1])

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    #if save_img or self.view_img:  # Add bbox to image
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=1)
                
                if depth_histogram:
                    plt.cla()
                    plt.title(self.names[int(cls)])
                    plt.hist(dist_mat.ravel(), np.arange(np.min(dist_mat), np.max(dist_mat), 0.01).tolist(), range=(np.min(dist_mat), np.max(dist_mat)))
                    plt.show(block=False)
                    plt.pause(0.0001)

                #if dist:
                #    dist = torch.reshape(torch.Tensor(dist), (-1, 1))
            dist = torch.reshape(torch.Tensor(dist), (-1, 1))
            det = torch.cat((det, dist), 1)
            # Print time (inference + NMS)
            #print('%sDone. (%.3fs)' % (s, t2 - t1))

            # Stream results
            if self.view_img:
                cv2.imshow('realsense', im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            
            # if save_img:
            #     if dataset.mode == 'images':
            #         cv2.imwrite(save_path, im0)
            #     else:
            #         if vid_path != save_path:  # new video
            #             vid_path = save_path
            #             if isinstance(vid_writer, cv2.VideoWriter):
            #                 vid_writer.release()  # release previous video writer

            #             fourcc = 'mp4v'  # output video codec
            #             fps = vid_cap.get(cv2.CAP_PROP_FPS)
            #             w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            #             h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            #             vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
            #         vid_writer.write(im0)

        if save_txt or save_img:
            s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
            print(f"Results saved to {save_dir}{s}")

        #print('Done. (%.3fs)' % (time.time() - self.t0))

        return im0, reversed(det)

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
        # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='yolov3.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='data/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    print(opt)

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov3.pt', 'yolov3-spp.pt', 'yolov3-tiny.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()
