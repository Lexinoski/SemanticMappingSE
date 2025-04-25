#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import os

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32MultiArray
from rospy.numpy_msg import numpy_msg

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

devide = 0
half = 0

imgsz = 0
old_img_w = old_img_h = imgsz
old_img_b = 1

model = 0
names = 0
colors = 0


pub = 0
rate = 0

def detect(save_img=False):
    weights, imgsz1, trace =  opt.weights, opt.img_size, not opt.no_trace

    # Initialize
    global device
    global half
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    global model
    global stride
    global imgsz
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz1, s=stride)  # check img_size

    if trace:
        model = TracedModel(model, device, opt.img_size)

    if half:
        model.half()  # to FP16


    # Get names and colors
    global names
    global colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    global old_img_w
    global old_img_h
    global old_img_b

    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    old_img_w = old_img_h = imgsz
    old_img_b = 1


# A equivalent CvBridge Function
def myCvBridge(img_msg):
# # Making a auxiliary function
    hei = img_msg.height
    wid = img_msg.width
    dat = img_msg.data
    stp = img_msg.step
    enc = img_msg.encoding

    image_eq_cv = np.zeros((hei, wid, int(stp/wid)), dtype = 'uint8')
    cont = 0
    for i in range(0, hei):
        for j in range(0, wid):
            for k in range(0, int(stp/wid)):
                image_eq_cv[i, j, 2-k] = int(dat[cont])
                cont += 1

    return image_eq_cv


def publish_bouding_boxes():
    
    global device
    global half
    global model
    global names
    global colors
    global old_img_w
    global old_img_h
    global old_img_b

    # Set 1 to view results
    view_img = 1

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('Publish_BB', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Create publishers
    bb_pub = rospy.Publisher('detections', Float32MultiArray, queue_size=10)
    
    while True:
        ## Start the detections by yolo v7
        t0 = time.time()

        # Wait a response from topic
        img_msg = rospy.wait_for_message("/kinect/rgb_image", Image)
            
        # log some info about the image topic
        # print(img_msg.data)    
        # rospy.loginfo(img_msg.header)

        eq_cv_image = myCvBridge(img_msg=img_msg)
        # bridge = CvBridge()                                                           # Dispensed
        # # Convert the ROS Image message to a CV2 Image                                # Dispensed
        # cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')      # Dispensed

        flip_vertical = cv2.flip(eq_cv_image, 0)
        name = 'SE_lacbot'

        # ## Start the detections by yolo v7
        # t0 = time.time()

        # Reshape for img
        img = np.zeros((3, 480, 640), dtype='uint8')
        img[0,:,:] = flip_vertical[:,:,0]       
        img[1,:,:] = flip_vertical[:,:,1]
        img[2,:,:] = flip_vertical[:,:,2]

        im0s = flip_vertical
     
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        # print(img.shape)        
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment=opt.augment)[0]
        
        
        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t3 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image

            s, im0, = '', im0s

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                # # Write results
                # for *xyxy, conf, cls in reversed(det):

                #     if view_img:  # Add bbox to image
                #         label = f'{names[int(cls)]} {conf:.2f}'
                #         plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=2)


            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Publish detections infos per image
            # print(type(det))
            my_msg = Float32MultiArray()

            d = list(det.tolist())  
            # print(f'det_list2D = {d}')                  
            d=[float(d[i][j]) for i in range(len(d)) for j in range(len(d[0]))]
            # print(f'det_list = {d}')
            my_msg.data = d
            
            # rospy.loginfo(my_msg)
            bb_pub.publish(my_msg)
            rate.sleep()

            # Stream results
            if view_img:

                try:
                    # Try to receive the message within the time limit
                    depths = rospy.wait_for_message("/depths_BB", Float32MultiArray, timeout=1)
                    distances = depths.data
                except rospy.exceptions.ROSException:
                    # If the time limit is reached, print a warning message
                    rospy.logwarn("Time limit reached. Unable to receive the message within %d seconds.", 1)
                    # Here you can add the code to proceed even if the message is not received
                    distances = tuple([np.nan] * len(d))  # Or any other value you wish to assign if the message is not received


                # depths = rospy.wait_for_message("/depths_BB", Float32MultiArray)
                # distances = depths.data
                # print(distances)   
   
                # Write results
                cont = 0
                # for *xyxy, conf, cls in reversed(det):
                for *xyxy, conf, cls in det:

                    ## Set bounding box upper bound
                    xyxy[1] = max([20, xyxy[1]])
                    
                    if view_img:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}, {distances[cont]:.2f}m'
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=2)
                        cont+=1

                # cv2.imshow(str(p), im0)
                cv2.imshow('SE_lacbot', im0)
                cv2.waitKey(1)  # 1 millisecond


        print(f'Done. ({time.time() - t0:.3f}s)')


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default='best_english.pt', help='model.pt path(s)')
        parser.add_argument('--source', type=str, default='robot', help='source')  # file/folder, 0 for webcam
        parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
        parser.add_argument('--conf-thres', type=float, default=0.7, help='object confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
        parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--view-img', action='store_true', help='display results')
        parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
        parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
        parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--augment', action='store_true', help='augmented inference')
        parser.add_argument('--update', action='store_true', help='update all models')
        parser.add_argument('--project', default='runs/detect', help='save results to project/name')
        parser.add_argument('--name', default='exp', help='save results to project/name')
        parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
        parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
        opt = parser.parse_args()
        print(opt)
        #check_requirements(exclude=('pycocotools', 'thop'))

        detect()
        # talker()
        publish_bouding_boxes()


    except rospy.ROSInterruptException:
        pass



