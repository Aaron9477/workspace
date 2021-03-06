#!/usr/bin/env python

# --------------------------------------------------------
# Faster R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""

import cv2 
#go back the origin path
import sys
sys.path.append("/home/zq610/WYZ/py-faster-rcnn/tools/")
#default dependencies   
import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys
import argparse

import time

#ros dependencies
import rospy
from fasterrcnn2.msg import output
#ros2cv dependencies
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#CLASSES = ('__background__',
#           'aeroplane', 'bicycle', 'bird', 'boat',
#           'bottle', 'bus', 'car', 'cat', 'chair',
#           'cow', 'diningtable', 'dog', 'horse',
#           'motorbike', 'person', 'pottedplant',
#           'sheep', 'sofa', 'train', 'tvmonitor')
CLASSES = ('__background__', # must have a background
           'car', 't')

#NETS = {'vgg16': ('VGG16',
#                  'VGG16_faster_rcnn_final.caffemodel'),
#        'zf': ('ZF',
#                  'ZF_faster_rcnn_final.caffemodel')}
NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
                  'ZF_faster_rcnn_final.caffemodel')} # change the model name there

# declare global variable for ros2cv
bridge = CvBridge()
cv_frame = None


def vis_detections(im, class_name, dets, pub, thresh=0.8):
    """Draw detected bounding boxes."""
    inds = np.where(dets[:, -1] >= thresh)[0]
    if len(inds) == 0:
        return

    im = im[:, :, (2, 1, 0)]
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(im, aspect='equal')
    for i in inds:
        bbox = dets[i, :4]
        out_bb = output()
        for j in range(0,4):
            out_bb.output[j] = bbox[j]
        print (bbox[0], bbox[1], bbox[2], bbox[3])
        #pub.publish(out_bb)
        score = dets[i, -1]

		#there is no need to draw!!!!!!!!!!!!!!!!!
        ax.add_patch(
            plt.Rectangle((bbox[0], bbox[1]),
                          bbox[2] - bbox[0],
                          bbox[3] - bbox[1], fill=False,
                          edgecolor='red', linewidth=3.5)
            )
        ax.text(bbox[0], bbox[1] - 2,
                '{:s} {:.3f}'.format(class_name, score),
                bbox=dict(facecolor='blue', alpha=0.5),
                fontsize=14, color='white')

    ax.set_title(('{} detections with '
                  'p({} | box) >= {:.1f}').format(class_name, class_name,
                                                  thresh),
                  fontsize=14)
    plt.axis('off')
    plt.tight_layout()
    plt.draw()

	#ros pubilsh
	#test data
    #out_bb.output[0] = 120
    #out_bb.output[1] = 80
    #out_bb.output[2] = 240
    #out_bb.output[3] = 160
    while True:
        pub.publish(out_bb)


def demo(net, im, pub):
    """Detect object classes in an image using pre-computed object proposals."""

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(net, im)
    timer.toc()
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, boxes.shape[0]) # print the time spent

    # Visualize detections for each class
    CONF_THRESH = 0.5
    NMS_THRESH = 0.1
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1 # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        vis_detections(im, cls, dets, pub, thresh=CONF_THRESH)

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
#VGG16 is a very big net, so it will cost a lot of memory
    parser.add_argument('--net', dest='demo_net', help='Network to use [zf]',
                        choices=NETS.keys(), default='vgg16') # choose the zf network
    #parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16]',
                        #choices=NETS.keys(), default='vgg16')

    args = parser.parse_args()

    return args

def capture_vedio():
    capture = cv2.VideoCapture('/home/zq610/WYZ/video/TestVideo.mp4')
    success, frame = capture.read()
    return frame

def rawCallback(raw_image):
    try:
        global cv_frame
        cv_frame = bridge.imgmsg_to_cv2(raw_image, "bgr8")
        print("transfer ros_image to cv_image")
    except CvBridgeError as e:
        print (e)

if __name__ == '__main__':
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    args = parse_args()

    prototxt = os.path.join(cfg.MODELS_DIR, NETS[args.demo_net][0],
                            'faster_rcnn_alt_opt', 'faster_rcnn_test.pt') #caffemodel path
    caffemodel = os.path.join(cfg.DATA_DIR, '/home/zq610/WYZ/py-faster-rcnn/train_myself/test_model', NETS[args.demo_net][1])

    #caffemodel = os.path.join(cfg.DATA_DIR, 'faster_rcnn_models',
    #                          NETS[args.demo_net][1])

    if not os.path.isfile(caffemodel):
        raise IOError(('{:s} not found.\nDid you run ./data/script/'
                       'fetch_faster_rcnn_models.sh?').format(caffemodel))

    if args.cpu_mode:
        caffe.set_mode_cpu()
    else:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        cfg.GPU_ID = args.gpu_id
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    print '\n\nLoaded network {:s}'.format(caffemodel)

    # Warmup on a dummy image
    im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
    for i in xrange(2):
        _, _= im_detect(net, im)

    roi_pub = rospy.Publisher('fasterrcnn', output, queue_size=100)
    rawimage_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, rawCallback)
    rospy.init_node('fasterrcnn', anonymous=True)
    print("before sleep!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    global cv_frame
    while True:
        try:
            if cv_frame.all() == None:
                global cv_frame
                time.sleep(0.01)
            else:
                break
        except Exception as e:
            continue
    # while cv_frame.all() == None:
    #     global cv_frame
    #     time.sleep(0.01)
    rawimage_sub.unregister()
    print("before demo!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    demo(net, cv_frame, roi_pub)
    plt.show()
