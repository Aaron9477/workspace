#!/usr/bin/env python2
#!coding=utf-8
import rospy
from dronet_perception.msg import CNN_out
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
# python3
# from Dronet import utils
# python2
import utils

from keras import backend as K

TEST_PHASE=0

class Dronet(object):
    def __init__(self,
                 json_model_path,
                 weights_path, target_size=(200, 200),
                 crop_size=(150, 150),
                 imgs_rootpath="../models"):

        self.pub = rospy.Publisher("cnn_predictions", CNN_out, queue_size=5)
        # self.feedthrough_sub = rospy.Subscriber("state_change", Bool, self.callback_feedthrough, queue_size=1)
        # self.land_sub = rospy.Subscriber("land", Empty, self.callback_land, queue_size=1)

        # 用于调节是否使用网络进行控制,之后需要用指令修改这个参数,此处默认为True
        self.use_network_out = True
        self.imgs_rootpath = imgs_rootpath

        # Set keras utils
        # 这里设定网络参数是否更新
        K.set_learning_phase(TEST_PHASE)

        # Load json and create model
        model = utils.jsonToModel(json_model_path)
        # Load weights
        model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))

        model.compile(loss='mse', optimizer='sgd')  # 模型参数设定
        self.model = model
        self.target_size = target_size
        self.crop_size = crop_size

    # # 指令控制是否使用网络
    # def callback_feedthrough(self, data):
    #     self.use_network_out = data.data
    # # 无人机降落指令
    # def callback_land(self, data):
    #     self.use_network_out = False

    def run(self):
        while not rospy.is_shutdown():
            msg = CNN_out()
            msg.header.stamp = rospy.Time.now()
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message("camera", Image, timeout=10)
                except:
                    pass

            # if self.use_network_out:
            #     print("Publishing commands!")
            # else:
            #     print("NOT Publishing commands!")

            cv_image = utils.callback_img(data, self.target_size, self.crop_size,
                self.imgs_rootpath, self.use_network_out)
            outs = self.model.predict_on_batch(cv_image[None])
            steer, coll = outs[0][0], outs[1][0]
            msg.steering_angle = steer
            msg.collision_prob = coll
            if self.use_network_out:
                print("the steer is %f"%steer)
                print("the coll is %f"%coll)
            else:
                print("NOT Publishing commands!")
            self.pub.publish(msg)
