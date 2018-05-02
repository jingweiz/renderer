from __future__ import absolute_import
from __future__ import division

import numpy as np
from copy import deepcopy
from gym.spaces.box import Box
import inspect
import os, subprocess
import cv2
import time
import timeit
import rospy, rosgraph
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils.helpers import preprocessAtari, rgb2gray, rgb2y, scale, center_crop_rgb, center_crop_depth
from core.env import Env
import math
from PIL import Image
import matplotlib.pyplot as plt

import torch
import torchvision
from torchvision import datasets, models, transforms

class RendererEnv(Env):
    def __init__(self, args, env_ind=0):
        super(RendererEnv, self).__init__(args, env_ind)
        self.env_ind = env_ind
        self.enable_continuous = args.enable_continuous

        # TODO: use this for training in docker on remotes
        # os.environ["ROS_MASTER_URI"] = "http://rosimage"+str(self.env_ind)+":11311"
        # TODO: use this for local test
        os.environ["ROS_MASTER_URI"] = "http://localhost:11311"

        time.sleep(10)
        self.root_dir = args.root_dir

        try:
            from renderer.srv import Render
        except ImportError as e: self.logger.warning("WARNING: renderer not found")

        # set up service
        self.render_srv = rospy.ServiceProxy('/render', Render)

        self.hei_state = args.hei_state
        self.wid_state = args.wid_state
        self.preprocess_mode = args.preprocess_mode
        self.img_encoding_type = args.img_encoding_type

        self.bridge = CvBridge()

    def __del__(self):
        self.render_service.close()

    @property
    def action_dim(self):
        # NOTE: in /render service,
        # NOTE: 0: go_straight | 1 : turn_left | 2: turn_right
        # NOTE: 3: reset the game
        return 3

    @property
    def state_shape(self):
        return (self.hei_state, self.wid_state)

    def _preprocessState(self, state):    # called before _preprocessState in A3CSingleProcess
        img = self.bridge.imgmsg_to_cv2(state, self.img_encoding_type)
        # # visualize img
        # plt.figure(); plt.imshow(img); plt.show(); raw_input()

        if self.preprocess_mode == 4:   # for rgb images: do not convert to grayscale
            img = 2 * (scale(img, self.hei_state, self.wid_state) / 255. - 0.5)  # (90, 160, 3) (-1, 1)
            img = np.transpose(img, (2, 0, 1))   # (3, 90, 160)

        return img

    def step(self, exp_action):
        # # in test phase, for synchronization
        # time.sleep(0.1)
        try:
            # TODO: use this for training in docker on remotes
            # os.environ["ROS_MASTER_URI"] = "http://rosimage"+str(self.env_ind)+":11311"
            # TODO: use this for local test
            os.environ["ROS_MASTER_URI"] = "http://localhost:11311"

            render_res = self.render_srv(exp_action)
            self.exp_state1 = render_res.color
            self.exp_reward = render_res.reward
            self.exp_terminal1 = render_res.terminal
        except rospy.ServiceException, e:
            print("Service call failed during step: %s" %e)
        return self._get_experience()

    def reset(self):
        return self.step(self.action_dim)
