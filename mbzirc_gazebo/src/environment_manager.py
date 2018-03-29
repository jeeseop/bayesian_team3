#!/usr/bin/env python

import rospy
import sys
import random
import rospkg
import math
import tf
from subprocess import call
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

class Environment:
    MODEL_TARGET_PATH = '/models/target'
    MODEL_DISC_PATH = '/models/disc'
    MODEL_STAND_PATH = '/models/stand'
    DISC_COLOR = 'disc_color'

    def __init__(self, service_proxy, package_directory, center_x, center_y, radius_disc):
        self.service_proxy = service_proxy
        self.package_directory = package_directory
        self.center_x = center_x
        self.center_y = center_y
        self.radius_disc = radius_disc
        self.disc_id = 0

    def spawn_discs(self):
        red_disc_model = open("{0}/{1}/model_red.sdf".format(self.package_directory, self.MODEL_DISC_PATH), 'r').read()
        green_disc_model = open("{0}/{1}/model_green.sdf".format(self.package_directory, self.MODEL_DISC_PATH), 'r').read()
        blue_disc_model = open("{0}/{1}/model_blue.sdf".format(self.package_directory, self.MODEL_DISC_PATH), 'r').read()
        yellow_disc_model = open("{0}/{1}/model_yellow.sdf".format(self.package_directory, self.MODEL_DISC_PATH), 'r').read()
        orange_disc_model = open("{0}/{1}/model_orange.sdf".format(self.package_directory, self.MODEL_DISC_PATH), 'r').read()
        stand_model = open("{0}/{1}/model.sdf".format(self.package_directory, self.MODEL_STAND_PATH), 'r').read()
        for i in range(0, 4):
            self.spawn_disc(red_disc_model, stand_model)
        for i in range(0, 4):
            self.spawn_disc(green_disc_model, stand_model)
        for i in range(0, 4):
            self.spawn_disc(blue_disc_model, stand_model)
        for i in range(0, 4):
            self.spawn_disc(yellow_disc_model, stand_model)
        for i in range(0, 4):
            self.spawn_disc(orange_disc_model, stand_model)

    def spawn_disc(self, disc_model, stand_model):
        initial_pose = Pose()
        min_x = self.center_x - self.radius_disc
        min_y = self.center_y - self.radius_disc
        max_x = self.center_x + self.radius_disc
        max_y = self.center_y + self.radius_disc
        initial_pose.position.x = random.uniform(min_x, max_x)
        initial_pose.position.y = random.uniform(min_y, max_y)
        initial_pose.position.z = 0.004
        quaternion = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0, 2 * math.pi))
        initial_pose.orientation.x = quaternion[0]
        initial_pose.orientation.y = quaternion[1]
        initial_pose.orientation.z = quaternion[2]
        initial_pose.orientation.w = quaternion[3]
        self.service_proxy('stand' + str(self.disc_id), stand_model, 'discs', initial_pose, 'world')
        initial_pose.position.z += 0.2035
        self.service_proxy('disc' + str(self.disc_id), disc_model, 'discs', initial_pose, 'world')
        self.disc_id += 1

    def generate_disc_models(self):
        model_directory = self.package_directory + self.MODEL_DISC_PATH
        self.generate_disc(model_directory, 'red')
        self.generate_disc(model_directory, 'green')
        self.generate_disc(model_directory, 'blue')
        self.generate_disc(model_directory, 'yellow')
        self.generate_disc(model_directory, 'orange')

    def generate_disc(self, model_directory, disc_color):
        model_template = model_directory + '/model.sdf.erb'
        command = ' '.join([
            'erb',
            "{0}={1}".format(self.DISC_COLOR, disc_color),
            model_template, '>', "{0}/model_{1}.sdf".format(model_directory, disc_color)
        ])
        print command
        call(command, shell=True)

if len(sys.argv) == 6:
    center_x = float(sys.argv[1])
    center_y = float(sys.argv[2])
    radius_disc = float(sys.argv[3])
    rospy.init_node('environment_manager')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    service_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    package_directory = rospkg.RosPack().get_path('mbzirc_gazebo')
    environment = Environment(service_proxy, package_directory, center_x, center_y, radius_disc)
    environment.generate_disc_models()
    environment.spawn_discs()
    rospy.spin()
