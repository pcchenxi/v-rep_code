#!/usr/bin/env python
import sys, os
sys.path.append("../../v-rep_plugin") 
import numpy as np

## v-rep
import vrep

import matplotlib.pyplot as plt

action_list = []
STATE_SIZE = 1

for a in range(-1, 2):
    for b in range(-1, 2):
        # for c in range(-1, 2):
            # for d in range(-1, 2):
            #     for e in range(-1, 2):
        action = []
        action.append(0)
        action.append(b)
        action.append(a)
        action.append(0)
        action.append(0)
        # print action
        action_list.append(action)
        # print action_list

# print action_list


class Simu_env:
    def __init__(self, port_num):
        # super(Vrep_env, self).__init__(port_num)
        self.action_space = ['l', 'f', 'r', 'h', 'e']
        self.n_actions = len(self.action_space)
        # self.n_features = 2
        # self.title('Vrep_env')
        self.port_num = port_num
        self.reached_index = -1
        self.dist_pre = 100
        self.state_size = 3
        # self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        # self.clientID = self._connect_vrep(port_num)


    def connect_vrep(self, close_all = False):
        if close_all:
            vrep.simxFinish(-1) # just in case, close all opened connections

        clientID = vrep.simxStart('127.0.0.1',self.port_num,True,True,5000,5)
        if clientID != -1:
            print 'Connected to remote API server with port: ', self.port_num, close_all
        else:
            print 'Failed connecting to remote API server'

        self.clientID = clientID
        # return clientID

    def disconnect_vrep(self):
        vrep.simxFinish(self.clientID)
        print ('Program ended')


    ########################################################################################################################################
    ###################################   interface function to communicate to the simulator ###############################################
    def call_sim_function(self, object_name, function_name, input_floats=[]):
        inputInts = []
        inputFloats = input_floats
        inputStrings = []
        inputBuffer = bytearray()
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.clientID, object_name,vrep.sim_scripttype_childscript,
                    function_name, inputInts, inputFloats, inputStrings,inputBuffer, vrep.simx_opmode_blocking)

        # print 'function call: ', self.clientID

        return res, retInts, retFloats, retStrings, retBuffer
        
    def get_laser_points(self):
        res,retInts,retFloats,retStrings,retBuffer = self.call_sim_function('LaserScanner_2D', 'get_laser_points')
        return retFloats

    def get_global_path(self):
        res,retInts, path_raw, retStrings, retBuffer = self.call_sim_function('rwRobot', 'get_global_path')
        path_dist = []
        path_angle = []

        for i in range(2, len(path_raw), 2):       
            path_dist.append(path_raw[i])
            path_angle.append(path_raw[i+1])
        
        return path_dist, path_angle


    def convert_state(self, laser_points, current_pose, path):
        state = np.asarray(path)
        state = state.flatten()
        # print state

        return state

        
    def reset(self):
        self.reached_index = -1
        res,retInts,retFloats,retStrings,retBuffer = self.call_sim_function('rwRobot', 'reset')
        return self.step([0,0,0,0,0])

    def step(self, action):
    #     print 'old index: ', self.reached_index
        if len(action) == 1:
            action = action_list[action[0]]

        is_finish = False
        reward = 0

        res,retInts,current_pose,retStrings,found_pose = self.call_sim_function('rwRobot', 'step', action)
        laser_points = self.get_laser_points()
        path_dist, path_angle = self.get_global_path()
        
        #compute reward
        ###
        # reward, self.reached_index = self.compute_reward(action, path_dist, found_pose)
        
        # if self.reached_index > len(path_dist)-2:
        #     is_finish = True
        #     reward = reward + 10
        # ###


        path_f = []
        for i in range(STATE_SIZE):
            # index = self.reached_index+1+i
            index = len(path_dist) - 1
            if index > len(path_dist) - 1:
                sub_path = [0, 0]
                path_f.append(sub_path)
            else:
                sub_path = [path_dist[index], path_angle[index]]
                path_f.append(sub_path)

        # sub_path = [path_dist[-1], path_angle[-1]]
        # path_f.append(sub_path)
        state_ = self.convert_state(laser_points, current_pose, path_f)


###################################################################
        if path_f[0][0] < self.dist_pre:
            reward = 1
        else:
            reward = -5

        self.dist_pre = path_f[0][0]
        # reward = -path_f[0][0]
        if path_f[0] < 0.1:
            is_finish = True

        return state_, reward, is_finish, ''

    ########################################################################################################################################

    ###################################################  reward function ###################################################################
    def compute_reward(self, action, path_dist, found_pose):
        reward = -0.1 #- np.sum(np.absolute(action))*0.1
        closest_index = np.argmin(path_dist)
        if path_dist[closest_index] < 0.15 and self.reached_index < closest_index:
            reward = reward + 1
            self.reached_index = closest_index
        # # print action, np.sum(np.absolute(action)), reward

        # if path_dist[0] < self.dist_pre:
        #     reward = reward + 1
        # self.dist_pre = path_dist[0]

        if found_pose == 'f':
            reward = reward - 5

        return reward, self.reached_index