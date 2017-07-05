#!/usr/bin/env python
import sys, os
sys.path.append("../../v-rep_plugin") 
import numpy as np

## v-rep
import vrep

import matplotlib.pyplot as plt

action_list = []

for a in range(-1, 2):
    for b in range(-1, 2):
        # for c in range(-1, 2):
            # for d in range(-1, 2):
            #     for e in range(-1, 2):
        action = []
        action.append(a)
        action.append(b)
        action.append(0)
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
        res,retInts,retFloats,retStrings,retBuffer = self.call_sim_function('rwRobot', 'get_global_path')
        path_dist = []
        path_angle = []
        for i in range(len(retFloats)):
            if i < 2:
                continue
                
            if i%2 == 0:
                path_dist.append(retFloats[i])
            else:
                path_angle.append(retFloats[i])
        
        return path_dist, path_angle


    def convert_state(self, laser_points, current_pose, path):
        state = path[0]
        state.extend(path[1])
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
        res,retInts,current_pose,retStrings,found_pose = self.call_sim_function('rwRobot', 'step', action)
        laser_points = self.get_laser_points()
        path_dist, path_angle = self.get_global_path()
        
    #     print path_dist
    #     print path_angle

        reward, self.reached_index = self.compute_reward(action, path_dist, found_pose)
        
        if self.reached_index > len(path_dist)-2:
            is_finish = True
            reward = reward + 10
        
        path_dist_in = []
        path_angle_in = []
        
        for i in range(2):
            index = self.reached_index+1+i
            if index > len(path_dist) - 1:
                path_dist_in.append(0)
                path_angle_in.append(0)
            else:
                path_dist_in.append(path_dist[index])
                path_angle_in.append(path_dist[index])
        
        # print path_dist_in
        # print path_angle_in
    # #     print 'befroe crop: ', len(path_dist)
    #     path_dist = path_dist[self.reached_index+1:]
    #     path_angle = path_angle[self.reached_index+1:]
    # #     print 'after crop: ', len(path_dist)

        state_ = self.convert_state(laser_points, current_pose, [path_dist_in, path_angle_in])

        return state_, reward, is_finish, ''

    ########################################################################################################################################

    ###################################################  reward function ###################################################################
    def compute_reward(self, action, path_dist, found_pose):
        reward = -0.1 - np.sum(np.absolute(action))*0.1
        closest_index = np.argmin(path_dist)
        if path_dist[closest_index] < 0.15 and self.reached_index < closest_index:
            reward = reward + 1
            self.reached_index = closest_index
        # # print action, np.sum(np.absolute(action)), reward

        # if path_dist[0] < self.dist_pre:
        #     reward = reward + 1
        # self.dist_pre = path_dist[0]

        if found_pose == 'f':
            reward = reward - 0.5

        return reward, self.reached_index