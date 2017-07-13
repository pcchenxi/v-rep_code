import chainer
import chainer.functions as F
import chainer.links as L
import chainerrl
import numpy as np

import time

import sys, os
sys.path.append("../environment") 

import env_vrep

file_name = time.time()

# init simu environment
env = env_vrep.Simu_env(10000)
env.connect_vrep(True)

class QFunction(chainer.Chain):

    def __init__(self, obs_size, n_actions, n_hidden_channels=100):
        super(QFunction, self).__init__()
        with self.init_scope():
            self.conv1 = L.Convolution2D(in_channels=1, out_channels=16, ksize=[1, 8], stride=1)
            self.conv2 = L.Convolution2D(in_channels=16, out_channels=32, ksize=[1, 4], stride=1)
            self.conv3 = L.Convolution2D(in_channels=32, out_channels=32, ksize=[1, 4], stride=1)
            self.l1=L.Linear(5344, 64)
            self.l2=L.Linear(66, 256)
            self.l3=L.Linear(256, n_actions)
            # self.l2=L.Linear(n_hidden_channels, n_hidden_channels/2)
            # self.l3=L.Linear(n_hidden_channels/2, n_hidden_channels/4)
            # self.l4=L.Linear(n_hidden_channels/4, n_actions)

    def __call__(self, x, test=False):
        """
        Args:
            x (ndarray or chainer.Variable): An observation
            test (bool): a flag indicating whether it is in test mode
        """
        # laser_in = np.zeros([x.shape[0], 1, 1, 180]).astype(np.float32)

        path = x[:, :2]
        laser = x[:, 2:]
        # print laser.shape
        # print path.shape

        laser_in = np.expand_dims(laser, axis=1)
        laser_in = np.expand_dims(laser_in, axis=1)

        h = F.relu(self.conv1(laser_in))
        h = F.relu(self.conv2(h))
        h = F.relu(self.conv3(h))

        flat = h.data

        fc1_in = np.zeros([flat.shape[0], flat.shape[1]*flat.shape[2]*flat.shape[3]]).astype(np.float32)
        for i in range(flat.shape[0]):
            temp = flat[i]
            temp = temp.flatten()
            fc1_in[i] = temp

        h = F.relu(self.l1(fc1_in))

        ## add target position
        flat = h.data
        fc2_in = np.zeros([flat.shape[0], flat.shape[1]+path.shape[1]]).astype(np.float32)

        for i in range(flat.shape[0]):
            temp2 = np.append(flat[i], path[i])
            fc2_in[i] = temp2

        h = F.relu(self.l2(fc2_in))
        # print fc2_in.shape

        h = self.l3(h)

        # print flat.shape
        # h = F.relu(self.l3(h))

        # h = F.tanh(self.l1(h))
        return chainerrl.action_value.DiscreteActionValue(h)

def sample_action():
    # print 'in action'
    action = np.random.randint(0, n_actions)
    return action


obs_size = env.state_size
n_actions = env.action_size
print obs_size
q_func = QFunction(obs_size, n_actions)

# Uncomment to use CUDA
# q_func.to_gpu(0)

# _q_func = chainerrl.q_functions.FCStateQFunctionWithDiscreteAction(
#     obs_size, n_actions,
#     n_hidden_layers=1, n_hidden_channels=10)

# Use Adam to optimize q_func. eps=1e-2 is for stability.
optimizer = chainer.optimizers.Adam(eps=1e-2)
optimizer.setup(q_func)

# Set the discount factor that discounts future rewards.
gamma = 0.95

# Use epsilon-greedy for exploration
explorer = chainerrl.explorers.ConstantEpsilonGreedy(
    epsilon=0.2, random_action_func=sample_action)
# explorer = chainerrl.explorers.LinearDecayEpsilonGreedy(
#     start_epsilon = 1, end_epsilon = 0.2, decay_steps = 3000, random_action_func=sample_action)

# DQN uses Experience Replay.
# Specify a replay buffer and its capacity.
replay_buffer = chainerrl.replay_buffer.ReplayBuffer(capacity=10 ** 6)

# Since observations from CartPole-v0 is numpy.float64 while
# Chainer only accepts numpy.float32 by default, specify
# a converter as a feature extractor function phi.
phi = lambda x: x.astype(np.float32, copy=False)

# Now create an agent that will interact with the environment.
agent = chainerrl.agents.DoubleDQN(
    q_func, optimizer, replay_buffer, gamma, explorer,
    replay_start_size=50, update_interval=1,
    target_update_interval=100, phi=phi)

n_episodes = 2000
max_episode_len = 200

chainer.serializers.load_npz("model/obs_conv.model", q_func)

for i in range(1, n_episodes + 1):
    obs = env.reset()
    reward = 0
    done = False
    R = 0  # return (sum of rewards)
    t = 0  # time step
    while not done and t < max_episode_len:

        action = agent.act_and_train(obs, reward)
        obs, reward, done, _ = env.step([action])
        R += reward
        t += 1

        # print t
        # print action, reward
        # o = obs.reshape(1, -1)
        # o = o.astype(np.float32, copy=False)
        # print q_func(o)
        if t % 10 == 0:
            chainer.serializers.save_npz("model/" + str(file_name) + ".model", q_func)

    # if i % 10 == 0:
    print('episode:', i,
            'R:', R,
            'avg R: ', R/t,
            'statistics:', agent.get_statistics())
    if i % 100 == 0:
        file_name = time.time()
    agent.stop_episode_and_train(obs, reward, done)
print('Finished.')


