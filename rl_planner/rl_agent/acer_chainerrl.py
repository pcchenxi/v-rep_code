from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import
from builtins import *  # NOQA
from future import standard_library
standard_library.install_aliases()
import argparse

import chainer
from chainer import functions as F
from chainer import links as L

import chainerrl
from chainerrl.agents import a3c
from chainerrl import experiments
from chainerrl import links
from chainerrl import misc
from chainerrl.optimizers.nonbias_weight_decay import NonbiasWeightDecay
from chainerrl.optimizers import rmsprop_async
from chainerrl import policies
from chainerrl.recurrent import RecurrentChainMixin
from chainerrl import v_function

import numpy as np

import time

import sys, os
sys.path.append("../environment") 

import env_vrep


file_name = time.time()
env = env_vrep.Simu_env(10000)
env.connect_vrep(True)
def sample_action():
    # print 'in action'
    action = np.random.randint(0, n_actions)
    return action
def phi(obs):
    return obs.astype(np.float32)

def make_env():
    env = env_vrep.Simu_env(10000)
    env.connect_vrep(True)
    return env

class Network(chainer.Chain):

    def __init__(self, obs_size, n_actions, n_hidden_channels=50):
        super(Network, self).__init__()
        with self.init_scope():
            l0=L.Linear(obs_size, n_hidden_channels)
            l1=L.Linear(n_hidden_channels, n_hidden_channels)

    def __call__(self, x, test=False):

        feature = F.relu(self.l0(x))
        feature = F.relu(self.l1(feature))
        return feature

class Actor_net(chainer.Chain):
    def __init__(self, feature_size, n_actions, n_hidden_channels=50):
        super(Actor_net, self).__init__()
        with self.init_scope():
            l0=L.Linear(feature_size, n_actions)

    def __call__(self, feature, test=False):

        # action_dis = F.relu(self.l0(feature))
        action_dis = self.la(feature)
        return chainerrl.policies.SoftmaxPolicy(action_dis)

class Critic_net(chainer.Chain):
    def __init__(self, feature_size, n_hidden_channels=50):
        super(Critic_net, self).__init__()
        with self.init_scope():
            l0=L.Linear(feature_size, 1)

    def __call__(self, feature, test=False):
        # action_dis = F.relu(self.l0(feature))
        state_value = self.la(feature)
        return chainerrl.action_value.DiscreteActionValue(state_value)

class A3CFFSoftmax(chainer.ChainList):
    """An example of A3C feedforward softmax policy."""

    # def __init__(self, obs_size, n_actions, hidden_sizes=50):
    #     self.feature = Network(obs_size, n_actions, hidden_sizes)
    #     feature_size = hidden_sizes
    #     self.pi = Actor_net(feature_size, n_actions, hidden_sizes)
    #     self.v = Critic_net(feature_size, hidden_sizes)

    #     super().__init__(self.feature, self.pi, self.v)

    # def pi_and_v(self, state):
    #     feature = self.feature(state)
    #     return self.pi(feature), self.v(feature)
    

    def __init__(self, ndim_obs, n_actions, hidden_sizes=(200, 200)):
        self.pi = policies.SoftmaxPolicy(
            model=links.MLP(ndim_obs, n_actions, hidden_sizes))
        self.v = links.MLP(ndim_obs, 1, hidden_sizes=hidden_sizes)
        super().__init__(self.pi, self.v)

    def pi_and_v(self, state):
        return self.pi(state), self.v(state)

def sample_action():
    # print 'in action'
    action = np.random.randint(0, n_actions)
    return action


obs_size = env.state_size
n_actions = env.action_size
model = A3CFFSoftmax(obs_size, n_actions)

opt = rmsprop_async.RMSpropAsync(
    lr=0.01, eps=0.2, alpha=0.99)
opt.setup(model)
opt.add_hook(chainer.optimizer.GradientClipping(40))


explorer = chainerrl.explorers.ConstantEpsilonGreedy(
    epsilon=0.2, random_action_func=sample_action)

agent = a3c.A3C(model, opt, t_max=0.01, gamma=0.99,
                beta=0.01, phi=phi)

experiments.train_agent_async(
    agent=agent,
    processes=2,
    make_env=make_env,
    steps=8 * 10 ** 7)
agent.save('final_agent')