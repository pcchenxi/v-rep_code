import numpy as np
# import pandas as pd
import tensorflow as tf

class DeepQNetwork:
    def __init__(
            self,
            action_size,
            state_size,
            learning_rate=0.01,
            reward_decay=0.9,
            e_greedy=0.9,
            replace_target_iter=300,
            memory_size=500,
            batch_size=32,
            e_greedy_increment=None,
            output_graph=False,
    ):
        self.action_size = action_size
        self.state_size = state_size
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon_max = e_greedy
        self.replace_target_iter = replace_target_iter
        self.memory_size = memory_size
        self.batch_size = batch_size
        self.epsilon_increment = e_greedy_increment
        self.epsilon = 0 if e_greedy_increment is not None else self.epsilon_max

        # total learning step
        self.learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]
        self.memory = np.zeros((self.memory_size, state_size * 2 + 2))

        # consist of [target_net, evaluate_net]
        self._build_net()

        self.sess = tf.Session()

        if output_graph:
            # $ tensorboard --logdir=logs
            # tf.train.SummaryWriter soon be deprecated, use following
            tf.summary.FileWriter("logs/", self.sess.graph)

        self.sess.run(tf.global_variables_initializer())
        self.cost_his = []


    def _build_net(self):
        self.s = tf.placeholder(tf.float32, [None, self.state_size], name = 'state')
        self.q_target = tf.placeholder(tf.float32, [None, self.action_size], name = 'q_target')
        # with tf.variable_scope('')
    
    
    def choose_action(self, observation):
        action = 0

        return action

    def store_transition(self, observation, action, reward, observation_):
        a = 0

    def learn(self):
        a = 0

    def plot_cost(self):
        a = 0
