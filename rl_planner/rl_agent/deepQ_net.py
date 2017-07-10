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
        self.memory_p = np.zeros((self.memory_size, state_size * 2 + 2))
        self.memory_n = np.zeros((self.memory_size, state_size * 2 + 2))



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
        def build_layers(s, c_names, n_l1, w_initializer, b_initializer):
            with tf.variable_scope('l1'):
                w1 = tf.get_variable('w1', [self.state_size, n_l1], initializer=w_initializer, collections=c_names)
                b1 = tf.get_variable('b1', [1, n_l1], initializer=b_initializer, collections=c_names)
                l1 = tf.nn.relu(tf.matmul(s, w1) + b1)

            with tf.variable_scope('l2'):
                w2 = tf.get_variable('w2', [n_l1, self.action_size], initializer=w_initializer, collections=c_names)
                b2 = tf.get_variable('b2', [1, self.action_size], initializer=b_initializer, collections=c_names)
                out = tf.matmul(l1, w2) + b2
            return out

        self.s = tf.placeholder(tf.float32, [None, self.state_size], name = 'state')
        self.s_ = tf.placeholder(tf.float32, [None, self.state_size], name = 'state_')
        self.r = tf.placeholder(tf.float32, [None, ], name = 'reward')
        self.a = tf.placeholder(tf.int32, [None, ], name = 'action')

        # build eval and target network
        with tf.variable_scope('eval_net'):
            c_names, n_l1, w_initializer, b_initializer = \
            ['eval_net_params', tf.GraphKeys.GLOBAL_VARIABLES], 30, \
            tf.random_normal_initializer(0, 0.3), tf.constant_initializer(0.1)
        
            self.q_eval = build_layers(self.s, c_names, n_l1, w_initializer, b_initializer)

        with tf.variable_scope('target_net'):
            c_names = ['target_net_params', tf.GraphKeys.GLOBAL_VARIABLES]
            self.q_next = build_layers(self.s_, c_names, n_l1, w_initializer, b_initializer)

        # build eval and target Q operator
        with tf.variable_scope('q_target'):
            q_target = self.r + self.gamma * tf.reduce_max(self.q_next, axis=1, name='Qmax_s_')
            self.q_target = tf.stop_gradient(q_target)
        
        with tf.variable_scope('q_eval'):
            a_one_hot = tf.one_hot(self.a, depth=self.action_size, dtype=tf.float32)
            self.q_eval_a = tf.reduce_sum(self.q_eval * a_one_hot, axis = 1)

        # build loss operator 
        with tf.variable_scope('loss'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.q_target, self.q_eval_a, name='TD_error'))
        with tf.variable_scope('train'):
            self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

    def choose_action(self, observation):
        # to have batch dimension when feed into tf placeholder
        observation = observation[np.newaxis, :]

        if np.random.uniform() < self.epsilon:
            # forward feed the observation and get q value for every actions
            actions_value = self.sess.run(self.q_eval, feed_dict={self.s: observation})
            action = np.argmax(actions_value)
            print 'self.epsilon: ', self.epsilon, ' Net ', 'action: ', action

        else:
            action = np.random.randint(0, self.action_size)
            print 'self.epsilon: ', self.epsilon, ' Rand ', 'action: ', action

        
        return action

    def store_transition(self, s, a, r, s_):

        transition = np.hstack((s, [a, r], s_))

        if not hasattr(self, 'memory_counter'):
            self.memory_counter = 0
        # if not hasattr(self, 'memory_counter_p'):
        #     self.memory_counter_p = 0
        # if not hasattr(self, 'memory_counter_n'):
        #     self.memory_counter_n = 0
        # # replace the old memory with new memory

        # if r > 0:
        #     index = self.memory_counter_p % self.memory_size
        #     self.memory_p[index, :] = transition
        #     self.memory_counter_p += 1
        # else:
        #     index = self.memory_counter_n % self.memory_size
        #     self.memory_n[index, :] = transition
        #     self.memory_counter_n += 1

        index = self.memory_counter % self.memory_size
        self.memory[index, :] = transition
        self.memory_counter += 1

    def _replace_target_params(self):
        t_params = tf.get_collection('target_net_params')
        e_params = tf.get_collection('eval_net_params')
        self.sess.run([tf.assign(t, e) for t, e in zip(t_params, e_params)])

    def learn(self):
        # if self.memory_counter_n == 0 or self.memory_counter_p == 0:
        #     print memory_counter_n, memory_counter_p
        #     return

        # # check to replace target parameters
        # if self.learn_step_counter % self.replace_target_iter == 0:
        #     self._replace_target_params()
        #     print('\ntarget_params_replaced\n')

        # # sample batch memory from all memory
        # if self.memory_counter_n > self.memory_size:
        #     sample_index_n = np.random.choice(self.memory_size, size=self.batch_size)
        # else:
        #     sample_index_n = np.random.choice(self.memory_counter_n, size=self.batch_size)

        # if self.memory_counter_p > self.memory_size:
        #     sample_index_p = np.random.choice(self.memory_size, size=self.batch_size)
        # else:
        #     sample_index_p = np.random.choice(self.memory_counter_p, size=self.batch_size)

        # batch_memory_n = self.memory_n[sample_index_n, :]
        # batch_memory_p = self.memory_p[sample_index_p, :]
        # batch_memory = np.append(batch_memory_n, batch_memory_p, axis=0)

        # check to replace target parameters
        if self.learn_step_counter % self.replace_target_iter == 0:
            self._replace_target_params()
            print('\ntarget_params_replaced\n')

        # sample batch memory from all memory
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        batch_memory = self.memory_n[sample_index, :]




        _, cost = self.sess.run(
            [self._train_op, self.loss],
            feed_dict={
                self.s: batch_memory[:, :self.state_size],
                self.a: batch_memory[:, self.state_size],
                self.r: batch_memory[:, self.state_size + 1],
                self.s_: batch_memory[:, -self.state_size:],
            })

        self.cost_his.append(cost)

        # increasing epsilon
        self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
        self.learn_step_counter += 1
        print 'learn_step_counter: ', self.learn_step_counter

    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(len(self.cost_his)), self.cost_his)
        plt.ylabel('Cost')
        plt.xlabel('training steps')
        plt.show()

# if __name__ == '__main__':
#     DQN = DeepQNetwork(3,4, output_graph=True)