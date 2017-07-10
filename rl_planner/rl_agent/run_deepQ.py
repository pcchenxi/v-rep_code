import sys, os
sys.path.append("../environment") 

import env_vrep
from deepQ_net import DeepQNetwork

# init simu environment
env = env_vrep.Simu_env(10000)
env.connect_vrep(True)

total_step = 0

# # init deepQ params
RL = DeepQNetwork(env.action_size, env.state_size,
                    learning_rate=0.01,
                    reward_decay=0.9,
                    e_greedy=0.9,
                    replace_target_iter=200,
                    memory_size=2000,
                    # output_graph=True
                    )

def run_one_ep():
    global env, total_step, RL
    # initial observation
    observation = env.reset()

    while True:
        # RL choose action based on observation
        action = RL.choose_action(observation) 

        # RL take action and get next observation and reward
        observation_, reward, done = env.step(action)

        RL.store_transition(observation, action, reward, observation_) 

        if (total_step > 200) and (total_step % 5 == 0):
            RL.learn() 

        # swap observation
        observation = observation_

        # break while loop when end of this episode
        if done:
            break

        total_step += 1


if __name__ == "__main__":
    global total_step
    total_step = 0
    for episode in range(300):
        run_one_ep()

    # end of game
    print('game over')

    RL.plot_cost()  