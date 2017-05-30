import gym
import numpy as np
import matplotlib.pyplot as plt
env = gym.make('FrozenLake-v0')

#Initialize table with all zeros
Q = np.zeros([env.observation_space.n,env.action_space.n])
# Set learning parameters
lr = .8
y = 0.95
num_episodes = 3000
#create lists to contain total rewards and steps per episode
#jList = []
rList = []
lList = []
for i in range(num_episodes):
    #Reset environment and get first new observation
    s = env.reset()
    # print s
    rAll = 0
    d = False
    j = 0
    elength = 1
    #The Q-Table learning algorithm
    while j < 99:
        j+=1
        #Choose an action by greedily (with noise) picking from Q table
        a = np.argmax(Q[s,:] + np.random.randn(1,env.action_space.n)*(1./(i+1)))
        # a = np.argmax(Q[s,:])
        #Get new state and reward from environment
        s1,r,d,_ = env.step(a)
        #Update Q-Table with new knowledge
        # Q[s,a] = Q[s,a] + lr*(r + y*np.max(Q[s1,:]) - Q[s,a])
        Q[s,a] = Q[s,a] + lr*(r + y*np.max(Q[s1,:]) - Q[s,a])
        # print s, qs, randv, a
        # print s1, r, d, Q[s, a]        
        rAll += r
        s = s1
        elength += 1
        if d == True:
            break
    #jList.append(j)
    rList.append(rAll)
    if rAll == 0:
        elength = 0
    lList.append(elength)
# print rList

print "Score over time: " +  str(sum(rList)/num_episodes)
print "Final Q-Table Values"
print Q

# plt.plot(lList)
# plt.ylabel('some numbers')
# plt.show()