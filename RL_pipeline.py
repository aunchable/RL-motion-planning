
import numpy as np
import tensorflow as tf
import tflearn

from ReplayBuffer import ReplayBuffer
from QNetwork import QNetwork
from discrete_2d_grid_world import GridWorld2D
import time

# LOGPATH = "../DDPG/logging/"
LOGPATH = "./logging/"

# Max training steps
MAX_EPISODES = 500000
# Max episode length
MAX_EP_STEPS = 100
# Base learning rate
LEARNING_RATE = .001
# Discount factor
GAMMA = 0.99
# Soft target update param
TAU = 0.001

# Noise for exploration
EPS_GREEDY_INIT = 1.0
EPS_EPISODES_ANNEAL = 1000

# Directory for storing tensorboard summary results
# SUMMARY_DIR = './results/tf_ddpg'
RANDOM_SEED = 1234
# Size of replay buffer
BUFFER_SIZE = 10000
MINIBATCH_SIZE = 1024

def main(_):
    with tf.Session() as sess:

        # Define window size
        window_size = 5

        # Sets state and action size based on window size
        state_dim = window_size*window_size + 2 + 1 + 1
        action_dim = 4

        QNet = QNetwork(sess, state_dim, action_dim,
        LEARNING_RATE, TAU, MINIBATCH_SIZE)

        sess.run(tf.global_variables_initializer())

        # Initialize target network weights
        QNet.update_target_network()

        # Initialize replay memory
        replay_buffer = ReplayBuffer(BUFFER_SIZE, RANDOM_SEED)


        # Goes through Episodes
        for i in xrange(MAX_EPISODES):

            world = GridWorld2D(10, 10, 2)
            # print world.world.flatten()
            with open("logging/episode" + str(i) + ".txt",'a') as f_handle:
                np.savetxt(f_handle,[world.world.flatten()])

            # Initialize statistics for each episode
            ep_reward = 0.0
            ep_ave_q = 0.0
            ep_ave_loss = 0.0

            status = 0
            # Grab the state features from the environment
            s1 = np.concatenate((
                np.reshape(world.get_neighborhood_state(window_size),
                            window_size**2),
                np.reshape(world.get_vector_to_goal(), 2),
                np.reshape(world.get_distance_to_goal(), 1),
                np.reshape(world.get_distance_to_closest_obstacle(), 1)))

            old_reward = 0

            for j in xrange(MAX_EP_STEPS):

                s = s1

                # s_noise = np.reshape(s, (1, state_dim)) #+ np.random.rand(1, 19)

                if replay_buffer.size() > MINIBATCH_SIZE:
                    # With some probability act randomly
                    if np.random.uniform() < max(0.01,
                      EPS_GREEDY_INIT - float(i) / EPS_EPISODES_ANNEAL):
                        index = np.random.choice(4)
                        action = np.zeros((action_dim, ))
                        action[index] = 1
                    # Otherwise pick action which maximizes the Q value
                    else:
                        maxq = -1 * float('inf')
                        maxq_act = []
                        for index in range(action_dim):
                            action = np.zeros((action_dim, ))
                            action[index] = 1
                            # print np.reshape(s, (1, state_dim)).shape
                            # print np.reshape(action, (1, action_dim)).shape
                            q = QNet.predict_target(np.reshape(s, (1, state_dim)),
                                    np.reshape(action, (1, action_dim)))
                            # print q
                            if q > maxq:
                                maxq = q
                                maxq_act = action
                        # print max_q

                else:
                    index = np.random.choice(4)
                    action = np.zeros((action_dim, ))
                    action[index] = 1

                # Log action
                with open("logging/episode" + str(i) + ".txt",'a') as f_handle:
                    np.savetxt(f_handle,[action])
                # Make action and step forward in time
                moved = world.take_action(action)
                # Get new state s_(t+1)

                s1 = np.concatenate((
                    np.reshape(world.get_neighborhood_state(window_size),
                                window_size**2),
                    np.reshape(world.get_vector_to_goal(), 2),
                    np.reshape(world.get_distance_to_goal(), 1),
                    np.reshape(world.get_distance_to_closest_obstacle(), 1)))

                # Update current distance measurments
                curr_goal_dist = world.get_distance_to_goal()
                curr_obs_dist = world.get_distance_to_closest_obstacle()

                # Decide if end of episode has been reached
                if ((j == MAX_EP_STEPS - 1) or (curr_goal_dist == 0) or
                        (moved == 0) or (curr_obs_dist==0)):
                    terminal = True
                else:
                    terminal = False

                r = 0.0
                if j != 0:
                    # If game has finished, calculate reward based on whether or
                    # not a goal was scored
                    if curr_goal_dist == 0:
                        r += 5
                    elif curr_obs_dist == 0 or moved == 0:
                        r -= 100

                    # Else calculate reward as distance between ball and goal
                    r += -1 * (curr_goal_dist - old_goal_dist)
                    r += 0.8 * (curr_obs_dist - old_obs_dist)

                old_goal_dist = curr_goal_dist
                old_obs_dist = curr_obs_dist

                # Add experience to memory
                replay_buffer.add(np.reshape(s, (state_dim,)), \
                    np.reshape(action, (action_dim,)), r, \
                    terminal, np.reshape(s1, (state_dim,)))

                # Keep adding experience to the memory until
                # there are at least minibatch size samples
                if replay_buffer.size() > MINIBATCH_SIZE:

                    s_batch, a_batch, r_batch, t_batch, s1_batch = \
                        replay_buffer.sample_batch(MINIBATCH_SIZE)

                    y_i = []
                    for k in xrange(MINIBATCH_SIZE):
                        if t_batch[k] == True:
                            y_i.append(r_batch[k])
                        else:
                            # Update y_i with action in state t+1 that maximizes Q
                            maxq = -1 * float('inf')
                            maxq_act = []
                            for index in range(action_dim):
                                action = np.zeros((action_dim, ))
                                action[index] = 1
                                q = QNet.predict_target(
                                    np.reshape(s1_batch[k], (1, state_dim)),
                                    np.reshape(action, (1, action_dim)))
                                if q > maxq:
                                    maxq = q
                                    maxq_act = action

                            y_i.append(r_batch[k] + GAMMA * maxq)

                    # Train network using minibatch
                    predicted_q_value, ep_critic_loss, _ = QNet.train(
                        s_batch, a_batch, np.reshape(y_i, (MINIBATCH_SIZE, 1)))

                    # Uodate episode statistics
                    ep_ave_q += np.mean(predicted_q_value)
                    ep_ave_loss += np.mean(ep_critic_loss)

                    # Update the target
                    QNet.update_target_network()
                    # break

                ep_reward += r

                if terminal:

                    f = open(LOGPATH +'logs5.txt', 'a')
                    f.write(str(float(ep_reward)) + "," +
                        str(ep_ave_q / float(j+1))+ "," +
                        str(float(ep_ave_loss)/ float(j+1)) + "," +
                        str(EPS_GREEDY_INIT - float(i) / EPS_EPISODES_ANNEAL) +
                        "\n")
                    f.close()


                    print('| Reward: ' , float(ep_reward), " | Episode", i, \
                        '| Qmax:',  (ep_ave_q / float(j+1)), \
                        ' | Critic Loss: ', float(ep_ave_loss)/ float(j+1))

                    break
            # print "FINISH"
            # break

if __name__ == '__main__':
    tf.app.run()
