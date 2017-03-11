
import numpy as np
import tensorflow as tf
import tflearn

from ReplayBuffer import ReplayBuffer
from QNetwork import QNetwork
from discrete_2d_grid_world import GridWorld2D


# LOGPATH = "../DDPG/logging/"
LOGPATH = "/home/snair/logging/"

# Max training steps
MAX_EPISODES = 500000
# Max episode length
MAX_EP_STEPS = 1000
# Base learning rate 
LEARNING_RATE = .0001
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


# # ===========================
# #   Tensorflow Summary Ops
# # ===========================
# def build_summaries():
#     episode_reward = tf.Variable(0.)
#     tf.summary.scalar("Reward", episode_reward)
#     episode_ave_max_q = tf.Variable(0.)
#     tf.summary.scalar("Qmax Value", episode_ave_max_q)

#     summary_vars = [episode_reward, episode_ave_max_q]
#     summary_ops = tf.summary.merge_all()

#     return summary_ops, summary_vars


def main(_):
    with tf.Session() as sess:


        state_dim = 25
        action_dim = 4

        QNet = QNetwork(sess, state_dim, action_dim., LEARNING_RATE, TAU, MINIBATCH_SIZE)

        # # Set up summary Ops
        # summary_ops, summary_vars = build_summaries()

        sess.run(tf.global_variables_initializer())
        # writer = tf.summary.FileWriter(SUMMARY_DIR, sess.graph)

        # Initialize target network weights
        critic.update_target_network()

        # Initialize replay memory
        replay_buffer = ReplayBuffer(BUFFER_SIZE, RANDOM_SEED)

        for i in xrange(MAX_EPISODES):

            world = GridWorld2D(10, 10, np.random.randint(10))

            ep_reward = 0.0
            ep_ave_q = 0.0
            ep_ave_loss = 0.0

            status = 0
            # Grab the state features from the environment
            s1 = world.get_state()
            old_reward = 0

            for j in xrange(MAX_EP_STEPS):

                s = s1

                # s_noise = np.reshape(s, (1, state_dim)) #+ np.random.rand(1, 19)

                if replay_buffer.size() > MINIBATCH_SIZE:
                    if np.random.uniform() < max(0.01, EPS_GREEDY_INIT - float(i) / EPS_EPISODES_ANNEAL):
                        index = np.random.choice(4)
                        action = np.zeros((action_dim, ))
                        action[index] = 1
                    else:
                        maxq = -1 * float('inf')
                        maxq_act = []
                        for index in range(action_dim):
                            action = np.zeros((action_dim, ))
                            action[index] = 1
                            q = QNet.predict_target(np.reshape(s, (1, state_dim)), np.reshape(action, (1, action_dim)))
                            if q > maxq:
                                maxq = q
                                maxq_act = action

                else:
                    index = np.random.choice(4)
                    action = np.zeros((action_dim, ))
                    action[index] = 1

                # Make action and step forward in time
                world.take_action(action)

                # Get new state s_(t+1)
                s1 = hfo.getState()
                world.get_state()



                curr_goal_dist = world.get_distance_to_goal()
                curr_obs_dist = world.get_distance_to_closest_obstacle()

                if (i == MAX_EPISODES - 1) or (curr_goal_dist == 0):
                    terminal = True
                else:
                    terminal = False

                r = 0.0
                if j != 0:
                    # If game has finished, calculate reward based on whether or not a goal was scored
                    if curr_goal_dist == 0:
                        r += 5

                    # Else calculate reward as distance between ball and goal
                    r += curr_goal_dist - old_goal_dist
                    r += -0.2 * (curr_obs_dist - old_obs_dist)

                old_goal_dist = curr_goal_dist
                old_obs_dist = curr_obs_dist


                replay_buffer.add(np.reshape(s, (actor.s_dim,)), np.reshape(a, (actor.a_dim,)), r, \
                    terminal, np.reshape(s1, (actor.s_dim,)))

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

                            maxq = -1 * float('inf')
                            maxq_act = []
                            for index in range(action_dim):
                                action = np.zeros((action_dim, ))
                                action[index] = 1
                                q = QNet.predict_target(np.reshape(s1_batch[k], (1, state_dim)), np.reshape(action, (1, action_dim)))
                                if q > maxq:
                                    maxq = q
                                    maxq_act = action

                            y_i.append(r_batch[k] + GAMMA * max_q)


                    predicted_q_value, ep_critic_loss, _ = QNet.train(s_batch, a_batch, np.reshape(y_i, (MINIBATCH_SIZE, 1)))

                    ep_ave_max_q += np.mean(predicted_q_value)
                    ep_ave_loss += np.mean(ep_critic_loss)

                    # Update the target
                    critic.update_target_network()
                    # break

                ep_reward += r

                if terminal:

                    # summary_str = sess.run(summary_ops, feed_dict={
                    #     summary_vars[0]: ep_reward,
                    #     summary_vars[1]: ep_ave_max_q / float(j+1)
                    # })

                    # writer.add_summary(summary_str, i)
                    # writer.flush()

                    f = open(LOGPATH +'logs.txt', 'a')
                    f.write(str(float(ep_reward)) + "," + str(ep_ave_max_q / float(j+1))+ "," + str(float(critic_loss)/ float(j+1)) + "," +  str(EPS_GREEDY_INIT - float(i) / EPS_EPISODES_ANNEAL) + "\n")
                    f.close()


                    print('| Reward: ' , float(ep_reward), " | Episode", i, \
                        '| Qmax:',  (ep_ave_max_q / float(j+1)), ' | Critic Loss: ', float(critic_loss)/ float(j+1))

                    break
            # print "FINISH"
            # break

if __name__ == '__main__':
    tf.app.run()
