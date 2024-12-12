# import gym
from environment import dynamicObstacleEnv
import numpy as np
from agent import Agent
import matplotlib.pyplot as plt
from plot import rendering

if __name__ == "__main__":
    # env = gym.make('CartPole-v1')
    env = dynamicObstacleEnv()
    N = 20
    batch_size = 10
    n_epochs = 20
    alpha = 0.00004
    gamma = 0.8
    policy_clip = 0.4
    gae_lambda = 0.95
    agent = Agent(n_actions = env.action_space.shape[0], batch_size=batch_size,
                  alpha=alpha, n_epochs=n_epochs,
                  input_dims = env.observation_space.shape, gamma=gamma, policy_clip=policy_clip, gae_lambda=gae_lambda)
    n_games = 150

    best_score = env.reward_range[0]
    score_history = []

    learn_iters = 0
    avg_score = 0
    n_steps = 0

    for i in range(n_games):
        observation = env.reset()
        # observation = observation[0]
        # plt.close()
        # render_plot = rendering(observation)

        done = False
        score = 0
        while not done:
            action, prob, val = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            n_steps += 1
            score += reward
            agent.remember(observation, action, prob, val, reward, done)
            if n_steps % N == 0:
                agent.learn()
                learn_iters += 1
            observation = observation_
            # render_plot.plot(observation)
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            best_score = avg_score
            agent.save_models()

        print('episode', i, 'score %.1f' % score, 'avg score %.1f' % avg_score,
                'time_steps', n_steps, 'learning_steps', learn_iters)
    x = [i+1 for i in range(len(score_history))]
    
    # plt.ioff()  # Turn off interactive mode
    # plt.show()
    plt.plot(x, score_history)
    plt.savefig("image.png")
    plt.close()
