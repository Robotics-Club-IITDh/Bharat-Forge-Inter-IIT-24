from environment import dynamicObstacleEnv
import matplotlib.pyplot as plt
import numpy as np

env = dynamicObstacleEnv()

print("Observation space shape:", env.observation_space.shape)
print("Action space:", env.action_space.shape)

observation = env.reset()
# print("Initial Observation: ", observation)

plt.ion()
fig, ax = plt.subplots()

initial_x = [observation[0], observation[2], observation[4], observation[6], observation[8]]
initial_y = [observation[1], observation[3], observation[5], observation[7], observation[9]]

scatter = ax.scatter(initial_x, initial_y, c=['red', 'green', 'blue', 'blue', 'blue'])

labels = ['Robot', 'Goal', 'Obstcale 1', 'Obstcale 2', 'Obstcale 3']

handles = [
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label=labels[0]),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label=labels[1]),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label=labels[2]),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label=labels[3]),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label=labels[4])
]
ax.legend(handles=handles)

ax.set_xlim(0, 10)
ax.set_ylim(0, 10)



for _ in range(200):
    action = env.action_space.sample()
    # print("Action taken: ", action)
    observation, reward, done, info = env.step(action)
    # print("New observation:", observation[:10])
    # print("Reard: ", reward)
    # print("Done: ", done)

    robot_position = observation[:2]
    goal_position = observation[2:4]
    obs_position = observation[4:]

    x = [robot_position[0], goal_position[0], obs_position[0], obs_position[2], obs_position[4]]
    y = [robot_position[1], goal_position[1], obs_position[1], obs_position[3], obs_position[5]]
    
    scatter.set_offsets(np.column_stack((x, y)))

    plt.draw()
    plt.pause(0.05)


    if done:
        break
plt.ioff()  # Turn off interactive mode
plt.show()