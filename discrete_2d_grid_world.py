import numpy as np

class GridWorld2D(object):

    # World will be 2D array with following values:
    #   0: empty spot
    #   1: obstacle/outside of grid environment
    #   2: robot
    #   3: goal
    #   4: robot + goal (robot has reached goal)
    self.world = []
    self.robot_loc = [0, 0]
    self.goal_loc = [0, 0]

    def __init__(self, world_x, world_y, num_obstacles):
        self.world = np.zeros(shape=(world_x, world_y))
        for i in range(num_obstacles):
            self.add_obstacle()
        self.robot_loc = set_random_empty_loc(2)
        self.goal_loc = set_random_empty_loc(3)

    def add_obstacle(self):
        # Set some rectangle in self.world to 1's

    def set_random_empty_loc(self, val):
        # Set the value in self.world of currently empty (0 value) to passed in val

    def set_robot_position(self, loc_x, loc_y):
        # Sets robot to specific location (reducing randomness in initialization)

    def set_goal_position(self, loc_x, loc_y):
        # Sets goal to specific location (reducing randomness in initialization)

    def take_action(self, action):
        # Move robot_loc in action direction

    def get_neighborhood_state(self, n):
        # Return n*n grid of world that is centered at robot_loc
        # Outside of grid boundaries will just be treated as obstacles (value 1)

    def get_vector_to_goal(self):
        # Returns vector to goal (delta x by delta y)

    def get_distance_to_goal(self):
        # Returns Euclidean distance between robot and goal

    def get_distance_to_closest_obstacle(self):
        # Returns Euclidean distance between robot and any obstacle/boundaries of world

    def display_world(self):
        # Prints out world in nice 2d format

    def get_state():
        i, j = self.robot_loc
        state = np.zeros((5,5))
        for r in state:
            for c in state[i]:
                try:
                    state[r,c] = self.world[i - 2 + r,j - 2 + r]
                except:
                    continue
        return np.append(np.flatten(state), self.get_distance_to_goal())
