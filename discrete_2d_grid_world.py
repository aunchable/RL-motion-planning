import numpy as np
import random
import copy

# Declaration of constants

WORLD = {
	'EMPTY': -3,
	'OBSTACLE': -2,
	'ROBOT': -1,
	'GOAL': 0
}

def getNumUnvisitedSpaces(grid):
    numUnvisited = 0
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i, j] == WORLD['EMPTY']:
                numUnvisited = numUnvisited + 1
    return numUnvisited

def getNeighborsD(grid, distance):
    boundaryPointsList = []
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            neighbors = getNeighbors( (i, j), grid)
            # check if it has a neighbor of distance d but is
            # unvisited itself
            if checkDistance(neighbors, distance, grid) == 1 \
            and grid[i][j] == WORLD['EMPTY']:
                boundaryPointsList.append( (i, j) )
    return boundaryPointsList

# Check if there is a neighboring point of some distance
def checkDistance(neighbors, distance, grid):
    for i in range(len(neighbors)):
        if grid[neighbors[i][0], neighbors[i][1] ] == distance:
            return 1
    return 0

# Get a list of valid neighbors for any given point in the grid
def getNeighbors(point, grid):
    y = point[0]
    x = point[1]
    listNeighbors = []
    # Get (y, x - 1)
    if x - 1 >= 0:
        listNeighbors.append( (y, x - 1) )
    # Get (y, x + 1)
    if x + 1 <= len(grid[0]) - 1:
        listNeighbors.append((y, x + 1))
    # Get (y + 1, x)
    if y + 1 <= len(grid) - 1:
        listNeighbors.append((y + 1, x))
    # Get (y - 1, x)
    if y - 1 >= 0:
        listNeighbors.append((y - 1, x))
    return listNeighbors

def getAllNeighbors(point, grid):

    listNeighbors = getNeighbors(point, grid)
    y = point[0]
    x = point[1]

    #Get (y + 1, x - 1), (y - 1, x - 1)
    if x - 1 >= 0:
        if y + 1 <= len(grid) - 1:
            listNeighbors.append( (y + 1, x - 1))
        if y - 1 >= 0:
            listNeighbors.append( (y - 1, x - 1))
    # Get (y + 1, x + 1), (y - 1, x + 1)
    if x + 1 <= len(grid[0]) - 1:
        if y + 1 <= len(grid) - 1:
            listNeighbors.append( (y + 1, x + 1))
        if y - 1 >= 0:
            listNeighbors.append( (y - 1, x + 1))

    return listNeighbors

def out_of_bounds(row, col, num_rows, num_cols):

    if row >= num_rows or col >= num_cols or row < 0 or col < 0:
        return True
    else:
        return False

class GridWorld2D:

    def __init__(self, world_x, world_y, num_obstacles):

        # World will be 2D array with following values:
        #   0: empty spot
        #   1: obstacle/outside of grid environment
        #   2: robot
        #   3: goal
        #   4: robot + goal (robot has reached goal)

        self.robot_loc = (0, 0)
        self.goal_loc = (0, 0)
        self.world = np.zeros((world_x, world_y)) + WORLD['EMPTY']

        # self.set_random_robot_loc(WORLD['ROBOT'])
        # self.set_random_goal_loc(WORLD['GOAL'])
        self.set_random_robot_loc()
        self.set_random_goal_loc()

        print 'INITIAL WORLD'
        print self.world
        obstacle_count = 0

        # CODE FOR ADDING SINGLE CELL OBSTACLES
        #
        # while obstacle_count < num_obstacles:
        #     obstacle_count += self.add_obstacle()

        # Multi-cell obstacles

        # TODO: perhaps add check to see if obstacle_count does not change
        # for a while, only really a problem if have a lot of obstacles:

        while obstacle_count < num_obstacles:
            obstacle_count += self.add_rect_obstacle()

        print 'WORLD'
        print self.world
        print 'WAVEFRONT'
        print self.wave_front()

    def generate_point_obstacle_at_empty_pos(self):
        obs_row = random.randint(0, len(self.world) - 1)
        obs_col = random.randint(0, len(self.world[0]) - 1)

        while self.world[obs_row, obs_col] != WORLD['EMPTY']:
            obs_row = random.randint(0, len(self.world) - 1)
            obs_col = random.randint(0, len(self.world[0]) - 1)

        return (obs_row, obs_col)

    def getRectangleCoordinates(self, rows, cols, tl_coor):
        rectCoors = []

        for i in range(tl_coor[0], tl_coor[0] + rows):
            for j in range(tl_coor[1], tl_coor[1] + cols):
                if out_of_bounds(i, j, len(self.world), len(self.world[0]) ) == False:
                    rectCoors.append((i, j))

        return rectCoors

    def generate_rect_obstacle_at_empty_pos(self):
        tl_coor = self.generate_point_obstacle_at_empty_pos()
        # print 'TL COOR'
        # print tl_coor

        # Get top left coordinates that we know are empty
        obs_tl_row = tl_coor[0]
        obs_tl_col = tl_coor[1]

        row_last = random.randint(obs_tl_row, len(self.world))
        col_last = random.randint(obs_tl_col, len(self.world[0]))

        # print 'ENDING COORDINATES'
        # print (row_last, col_last)

        rectangle_coordinates = []

        for i in range(obs_tl_row, row_last):
            for j in range(obs_tl_col, col_last):
                rectCoor = self.getRectangleCoordinates(i - obs_tl_row + 1, j - obs_tl_col + 1, tl_coor)
                all_free = True
                for k in range(len(rectCoor)):
                    if self.world[rectCoor[k][0], rectCoor[k][1]] != WORLD['EMPTY']:
                        all_free = False

                # If all of these coordinates are free, fill up the rectangle
                if all_free == True:
                    for coor in rectCoor:
                        rectangle_coordinates.append(coor)
                # Created a rectangle that is not all empty... so return coordinates so far
                else:
                    return rectangle_coordinates

        return rectangle_coordinates

    def add_rect_obstacle(self):

        # Generate obstacle and add it to the world
        obstacle_added = 0
        coordinates = self.generate_rect_obstacle_at_empty_pos()
        for coor in coordinates:
            self.world[coor[0], coor[1] ] = WORLD['OBSTACLE']

        # No obstacle actually generated, so return 0
        if len(coordinates) == 0:
            return 0

        # Check if it can be added while keeping path between goal and robot.
        # If so, add it. Otherwise remove the added obstacle

        if self.wave_front()[self.robot_loc[0], self.robot_loc[1]] != WORLD['EMPTY']:
            obstacle_added = 1
        else:
            for coor in coordinates:
                self.world[coor[0], coor[1] ] = WORLD['EMPTY']

        # print 'OBSTACLE ADDED ' + str(obstacle_added)
        return obstacle_added

    def add_obstacle(self):
        # Add obstacles at empty positions while maintaining path between
        # robot and goal

        # First generate an obstacle at empty position and add it to the world
        obstacle_added = 0
        obs_position = self.generate_point_obstacle_at_empty_pos()
        # Add an obstacle at an empty position
        self.world[obs_position[0], obs_position[1]] = WORLD['OBSTACLE']

        # Check if it can be added while keeping path between goal and robot.
        # If so, add it. Otherwise remove the added obstacle

        if self.wave_front()[self.robot_loc[0], self.robot_loc[1]] != WORLD['EMPTY']:
            obstacle_added = 1
        else:
            self.world[obs_position[0], obs_position[1]] = WORLD['EMPTY']

        print 'OBSTACLE ADDED ' + str(obstacle_added)
        return obstacle_added

    # Get wave_front coloring
    def wave_front(self):

        grid = copy.deepcopy(self.world) # make copy of the world to fill in
        # get robot and goal position
        robot_pos = self.robot_loc
        goal_pos = self.goal_loc

        # Treat the robot position as an empty cell that can be accessed
        # by the wave-front

        grid[robot_pos[0], robot_pos[1]] = WORLD['EMPTY']

        # Initialize to 0 so we first check distance from goal position
        d = 0
        # Iterate until all spaces have been visited
        while getNumUnvisitedSpaces(grid) > 0:
            # Get current points to update
            current_points = getNeighborsD(grid, d)
            # Update distance until the robot position is reached
            # Then return the grid
            if not current_points:
                return grid
            elif robot_pos in current_points:
                grid[robot_pos[0], robot_pos[1]] = d + 1
                return grid
            else:
                for i in range(len(current_points)):
                    grid[current_points[i][0], current_points[i][1]] = d + 1

            # Go to next layer of distances
            d = d + 1

        return grid

    def set_random_robot_loc(self):

        # Wait till find an empty value and then set it to val
        row = random.randint(0, len(self.world) - 1 )
        col = random.randint(0, len(self.world[0]) - 1)

        while self.world[row, col] != WORLD['EMPTY']:
            row = random.randint(0, len(self.world) - 1 )
            col = random.randint(0, len(self.world[0]) - 1)

        self.world[row, col] = WORLD['ROBOT']
        self.robot_loc = (row, col)

        return

    def set_random_goal_loc(self):

        # Set the value in self.world of currently empty (0 value) to passed in val
        # and set goal location

        # Wait till find an empty value and then set it to val
        row = random.randint(0, len(self.world) - 1 )
        col = random.randint(0, len(self.world[0]) - 1)

        while self.world[row, col] != WORLD['EMPTY']:
            row = random.randint(0, len(self.world) - 1 )
            col = random.randint(0, len(self.world[0]) - 1)

        self.world[row, col] = WORLD['GOAL']
        self.goal_loc = (row, col)

        return

    def set_robot_position(self, loc_x, loc_y):
        # Sets robot to specific location (reducing randomness in initialization)

        self.world[loc_x, loc_y] = WORLD['ROBOT']
        return

    def set_goal_position(self, loc_x, loc_y):
        # Sets goal to specific location (reducing randomness in initialization)

        self.world[loc_x, loc_y] = WORLD['GOAL']
        return

    # one hot vector input [LEFT, RIGHT, UP, DOWN]
    # do nothing if you can't move in that direction
    def take_action(self, action):
        # Move robot_loc in action direction

        robot_pos = self.robot_loc
        # Move Left
        if action[0] == 1:
            if out_of_bounds(robot_pos[0], robot_pos[1] - 1, len(self.world), len(self.world[0])) == False and self.world[robot_pos[0], robot_pos[1] - 1] == WORLD['EMPTY']:
                self.robot_loc = (robot_pos[0], robot_pos[1] - 1)
                self.world[robot_pos[0], robot_pos[1] - 1] = WORLD['ROBOT']
        # Move Right
        elif action[1] == 1:
            if out_of_bounds(robot_pos[0], robot_pos[1] + 1, len(self.world), len(self.world[0])) == False and self.world[robot_pos[0], robot_pos[1] + 1] == WORLD['EMPTY']:
                self.robot_loc = (robot_pos[0], robot_pos[1] + 1)
                self.world[robot_pos[0], robot_pos[1] + 1] = WORLD['ROBOT']
        # Move Up
        elif action[2] == 1:
            if out_of_bounds(robot_pos[0] - 1, robot_pos[1], len(self.world), len(self.world[0])) == False and self.world[robot_pos[0] - 1, robot_pos[1]] == WORLD['EMPTY']:
                self.robot_loc = (robot_pos[0] - 1, robot_pos[1])
                self.world[robot_pos[0] - 1, robot_pos[1]] = WORLD['ROBOT']
        # Move Down
        else:
            if out_of_bounds(robot_pos[0] + 1, robot_pos[1], len(self.world), len(self.world[0])) == False and self.world[robot_pos[0] + 1, robot_pos[1]] == WORLD['EMPTY']:
                self.robot_loc = (robot_pos[0] + 1, robot_pos[1])
                self.world[robot_pos[0] + 1, robot_pos[1]] = WORLD['ROBOT']

        return

    # n must be odd
    def get_neighborhood_state(self, n):

        # Return n*n grid of world that is centered at robot_loc
        # Outside of grid boundaries will just be treated as obstacles
        rows = (self.robot_loc[0] - int(n/2), self.robot_loc[0] + int(n/2)  )
        cols = (self.robot_loc[1] - int(n/2), self.robot_loc[1] + int(n/2)  )
        print rows
        print cols
        neighborhood_grid = np.zeros((n, n))

        for row in range(rows[0], rows[1] + 1):
            for col in range(cols[0], cols[1] + 1):
                if out_of_bounds(row, col, len(self.world), len(self.world[0])  ) == False:
                    neighborhood_grid[ row - rows[0], col - cols[0] ] = self.world[row][col]
                else:
                    neighborhood_grid[ row - rows[0], col - cols[0] ] = WORLD['OBSTACLE']

        return neighborhood_grid

    def get_vector_to_goal(self):

        # Returns vector to goal (delta x by delta y)
        return (self.goal_loc[0] - self.robot_loc[0], self.goal_loc[1] - self.robot_loc[1])

    def get_distance_to_goal(self):

        # Returns Euclidean distance between robot and goal
        vec_to_goal = self.get_vector_to_goal()
        return np.sqrt( (vec_to_goal[0])**2 + (vec_to_goal[1])**2 )

    def get_distance_to_closest_obstacle(self):
        # Returns Euclidean distance between robot and any obstacle/boundaries of world

        # Expand out from robot until find an obstacle
        robot_pos = self.robot_loc
        queue = getAllNeighbors(robot_pos, self.world)
        nearest_obstacle_coor = (0, 0)
        done = False

        while done == False:

            # Look in current queue
            for loc in queue:
                if self.world[loc[0], loc[1]] == WORLD['OBSTACLE']:
                    nearest_obstacle_coor = (loc[0], loc[1])
                    done = True
                    break

            if done == False:
                # Add to queue
                orig_queue = copy.deepcopy(queue)

                for loc in orig_queue:
                    neighbor_list = getAllNeighbors(loc, self.world)
                    for neighbors in neighbor_list:
                        queue.append(neighbors)

        # Get distance to borders:
        row_border_dist = min(robot_pos[0], len(self.world) - robot_pos[0] - 1 ) + 1
        col_border_dist = min(robot_pos[1], len(self.world[0]) - robot_pos[1] - 1 ) + 1
        border_dist = min(row_border_dist, col_border_dist)

        # Get distance to nearest_obstacle_coor
        obstacle_dist = np.sqrt( (nearest_obstacle_coor[0] - robot_pos[0])**2 + (nearest_obstacle_coor[1] - robot_pos[1])**2 )
        return min(obstacle_dist, border_dist)

    def display_world(self):
        for i in range(len(self.world)):
            # Initialize string for that line
            lineStr = ""
            for j in range(len(self.world[i])):
                # Print O for obstacle, G for goal,
                # R for robot, and E for empty
                if self.world[i][j] == WORLD['OBSTACLE']:
                    lineStr += "O "
                elif self.world[i][j] == WORLD['GOAL']:
                    lineStr += "G "
                elif self.world[i][j] == WORLD['ROBOT']:
                    lineStr += "R "
                else:
                    lineStr += "E "
            print lineStr

        return

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

shiet = GridWorld2D(5, 5, 3)
