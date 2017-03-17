import numpy as np
from collections import deque
import copy

# Declaration of constants to represent each possible
# item in the grid, world representation.

WORLD = {
	'EMPTY': -3,
	'OBSTACLE': -2,
	'ROBOT': -1,
	'GOAL': 0
}

# Obtain the number of empty spaces in the grid for
# the wavefront potential function
def getNumUnvisitedSpaces(grid):
	numUnvisited = 0
	for i in range(len(grid)):
		for j in range(len(grid[i])):
			if grid[i, j] == WORLD['EMPTY']:
				numUnvisited = numUnvisited + 1
	return numUnvisited

# Get list of unvisited points that have neighbors of a given distance
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
# for the wavefront potential calculation
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

# Determine if a coordinate is out of bounds in a grid
def out_of_bounds(row, col, num_rows, num_cols):

	if row >= num_rows or col >= num_cols or row < 0 or col < 0:
		return True
	else:
		return False

# Class to handle a 2D world representation where a robot navigates
# from a set starting position to goal position amidst a number of
# randomly placed obstacles in the world.
class GridWorld2D:

	# Initialize world with its x and y dimensions and the
	# total number of obstacles to be created in the world
	def __init__(self, world_x, world_y, num_obstacles):

		# World will be 2D array with empty spots, obstacles,
		# a marker of the robot's current position, and the goal position.

		# Initialize robot and goal location in the world in addition
		# to initializing all cells in the grid representation in the
		# world as empty
		self.robot_loc = (0, 0)
		self.goal_loc = (0, 0)
		self.world = np.zeros((world_x, world_y)) + WORLD['EMPTY']

		# Randomly set the robot and goal locations
		self.set_random_robot_loc()
		self.set_random_goal_loc()

		# Initialize a obstacle count and list of obstacles,
		# and modify these as obstacles are generated and
		# valid ones are added to the world until the total
		# number of obstacles desired have been added.
		obstacle_count = 0
		self.obs_list = []
		while obstacle_count < num_obstacles:
			obstacle_count += self.add_rect_obstacle()

		# Display the world in a nice 2D format and display
		# a representation of the robot location, goal location,
		# and obstacles in the world.
		self.display_world()
		print(self.world_representation())

	# Returns a random empty coordinate for the top left
	# corner of an obstacle.
	def generate_point_obstacle_at_empty_pos(self):
		obs_row = np.random.randint(0, len(self.world))
		obs_col = np.random.randint(0, len(self.world[0]))

		while self.world[obs_row, obs_col] != WORLD['EMPTY']:
			obs_row = np.random.randint(0, len(self.world))
			obs_col = np.random.randint(0, len(self.world[0]))

		return (obs_row, obs_col)

	# Obtain all the coordinates of a rectangle in the world
	# given the to left corner, the number of desired rows,
	# and the number of desired columns in the rectangle.
	def getRectangleCoordinates(self, rows, cols, tl_coor):
		rectCoors = []

		# Add all rectangle coordinates to a list to be returned
		# that are in-bounds in the world.
		for i in range(tl_coor[0], tl_coor[0] + rows):
			for j in range(tl_coor[1], tl_coor[1] + cols):
				if out_of_bounds(i, j, len(self.world),
				len(self.world[0]) ) == False:
					rectCoors.append((i, j))

		return rectCoors

	# Returns the coordinates of a randomly generated
	# rectangular obstacle that has coordinates solely in
	# a region of the world that is currently empty
	def generate_rect_obstacle_at_empty_pos(self):

		# Generate an empty top-left corner for the obstacle
		tl_coor = self.generate_point_obstacle_at_empty_pos()

		# Get top left coordinates that we know are empty
		obs_tl_row = tl_coor[0]
		obs_tl_col = tl_coor[1]

		# Generate a bottom right corner. Here we enforce that no
		# obstacle has either dimension larger than 5 units in length
		row_last = np.random.randint(obs_tl_row, obs_tl_row + 5)
		col_last = np.random.randint(obs_tl_col, obs_tl_col + 5)

		rectangle_coordinates = []

		# Work up from the top left corner and fill in the largest rectangle
		# contained between the generated top left and bottom right corners
		# while ensuring that the region in the world corresponding to these
		# coordinates is empty.
		for i in range(obs_tl_row, row_last):
			for j in range(obs_tl_col, col_last):
				# Get rectangle coordinates
				rectCoor = self.getRectangleCoordinates(i - obs_tl_row + 1,
				j - obs_tl_col + 1, tl_coor)
				all_free = True
				# Check if all of these coordinates are empty in the world
				for k in range(len(rectCoor)):
					if self.world[rectCoor[k][0],
					rectCoor[k][1]] != WORLD['EMPTY']:
						all_free = False

				# If all of these coordinates are free, add them to the list
				# of rectangle coordinates to fill up
				if all_free == True:
					for coor in rectCoor:
						rectangle_coordinates.append(coor)
				# Otherwise, our rectangle is as big as it can be without
				# running into non-empty spaces, so we return the rectangle
				# coordinates
				else:
					return rectangle_coordinates

		# Return list of rectangle coordinates if we have not already
		return rectangle_coordinates

	# Add a rectangular obstacle to the world
	def add_rect_obstacle(self):

		# Generate obstacle and add it to the world
		obstacle_added = 0
		coordinates = self.generate_rect_obstacle_at_empty_pos()
		for coor in coordinates:
			self.world[coor[0], coor[1] ] = WORLD['OBSTACLE']

		# If no obstacle was actually created, indicate this
		if len(coordinates) == 0:
			return obstacle_added

		# Check if the obstacle can be added while keeping path
		# between goal and robot. If so, add it and indicate that
		# it was added. The existence of a path between the goal
		# and the robot is determined by whether the wavefront can
		# reach the robot location.
		if self.wave_front()[self.robot_loc[0],
		self.robot_loc[1]] != WORLD['EMPTY']:
			obstacle_added = 1
			self.obs_list.append(coordinates)
		# If the obstacle cannot be added without eliminating paths
		# between the robot and the goal, erase it from the world
		else:
			for coor in coordinates:
				self.world[coor[0], coor[1] ] = WORLD['EMPTY']

		return obstacle_added

	# Get wave_front coloring for the world using the wave front
	# potential method
	def wave_front(self):
		# Make copy of the world to fill in
		grid = copy.deepcopy(self.world)
		# Get robot and goal position
		robot_pos = self.robot_loc
		goal_pos = self.goal_loc

		# Treat the robot position as an empty cell that can be accessed
		# by the wave-front potential method
		grid[robot_pos[0], robot_pos[1]] = WORLD['EMPTY']

		# Initialize to 0 so we first check distance from goal position
		d = 0
		# Iterate until all spaces have been visited
		while getNumUnvisitedSpaces(grid) > 0:
			# Get current points to update
			current_points = getNeighborsD(grid, d)
			# Update distance until the robot position is reached
			# Then return the grid
			# If there are no points left to visit, terminate the algorithm
			if not current_points:
				return grid
			# If the robot position is on the horizon, set the distance
			# from the robot to the goal in the wavefront grid and terminate
			# the algorithm
			elif robot_pos in current_points:
				grid[robot_pos[0], robot_pos[1]] = d + 1
				return grid
			# Otherwise update the distances
			else:
				for i in range(len(current_points)):
					grid[current_points[i][0], current_points[i][1]] = d + 1

			# Go to next layer of distances
			d = d + 1

		return grid

	# Set a random robot location in an empty space in the world
	def set_random_robot_loc(self):
		row = np.random.randint(0, len(self.world) )
		col = np.random.randint(0, len(self.world[0]))

		while self.world[row, col] != WORLD['EMPTY']:
			row = np.random.randint(0, len(self.world) )
			col = np.random.randint(0, len(self.world[0]))

		self.world[row, col] = WORLD['ROBOT']
		self.robot_loc = (row, col)

		return

	# Set a random goal location in an empty space in the world
	def set_random_goal_loc(self):
		row = np.random.randint(0, len(self.world) )
		col = np.random.randint(0, len(self.world[0]))

		while self.world[row, col] != WORLD['EMPTY']:
			row = np.random.randint(0, len(self.world) )
			col = np.random.randint(0, len(self.world[0]))

		self.world[row, col] = WORLD['GOAL']
		self.goal_loc = (row, col)

		return

	# Set specific robot position in the grid
	def set_robot_position(self, loc_x, loc_y):
		# Sets robot to specific location (reducing randomness in
		# initialization)
		self.world[loc_x, loc_y] = WORLD['ROBOT']
		self.robot_loc = (loc_x, loc_y)
		return

	# Set specific robot position in the grid
	def set_goal_position(self, loc_x, loc_y):
		# Sets goal to specific location (reducing randomness in
		# initialization)
		self.world[loc_x, loc_y] = WORLD['GOAL']
		self.goal_loc = (loc_x, loc_y)
		return

	# Takes one hot vector input [LEFT, RIGHT, UP, DOWN]
	# Indicate whether the move was successful
	def take_action(self, action):
		# Set new robot position
		robot_pos = self.robot_loc
		moved = 0
		new_position = (robot_pos[0], robot_pos[1])

		# Move Left
		if action[0] == 1:
			new_position = (robot_pos[0], robot_pos[1] - 1)
			try_move = 1
		# Move Right
		elif action[1] == 1:
			new_position = (robot_pos[0], robot_pos[1] + 1)
			try_move = 1
		# Move Up
		elif action[2] == 1:
			new_position = (robot_pos[0] - 1, robot_pos[1])
			try_move = 1
		# Move Down
		else:
			new_position = (robot_pos[0] + 1, robot_pos[1])
			try_move = 1

		# Make a move if the move would not take the robot out of bounds.
		# If a move was successful, indicate this. Otherwise, indicate
		# that the robot could not be moved.
		if try_move == 1:
			if out_of_bounds(new_position[0], new_position[1],
			len(self.world), len(self.world[0])) == False:
				self.robot_loc = new_position
				self.world[new_position[0], new_position[1]] = WORLD['ROBOT']
				self.world[robot_pos[0], robot_pos[1]] = WORLD['EMPTY']
				moved = 1

		return moved

	# Returns a NxN neighborhood of the robot centered on the robot location
	# in the world. Out of bounds cells are treated as obstacles.
	def get_neighborhood_state(self, n):

		# Get row and column ranges for the neighborhood state grid
		# and initialize the neighborhood grid.
		rows = (self.robot_loc[0] - int(n/2), self.robot_loc[0] + int(n/2)  )
		cols = (self.robot_loc[1] - int(n/2), self.robot_loc[1] + int(n/2)  )
		neighborhood_grid = np.zeros((n, n))

		# Fill in neighborhood grid from spaces in the world around the robot
		# location.
		for row in range(rows[0], rows[1] + 1):
			for col in range(cols[0], cols[1] + 1):
				if out_of_bounds(row, col, len(self.world),
				len(self.world[0])  ) == False:
					neighborhood_grid[ row - rows[0],
					col - cols[0] ] = self.world[row][col]
				else:
					neighborhood_grid[ row - rows[0],
					col - cols[0] ] = WORLD['OBSTACLE']

		return neighborhood_grid

	# Returns unit vector to goal
	def get_vector_to_goal(self):
		return (self.goal_loc[0] - self.robot_loc[0],
		self.goal_loc[1] - self.robot_loc[1]) / np.linalg.norm(np.array(
		[self.goal_loc[0] - self.robot_loc[0],
		self.goal_loc[1] - self.robot_loc[1]]))

	# Returns Euclidean distance between robot and goal
	def get_distance_to_goal(self):
		vec_to_goal = (self.goal_loc[0] - self.robot_loc[0],
		self.goal_loc[1] - self.robot_loc[1])
		return np.sqrt( (vec_to_goal[0])**2 + (vec_to_goal[1])**2 )

	# Get the distance from the robot to the nearest obstacle or edge of the
	# world.
	def get_distance_to_closest_obstacle(self):

		# Determine minimum distance to allt he obstacles in the list
		# of obstacles
		robot_pos = self.robot_loc
		obstacle_dist = float('inf')
		for obs in self.obs_list:
			for coord in obs:
				obstacle_dist = min(obstacle_dist,
				np.sqrt( (coord[0] - robot_pos[0])**2 + (
				coord[1] - robot_pos[1])**2 ))

		# Get minimum distance to borders:
		row_border_dist = min(robot_pos[0],
		len(self.world) - robot_pos[0] - 1 ) + 1
		col_border_dist = min(robot_pos[1],
		len(self.world[0]) - robot_pos[1] - 1 ) + 1
		border_dist = min(row_border_dist, col_border_dist)

		# Return the minimum distance to a forbidden space (obstacle
		# or world border).
		return min(obstacle_dist, border_dist)

	# Display the world in a nice 2D format, with an X outside the
	# border of the world, an O for obstacles, a G for the goal,
	# an R for the robot, and a space character for an empty space
	# in the grid.
	def display_world(self):
		lineStr = ""
		for j in range(len(self.world[0]) + 2):
			lineStr += "X "
		print lineStr
		for i in range(len(self.world)):
			# Initialize string for that line
			lineStr = "X "
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
					lineStr += "  "
			print lineStr + "X "
		lineStr = ""
		for j in range(len(self.world[0]) + 2):
			lineStr += "X "
		print lineStr
		return

	# Return dictionary of robot location, goal location,
	# and obstacle locations in the world.
	def world_representation(self):
		return {
			'robot_loc': self.robot_loc,
			'goal_loc': self.goal_loc,
			'obs_list': self.obs_list
		}
