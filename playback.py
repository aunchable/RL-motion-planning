import numpy as np

## This file takes a logged episode and plays it back

WORLD = {
	'EMPTY': -3,
	'OBSTACLE': -2,
	'ROBOT': -1,
	'GOAL': 0
}

EPISODE_NUM = 30

with open("logging/episode"+str(EPISODE_NUM)+".txt", 'r') as f:
	s = f.readline()
actions = np.loadtxt("logging/episode"+str(EPISODE_NUM)+".txt", skiprows =1)

s = (s.replace("+", "").replace("e", "").replace("\n", "").\
	replace(".", "").replace("0", "").split(" "))

clean_state = []
for i, st in enumerate(s):
	if st == "-1":
		player_loc = i / 10, i % 10
	if len(st) > 0:
		clean_state.append(st)
	else:
		clean_state.append(0)
world = np.array(map(int, clean_state)).reshape((10,10))

# s = np.fromstring(s)
print clean_state

print actions
if len(actions.shape) == 1:
	actions = np.array([actions])

def display_world(world):
	lineStr = ""
	for j in range(len(world[0]) + 2):
		lineStr += "X "
	print lineStr
	for i in range(len(world)):
		# Initialize string for that line
		lineStr = "X "
		for j in range(len(world[i])):
			# Print O for obstacle, G for goal,
			# R for robot, and E for empty
			if world[i][j] == WORLD['OBSTACLE']:
				lineStr += "O "
			elif world[i][j] == WORLD['GOAL']:
				lineStr += "G "
			elif world[i][j] == WORLD['ROBOT']:
				lineStr += "R "
			elif world[i][j] == WORLD['EMPTY']:
				lineStr += "  "
			else:
				lineStr += (str(world[i][j]) + " ")
		print lineStr + "X "
	lineStr = ""
	for j in range(len(world[0]) + 2):
		lineStr += "X "
	print lineStr
	return

# ballloc
# for action
display_world(world)
x, y = player_loc
k = 1
for a in actions:
	player_loc = x, y
	print x, y
	index = np.argmax(a)
	if index == 0:
		x, y = x , y-1
	elif index == 1:
		x, y = x , y+1
	elif index == 2:
		x, y = x-1 , y
	else:
		x, y = x+1 , y
	x = max(0, x)
	y = max(0, y)
	try:
		if world[x, y] != WORLD['OBSTACLE']:
			world[x, y] = k
		else:
			x, y = player_loc
	except:
		x, y = player_loc
	k += 1


display_world(world)
print player_loc
