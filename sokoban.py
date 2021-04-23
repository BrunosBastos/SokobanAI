import math
from tree_search import *
from pathfind import *
from assignment_problem import hungarian

class Sokoban:
	def __init__(self):
		pass

	def set_mapa(self, state):
		self.state = state

		# checks if the level is symmetric
		
		self.storages = state.filter_tiles([Tiles.GOAL, Tiles.MAN_ON_GOAL, Tiles.BOX_ON_GOAL])
		self.dead_squares = self.simple_deadlock(state)

		self.dys_distances = self.dynamic_distances(state)

		self.heuristic = self.greedy_heuristic
		self.blocked_distances = None


		# next operations are very slow, so the map needs to be small for it to be used
		if not (state.size[0]*state.size[1]>1000 or len(self.storages)>20):	
		
			# checks if the distances are the same with walls or without them 
			for storage in self.storages:
				for box in state.boxes:
					x, y = box
					flag = False
					if self.dys_distances[storage][y][x] != manhattan(storage,box):
						flag = True					
						self.heuristic = self.hungarian_heuristic
						break
				if flag:
					break

			self.blocked_distances = self.macro_goals(state)


	def actions(self, node):
		"""Return all possible actions for a state."""

		# if a box reaches a goal see if it blocks other goals for the other boxes
		if self.blocked_distances and node.action and node.state.get_tile( (node.action[0]+node.action[2], node.action[1]+node.action[3]) ) == 0b0101:
			x, y = (node.action[0]+node.action[2], node.action[1]+node.action[3])

			if self.freeze_deadlock(node.parent.state, node.parent.keeper, node.action, False):
				if self.hungarian_heuristic(node.state, node.boxes, self.blocked_distances[(x,y)]) == math.inf:
					return []

		# map with all the squares reachable by the keeper
		visited = breadth_first_search(node.keeper, None, node.state)


		actions = []
		for x, y in node.boxes:
			for dx, dy, d in [(-1, 0, "a"), (0, 1, "s"), (1, 0, "d"), (0, -1, "w")]:
				action = (x, y, dx, dy, d)
				if (
					not node.state.get_tile((x + dx, y + dy)) & 0b1100
					and visited[y - dy][x - dx]
					and not self.dead_squares[y + dy][x + dx]
				):	# one side of the box is empty and not dead square
					# the opposite side is reachable
					"""
					steps = self.tunnel(node.state, action)
					action = (x, y, dx*steps, dy*steps, d*steps)
					"""
					# tile = node.state.get_tile(storage)
					# node.state.set_tile(storage, 0b1000)
					if not self.freeze_deadlock(node.state, node.keeper, action):
						actions.append(action)
		
		# return the actions if there 
		return self.pi_corral_deadlock(node, actions)
	

	def result(self,state,keeper,action):
		"""Perform an action in the new copy of the map and returns it."""

		# creates a new copy of the state to generate the new state
		state = deepcopy(state)
		x, y, dx, dy, _ = action
		cpos_box = x, y 
		# keeper position immediately before pushing the box
		# this position works for tunnels and the normal case
		# it goes to the position of the box being moved
		npos_keeper = x + dx + ((-1)**(dx > 0) if dx else 0), y + dy + ((-1)**(dy > 0) if dy else 0)
		npos_box = x + dx, y + dy 

		# performs the given action by moving the box and the player
		state.clear_tile(keeper)
		state.clear_tile(cpos_box)
		state.set_tile(npos_keeper, Tiles.MAN)
		state.set_tile(npos_box, Tiles.BOX)
		return state


	def cost(self, node, action):
		"""Calculate the number of steps to perform an action. 

		This method is not being used right now but could still be usefull	
		"""
		x, y, dx, dy, d = action
		lpos_keeper = x + dx + ((-1)**(dx > 0) if dx else 0) - dx, y + dy + ((-1)**(dy > 0) if dy else 0) - dy
		path = breadth_first_search(node.keeper, lpos_keeper, node.state) 
		return len(path) + 1


	def hungarian_heuristic(self, state, boxes, heur_distances=None):
		"""Calculate the minimum distance from boxes to goals with minimum matching. 

		This method calculates the minimum perfect match distance between boxes and goals
		having into account the walls in the way.
		It returns a precise measurement of the state but it's slow, so it shouldn't be used
		when the walls don't influenciate the distance to the storages.
		"""
		
		# if heur_distances is not passed as a parameter it initializes it
		if heur_distances is None:
			heur_distances = self.dys_distances

		# initializes a table full of zeros to be used on the hungarian function
		table = [[0]*len(boxes) for _ in range(len(self.storages))]

		# calculates the heuristic using the hungarian method
		for i, storage in enumerate(self.storages):
			for j, box in enumerate(boxes):
				table[i][j] = heur_distances[storage][box[1]][box[0]]
		h = hungarian(table, len(table[0]))
	
		return h


	def greedy_heuristic(self, state, boxes):
		"""Calculate the minimum distance from boxes to goals greedly. 

		Chooses the closest box to each goal. However, each box has a goal assigned.
		Returns the sum of distance of each goal-box pair. It is fast, however it does
		not measure the node state with great accuracy.
		"""

		heuristic = 0
		# set of boxes to store those who have already been selected
		chosen_boxes = set()

		for i, storage in enumerate(self.storages):
			m, min_distance = 0, math.inf			
			# chose a box not assigned with the minimum distance
			for j, box in enumerate(boxes):
				if j in chosen_boxes:
					continue
				distance = manhattan(box, storage)
				if distance < min_distance:
					m, min_distance = j, distance
			chosen_boxes.add(m)
			heuristic += min_distance
		return heuristic


	def satisfies(self, state):
		""" Check if the state represents a winning state """
		return all( state.get_tile((x, y)) & 0b0100 for (x, y) in self.storages )


	def dynamic_distances(self, state, flag=True):
		"""Pre-calculate the distances respecting the walls and pushes.
		
		This is useful to make a dynamic update on the heuristic distance to the solution.

		If ``flag`` is True, the search expands from a storage to all reachable position,
		and the shrinks back to the storage. This must be done as there are positions where the
		box must be pushed on an opposite direction in the beginning.

		@param state: the map
		@param flag: a flag to check if the player reach a push
		"""
		hor_tiles, ver_tiles = state.size

		open_nodes = []
		dynamic_distances = {k: [] for k in self.storages}
		for storage in self.storages:
			distances = [[math.inf] * hor_tiles for _ in range(ver_tiles)]
			open_nodes = [(storage[0], storage[1], 0, 0, 0)]
			
			if state.get_tile(storage) & 0b1000:				##
				distances[storage[1]][storage[0]] = 0	##
			else:
				while open_nodes:
					x, y, dx, dy, dist = open_nodes.pop(0)
					bx, by = x + dx, y + dy
					px, py = x + 2*dx, y + 2*dy
					
					if self.dead_squares[by][bx] or (distances[by][bx] != math.inf and (flag or distances[by][bx] >= distances[y][x])):
						continue

					distances[by][bx] = min(dist, distances[by][bx])

					# continue search if two squares behind the box are empty
					if bx in range(hor_tiles) and by + 2 in range(ver_tiles) and not (state.get_tile((bx, by + 2)) & 0b1000) and not (state.get_tile((bx, by + 1)) & 0b1000):
						if flag or ((dx, dy) != (0, -1) and (dx, dy) == (0, 0) or breadth_first_search((px, py), (bx, by + 1), state, wall=0b1000, walls=[(bx, by)]) is not None):
							open_nodes.append((bx, by, 0, 1, dist + 1))
					if bx + 2 in range(hor_tiles) and by in range(ver_tiles) and not (state.get_tile((bx + 2, by)) & 0b1000) and not (state.get_tile((bx + 1, by)) & 0b1000):
						if flag or ((dx, dy) != (-1, 0) and (dx, dy) == (0, 0) or breadth_first_search((px, py), (bx + 1, by), state, wall=0b1000, walls=[(bx, by)]) is not None):
							open_nodes.append((bx, by, 1, 0, dist + 1))
					if bx in range(hor_tiles) and by - 2 in range(ver_tiles) and not (state.get_tile((bx, by - 2)) & 0b1000) and not (state.get_tile((bx, by - 1)) & 0b1000):
						if flag or ((dx, dy) != (0, 1) and (dx, dy) == (0, 0) or breadth_first_search((px, py), (bx, by - 1), state, wall=0b1000, walls=[(bx, by)]) is not None):
							open_nodes.append((bx, by, 0, -1, dist + 1))
					if bx - 2 in range(hor_tiles) and by in range(ver_tiles) and not (state.get_tile((bx - 2, by)) & 0b1000) and not (state.get_tile((bx - 1, by)) & 0b1000):
						if flag or ((dx, dy) != (1, 0) and (dx, dy) == (0, 0) or breadth_first_search((px, py), (bx - 1, by), state, wall=0b1000, walls=[(bx, by)]) is not None):
							open_nodes.append((bx, by, -1, 0, dist + 1))
			
			dynamic_distances[storage] = distances
		return dynamic_distances



	def macro_goals(self, state):
		""" Checks if putting a box in a  """
		
		blocked_distances = {}
		for storage in self.storages:
			tile = state.get_tile(storage)
			state.set_tile(storage, 0b1000)
			blocked_distances[storage] = self.dynamic_distances(state, False)
			state.clear_tile(storage)
			state.set_tile(storage, tile)
		return blocked_distances


	def simple_deadlock(self, state):
		"""Mark squares where boxes can reach to any goal."""


		hor_tiles, ver_tiles = state.size

		# bi-dimensional list that stores the positions where boxes can reach at least one goal
		# Positions with 1 are not reachable and are considered dead squares
		unvisited = [[1] * hor_tiles for _ in range(ver_tiles)]

		# iterative breath first search to find the unreacheable positions starting at a goal
		# and doing the opposite movement ( pulling the box instead of pushing )
		open_nodes = []
		for storage in self.storages:
			open_nodes = [storage]
			while open_nodes:
				x, y = open_nodes.pop()
				
				if not unvisited[y][x]:
					continue

				unvisited[y][x] = 0		# mark as visited

				# continue search if two squares behind the box are empty 
				if x in range(hor_tiles) and y + 2 in range(ver_tiles) and not (state.get_tile((x, y + 2)) & 0b1000) and not (state.get_tile((x, y + 1)) & 0b1000):
					open_nodes.append((x, y + 1))
				if x + 2 in range(hor_tiles) and y in range(ver_tiles) and not (state.get_tile((x + 2, y)) & 0b1000) and not (state.get_tile((x + 1, y)) & 0b1000):
					open_nodes.append((x + 1, y))
				if x in range(hor_tiles) and y - 2 in range(ver_tiles) and not (state.get_tile((x, y - 2)) & 0b1000) and not (state.get_tile((x, y - 1)) & 0b1000):
					open_nodes.append((x, y - 1))
				if x - 2 in range(hor_tiles) and y in range(ver_tiles) and not (state.get_tile((x - 2, y)) & 0b1000) and not (state.get_tile((x - 1, y)) & 0b1000):
					open_nodes.append((x - 1, y))
		return unvisited

	

	def freeze_deadlock(self, state, keeper, action, ignore_goals=True):
		"""Check if a given action freezes a box.
		
		@param state: the map
		@param keeper: the keeper's position
		@param action: the last action performed on the state
		@param ignore_goals: a flag to ignore the blocked boxes that are on a goal
		"""
		def is_blocked(state, box, ignore_goals):
			x, y = box

			vis_boxes = {(x, y)} 

			# used to see if there is at least one box outside of a goal
			global_any_off_goal = not state.get_tile((x, y)) & 0b1

			# used to see if the box is blocked
			global_blocked = 0

			# x-axis
			if state.get_tile((x - 1, y)) & 0b1000 or state.get_tile((x + 1, y)) & 0b1000:
				# blocked by a wall on any side
				global_blocked |= 0b01
			elif self.dead_squares[y][x - 1] and self.dead_squares[y][x + 1]:
				# blocked by a dead square on both sides
				global_blocked |= 0b01
			elif not state.get_tile((x - 1, y)) & 0b1100 and not state.get_tile((x + 1, y)) & 0b1100:
				return False
			# y-axis
			if state.get_tile((x, y - 1)) & 0b1000 or state.get_tile((x, y + 1)) & 0b1000:
				# blocked by a wall on any side
				global_blocked |= 0b10
			elif self.dead_squares[y - 1][x] and self.dead_squares[y + 1][x]:
				# blocked by a dead square on both sides
				global_blocked |= 0b10
			elif not state.get_tile((x, y - 1)) & 0b1100 and not state.get_tile((x, y + 1)) & 0b1100:
				return False

			if global_blocked == 0b11 and (global_any_off_goal or not ignore_goals):
				# blocked locally on both axis and one box off goal
				return True

			# prepare recursive search for every adjacent box
			open_boxes = []
			# not blocked yet or no box off goal
			for dx in (-1, 1):
				if state.get_tile((x + dx, y)) & 0b0100:
					open_boxes.append( (x + dx, y, (global_blocked & 0b10) >> 1, 0b01, False, global_any_off_goal) )
			for dy in (-1, 1):
				if state.get_tile((x, y + dy)) & 0b0100:
					open_boxes.append( (x, y + dy, (global_blocked & 0b01) << 1, 0b10, True, global_any_off_goal) )

			# search on other boxes iterative
			while open_boxes:
				x, y, blocked, blocked_axis, search_x_axis, any_off_goal = open_boxes.pop()

				vis_boxes.add((x, y))
				# checks if the box is on goal and update the any_off_goal
				any_off_goal |= not state.get_tile((x, y)) & 0b1

				# sees if it's blocked on x-axis if True else y-axis 
				if search_x_axis:
					if (
						(state.get_tile((x - 1, y)) & 0b1000 or state.get_tile((x + 1, y)) & 0b1000)
						or (self.dead_squares[y][x - 1] and self.dead_squares[y][x + 1])
						or ((x - 1, y) in vis_boxes or (x + 1, y) in vis_boxes)
					):	# blocked by a wall on any side
						# or blocked by dead squares on both sides
						# or blocked by a box on any side
						
						# local blocked updates x-axis
						blocked |= 0b01
						global_blocked |= blocked_axis
						global_any_off_goal |= any_off_goal
					elif not state.get_tile((x - 1, y)) & 0b1100 and not state.get_tile((x + 1, y)) & 0b1100:
						continue
				else:
					if (
						(state.get_tile((x, y - 1)) & 0b1000 or state.get_tile((x, y + 1)) & 0b1000)
						or (self.dead_squares[y - 1][x] and self.dead_squares[y + 1][x])
						or ((x, y - 1) in vis_boxes or (x, y + 1) in vis_boxes)
					):	# blocked by a wall on any side
						# or blocked by dead squares on both sides
						# or blocked by a box on any side
						blocked |= 0b10
						global_blocked |= blocked_axis
						global_any_off_goal |= any_off_goal
					elif not state.get_tile((x, y - 1)) & 0b1100 and not state.get_tile((x, y + 1)) & 0b1100:
						continue
				
				if blocked == 0b11 and (any_off_goal or not ignore_goals):
					# blocked on both axis and one box off goal
					return True
				
				if global_blocked == 0b11 and (global_any_off_goal or not ignore_goals):
					return True
				
				# if it is not blocked and there are other boxes around, search for them
				if search_x_axis:
					for dx in (-1, 1):
						if (x + dx, y) not in vis_boxes and state.get_tile((x + dx, y)) & 0b0100:
							open_boxes.append( (x + dx, y, (blocked & 0b10) >> 1, blocked_axis, False, any_off_goal) )
				else:
					for dy in (-1, 1):
						if (x, y + dy) not in vis_boxes and state.get_tile((x, y + dy)) & 0b0100:
							open_boxes.append( (x, y + dy, (blocked & 0b01) << 1, blocked_axis, True, any_off_goal) )
			return False

		x, y, dx, dy, _ = action
		cpos_box = x, y 
		npos_keeper = x + dx + ((-1)**(dx > 0) if dx else 0), y + dy + ((-1)**(dy > 0) if dy else 0)
		npos_box = x + dx, y + dy 
		
		# do action
		state.clear_tile(keeper)
		state.clear_tile(cpos_box)
		state.set_tile(npos_keeper, Tiles.MAN)
		state.set_tile(npos_box, Tiles.BOX)

		blocked = is_blocked(state, (x + dx, y + dy), ignore_goals)

		# undo action
		state.clear_tile(npos_keeper)
		state.clear_tile(npos_box)
		state.set_tile(keeper, Tiles.MAN)
		state.set_tile(cpos_box, Tiles.BOX)

		return blocked




	def pi_corral_deadlock(self,node,actions):
		"""Prune actions if a pi-corral is found."""
		
		zones = self.multi_corral_zones(node)
		hor_tiles, ver_tiles = node.state.size

		# separate boxes by unique colors (corrals)
		corrals = {}
		
		# When using the multi_corral_zones
		#for box in node.boxes:
		#	x, y = box
		#	color = zones[y][x]
		#	if color not in corrals:
		#		corrals[color] = {(x,y)}
		#	else:
		#		corrals[color].add((x,y))
		
		# When using single_corral_zones 
		# 
		# Each corral zone is represented with a bit
		# Some boxes can be part of multiple colors so they have different bits = 1
		# This boxes must be stored in the different zone lists 
		# The rest of the boxes goes to the list of their respective color 
		#
		for box in node.boxes:
		 	x, y = box
		 	color = zones[y][x]
		 	i = 0b10
		 	while i <= color:
		 		if color & i:
		 			if i not in corrals:
		 				corrals[i] = {(x,y)}
		 			else:
		 				corrals[i].add((x,y))
		 		i <<= 1


		# filter i-corrals
		# 
		# i-corrals are corrals where all its boxes only have inwards pushes
		for color in list(corrals.keys()):
			for action in actions:
				x,y,dx,dy,_ = action
				if (x,y) in corrals[color]:
					# remove corrals which have boxes with moves outside the corral
					if zones[y+dy][x+dx] != color:
						del corrals[color]
						break
		

		# filter pi-corrals
		# 
		# pi-corrals are i-corrals that removing the all the boxes that are not part of that corral,
		# the number of possible pushes stays the same
		pi_corrals = dict(corrals)
		
		for color in list(pi_corrals.keys()):
			newstate = deepcopy(node.state) # TODO: avoid deepcopying twice with freeze
			newboxes = list(node.boxes)

			# remove boxes outside corral
			for box in node.boxes:
				x, y = box					
				if not zones[y][x] & color:
					newstate.clear_tile((x,y))
					newboxes.remove(box)

			# remove corrals which have more actions than before
			visited = breadth_first_search(node.keeper, None, newstate)
			for box in pi_corrals[color]:
				x, y = box
				flag = False
				for dx, dy, d in [(-1, 0, 'a'), (0, 1, 's'), (1, 0, 'd'), (0, -1, 'w')]:
					action = (x, y, dx, dy, d)
					if action not in actions:
						if (
							not newstate.get_tile((x + dx, y + dy)) & 0b1100
							and visited[y - dy][x - dx]
							and not self.dead_squares[y + dy][x + dx]
							and not self.freeze_deadlock(newstate, node.keeper, action)
						):
							del pi_corrals[color]
							flag = True
							break
				if flag:
					break
			if flag:
				continue

			# pi-corral prunning
			# 
			# if the i-corral is a pi-corral it can only be prunned if there is at least one box
			# part of that corral that is not on a goal or when there is an empty goal inside the corral
			#
			if not (
				all((node.state.get_tile(box) & 0b1) for box in pi_corrals[color])
				and all((node.state.get_tile((x, y)) & 0b0100) for (x, y) in self.storages if zones[y][x] & color)
			):	# all boxes on goal and all goals with boxes
				pruned_actions = []
				corral_boxes = pi_corrals[color]
				
				# the prunning then returns only the actions corresponding to boxes inside
				# the pi-corral
				for action in actions:
					x, y, _, _, _ = action
					if (x, y) in corral_boxes:
						pruned_actions.append(action)
				return pruned_actions

		# if there are no pi-corrals or they cannot be prunned the actions are not changed
		return actions


	def multi_corral_zones(self, node):
		"""Paint map corral regions with different colors represented by bits.
		Neighbour regions are assembled together.
		"""
		state = node.state
		kx,ky = node.keeper
		# map with all the squares reachable by the keeper
		visited = breadth_first_search(state.keeper, None, state)

		hor_tiles, ver_tiles = state.size

		counter = 0b10
		for y in range(ver_tiles):
			for x in range(hor_tiles):
				if visited[y][x] == 0 and not state.get_tile((x,y)) & 0b1100:
					# not painted or reachable and not box or wall
					queue = [(x,y,False)]
					while queue:
						nx,ny,lbox = queue.pop()

						if state.get_tile((nx,ny)) & 0b1000:
							# is a wall
							continue
						if visited[ny][nx] == 1:
							# is reached by the player
							continue
						if visited[ny][nx] & counter == counter:
							# already painted by the same color
							continue
						if lbox:
							if state.get_tile((nx,ny)) & 0b0100:
								blocked = 0
								if state.get_tile((nx - 1, ny)) & 0b1100 or state.get_tile((nx + 1, ny)) & 0b1100:
									blocked |= 0b01
								elif self.dead_squares[ny][nx - 1] and self.dead_squares[ny][nx + 1]:
									blocked |= 0b01
								if state.get_tile((nx, ny - 1)) & 0b1100 or state.get_tile((nx, ny + 1)) & 0b1100:
									blocked |= 0b10
								elif self.dead_squares[ny - 1][nx] and self.dead_squares[ny + 1][nx]:
									blocked |= 0b10

								if blocked == 0b11:
									#visited[ny][nx] |= counter
									colors_x = 0
									for dx in (-1, 1):
										if state.get_tile((nx + dx, ny)) & 0b0100:
											colors_x |= visited[ny][nx + dx]
									colors_y = 0
									for dy in (-1, 1):
										if state.get_tile((nx, ny + dy)) & 0b0100:
											colors_y |= visited[ny + dy][nx]
									# paint with the common colors between axis from the boxes which are blocking
									if colors_x & colors_y:
										visited[ny][nx] |= colors_x & colors_y
									else:
										continue
								else:
									continue
							else:
								visited[ny][nx] |= counter	# paint square
						else:
							visited[ny][nx] |= counter	# paint square

						if 0 <= ny - 1 < ver_tiles:
							queue.append((nx, ny - 1, state.get_tile((nx,ny)) & 0b0100))
						if 0 <= nx - 1 < hor_tiles:
							queue.append((nx - 1, ny, state.get_tile((nx,ny)) & 0b0100))
						if 0 <= ny + 1 < ver_tiles:
							queue.append((nx, ny + 1, state.get_tile((nx,ny)) & 0b0100))
						if 0 <= nx + 1 < hor_tiles:
							queue.append((nx + 1, ny, state.get_tile((nx,ny)) & 0b0100))
					counter <<= 1
		return visited


	def single_corral_zones(self, node):
		"""Paint map corral regions with different colors represented by bits."""

		state = node.state
		kx,ky = node.keeper
		
		# map with all the squares reachable by the keeper
		# reachable squares are marked with an 1
		# unreachable squares are marked with a 0
		visited = breadth_first_search(state.keeper, None, state)

		hor_tiles, ver_tiles = state.size

		# each zone has a corresponding bit ( starting at 2 since 1 means that the position is reachable by the player)
		counter = 0b10
		for y in range(ver_tiles):
			for x in range(hor_tiles):

				# not painted or reachable and not box or wall
				if visited[y][x] == 0 and not state.get_tile((x,y)) & 0b1100:

					# iterative search to paint all surrounding unreachable empty spaces with the color bit
					queue = [(x,y)]
					while queue:
						nx,ny = queue.pop()

						if state.get_tile((nx,ny)) & 0b1000:
							continue
						if state.get_tile((nx,ny)) & 0b0100:
							visited[ny][nx] |= counter
							continue
						if visited[ny][nx] & counter == counter:
							# already painted by the same color
							continue
							
						visited[ny][nx] |= counter	# paint square

						if 0 <= ny - 1 < ver_tiles:
							queue.append((nx, ny - 1))
						if 0 <= nx - 1 < hor_tiles:
							queue.append((nx - 1, ny))
						if 0 <= ny + 1 < ver_tiles:
							queue.append((nx, ny + 1))
						if 0 <= nx + 1 < hor_tiles:
							queue.append((nx + 1, ny))	
					counter <<= 1

		# iterates the boxes that are not painted
		for box in node.boxes:
			x, y = box
			if visited[y][x] == 0:
				# not painted

				# blocked is a 2bit number where the left bit represents the y-axis and the other the x-axis
				blocked = 0
				# checks if the box is locally blocked ( walls or dead squares)
				if state.get_tile((x - 1, y)) & 0b1100 or state.get_tile((x + 1, y)) & 0b1100:
					blocked |= 0b01
				elif self.dead_squares[y][x - 1] and self.dead_squares[y][x + 1]:
					blocked |= 0b01
				if state.get_tile((x, y - 1)) & 0b1100 or state.get_tile((x, y + 1)) & 0b1100:
					blocked |= 0b10
				elif self.dead_squares[y - 1][x] and self.dead_squares[y + 1][x]:
					blocked |= 0b10

				if blocked == 0b11:
					# blocked locally on both axis

					# if it is blocked it will paint the box with the color of the neighbour boxes
					colors_x = 0
					for dx in (-1, 1):
						if state.get_tile((x + dx, y)) & 0b0100:
							colors_x |= visited[y][x + dx]
					colors_y = 0
					for dy in (-1, 1):
						if state.get_tile((x, y + dy)) & 0b0100:
							colors_y |= visited[y + dy][x]
					# paint with the common colors between axis from the boxes which are blocking
					visited[y][x] |= colors_x & colors_y
		
		# returns the color map
		return visited
	
	# TODO: Not being used, but could be useful for specific levels. Fix errors!
	def tunnel(self, state, action):
		"""Check if a given action is on a tunnel.
		If so, return the steps needed to go traverse the tunnel till the end.

		It is considered a tunnel while the tiles on the side are walls and no goal is reached.

		@param state : the state being used
		@param action: the action that would be performed without consedering if on tunnel.
		"""
		x, y, dx, dy, d = action
		hor_tiles, ver_tiles = state.size

		steps = 1
		while True:
			if dx != 0:
				# x axis	
				if not (state.get_tile((x + dx*steps, y - 1)) & 0b1000 and state.get_tile((x + dx*steps, y + 1)) & 0b1000) or state.get_tile((x + dx*steps, y)) & 0b1:
					# has not walls on both sides or over a goal
					break
				if (
					not (state.get_tile((x + dx*(steps + 1), y - 1)) & 0b1000 or state.get_tile((x + dx*(steps + 1), y + 1)) & 0b1000) 
					or state.get_tile((x + dx*(steps + 1), y)) & 0b1100 or self.dead_squares[y][x + dx*(steps + 1)]
				):	# has not walls on either side or blocked n steps ahead
					break
			else:	
				# y axis
				if not (state.get_tile((x - 1, y + dy*steps)) & 0b1000 and state.get_tile((x + 1, y + dy*steps)) & 0b1000) or state.get_tile((x, y + dy*steps)) & 0b1:
					# has not walls on both sides or over a goal
					break
				if (
					not (state.get_tile((x - 1, y + dy*(steps + 1))) & 0b1000 or state.get_tile((x + 1, y + dy*(steps + 1))) & 0b1000)
					or state.get_tile((x, y + dy*(steps + 1))) & 0b1100 or self.dead_squares[y + dy*(steps + 1)][x]
				):	# has not walls on either side or blocked n steps ahead
					break
			steps += 1
		return steps
		