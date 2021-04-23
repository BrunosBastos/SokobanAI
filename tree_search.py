
import asyncio
import datetime
import heapq
from abc import ABC, abstractmethod
from copy import deepcopy

from pathfind import *

DEBUG = 0
DEBUG_PI = 0
DEBUG_TERMINALS = 0


class SearchProblem:
	"""Initial state of a problem with its domain."""
	
	def __init__(self, domain, initial):
		self.domain = domain		# Sokoban	-> domain of the problem
		self.initial = initial		# Map		-> initial map
		self.sym = self.symmetry(initial)
		
	def goal_test(self, mapa):
		"""Check if the level is completed."""
		return self.domain.satisfies(mapa)

	def symmetry(self, state):
		"""Check if the problem is symmetric."""
		hor_tiles, ver_tiles = state.size
		ml = mr = mt = mb = None

		for x in range(hor_tiles):
			for y in range(ver_tiles):
				if state.get_tile((x, y)) & 0b1000:
					mt = x
					break
			if mt:
				break
		for x in range(hor_tiles - 1, -1, -1):
			for y in range(ver_tiles):
				if state.get_tile((x, y)) & 0b1000:
					mb = hor_tiles-1 - x
					break
			if mb:
				break
		for y in range(ver_tiles):
			for x in range(hor_tiles):
				if state.get_tile((x, y)) & 0b1000:
					ml = y
					break
			if ml:
				break
		for y in range(ver_tiles - 1, -1, -1):
			for x in range(hor_tiles):
				if state.get_tile((x, y)) & 0b1000:
					mr = ver_tiles-1 - y
					break
			if mr:
				break
		
		if hor_tiles - mt - mb == ver_tiles - ml - mr:
			if all(
				state.get_tile((x, y)) & 0b1101 == state.get_tile((ver_tiles-1 - y, x)) & 0b1101
				for y in range(ml, ver_tiles - mr) for x in range(mt, hor_tiles - mb)
			):	# 90º rotation
				return '90-rotation'
		if all(
			state.get_tile((x, y)) & 0b1101 == state.get_tile((hor_tiles-1 - x, ver_tiles-1 - y)) & 0b1101
			for y in range(ml, ver_tiles - mr) for x in range(mt, hor_tiles - mb)
		):	# 180º rotation
			return '180-rotation'
		if all(
			state.get_tile((x, y)) & 0b1101 == state.get_tile((hor_tiles-1 - x, y)) & 0b1101
			for y in range(ml, ver_tiles - mr) for x in range(mt, hor_tiles - mb)
		):	# x reflection
			return 'x-reflection'
		if all(
			state.get_tile((x, y)) & 0b1101 == state.get_tile((x, ver_tiles-1 - y)) & 0b1101
			for y in range(ml, ver_tiles - mr) for x in range(mt, hor_tiles - mb)
		):	# y reflection
			return 'y-reflection'
		
		return None
		
	def symmetric_coords(self, coords, symmetry):
		x, y = coords
		hor_tiles, ver_tiles = self.initial.size

		if symmetry == '90-rotation':
			return (ver_tiles-1 - y, x)
		elif symmetry == '180-rotation':
			return (hor_tiles-1 - x, ver_tiles-1 - y)
		elif symmetry == 'x-reflection':
			return (hor_tiles-1 - x, y)
		elif symmetry == 'y-reflection':
			return (x, ver_tiles-1 - y)
		else:
			assert False

	def symmetric_backtrack(self, boxes, keeper, backtrack_states):
		""" Add to the backtrack the equivalent states rotated """

		if self.sym == '90-rotation':
			keeper1 = self.symmetric_coords(keeper, self.sym)
			keeper2 = self.symmetric_coords(keeper1, self.sym)
			keeper3 = self.symmetric_coords(keeper2, self.sym)
			boxes1 = []
			boxes2 = []
			boxes3 = []
			for box in boxes:
				box1 = self.symmetric_coords(box, self.sym)
				box2 = self.symmetric_coords(box1, self.sym)
				box3 = self.symmetric_coords(box2, self.sym)
				boxes1.append(box1)
				boxes2.append(box2)
				boxes3.append(box3)
			backtrack_states[hash(frozenset(boxes1))] = [keeper1]
			backtrack_states[hash(frozenset(boxes2))] = [keeper2]
			backtrack_states[hash(frozenset(boxes3))] = [keeper3]
		else:
			keeper1 = self.symmetric_coords(keeper, self.sym)
			boxes1 = []
			for box in boxes:
				box1 = self.symmetric_coords(box, self.sym)
				boxes1.append(box1)
			backtrack_states[hash(frozenset(boxes1))] = [keeper1]


class SearchNode:
	"""Contains the information of tree node."""
	
	def __init__(self, state, boxes, keeper, *, 
			parent=None, depth=None, cost=None,
			heuristic=None, action=None):
		self.state = state			# Map   -> map
		self.boxes = boxes			# list  -> position of the boxes
		self.keeper = keeper		# tuple -> position of the keeper
		self.parent = parent		# SearchNode -> parent node
		self.depth = depth			# int 		 -> depth of the node
		self.cost = cost			# int 	     -> acummulative cost of the node
		self.heuristic = heuristic	# int		 -> heuristic value of the state stored in the node
		self.action = action		# tuple		 -> last performed action

	def __str__(self):
		return (
f"""
{self.state}

depth:	   {self.depth}
cost:	   {self.cost}
heuristic: {self.heuristic}
action:	   {self.action}
"""
		)
		
	def __repr__(self):
		return str(self)


class SearchTree:
	"""Search Tree that contains the nodes."""

	# THRESHOLD to change the search strategy
	# when the number of non-terminal nodes is higher than the threshold the strategy changes
	STRATEGY_THRESHOLD = 10000

	def __init__(self, problem, strategy='breadth'): 
		self.problem = problem			# SearchProblem  -> problem with the initial map and the domain
		self.strategy = strategy		# String 		 -> strategy used to search  

	def get_path(self, node):
		""" Obtain the path from solution node to root node. """
		
		if node.parent == None:
			return ''
		# path from parent nodes
		parent_path = self.get_path(node.parent)
		x, y, dx, dy, d = node.action
		# keeper position immediately before pushing the box
		lpos_keeper = x + ((-1)**(dx > 0) if dx else 0), y + ((-1)**(dy > 0) if dy else 0)
		path = breadth_first_search(node.parent.keeper, lpos_keeper, node.parent.state) 
		return parent_path + path + d
			
	@property
	def length(self):
		return self.solution.depth
	
	@property
	def cost(self):
		return self.solution.cost 

	
	
	async def search(self, limit=None):
		"""Search the solution to the problem.

		:param limit: depth limit of the search tree 
		"""
		# self.end_time = datetime.datetime.now() + datetime.timedelta(minutes=6)
		
		# Root node is initialized here with all the attributes needed
		# It was done this way so that it is possible to use timeit to experiment different approaches
		boxes = self.problem.initial.boxes
		root = SearchNode(self.problem.initial, boxes, self.problem.initial.keeper, 
			parent=None, depth=0, cost=0, 
			heuristic=self.problem.domain.heuristic(self.problem.initial, boxes), action=None) 
		
		self.open_nodes = [root]	   # list(Nodes)  -> Contains the nodes that have not been open yet
		self.terminals = 1			   # int 		  -> Number of nodes waiting in open_nodes
		self.non_terminals = 0		   # int		  -> Number of nodes that have been open

		# set with all the states that were visited
		# this set uses an hash of the boxes and the keeper to save space
		self.backtrack_states = {} 

		while self.open_nodes:
			await asyncio.sleep(0)

			# if datetime.datetime.now() > self.end_time:
			# 	exit(1)
			
			# Removes the node in front of the list
			node = self.open_nodes.pop(0)

			if self.problem.goal_test(node.state): 			# Level is completed
				self.solution = node			   			# Save the current node as the solution
				self.terminals = len(self.open_nodes) + 1	# Solution node is terminal because it's not explored
				return self.get_path(node)					# Returns the full path to reach this state
			
			self.non_terminals += 1			# Increment when opening a node

			if self.non_terminals > self.STRATEGY_THRESHOLD: # Change the strategy to greedy
				self.strategy = 'greedy'					 

			lnewnodes = []		# List(Node) 	-> list of nodes obtained from ``node``
			pushes = self.problem.domain.actions(node) # List(tuple) -> lists with the possible actions
			if DEBUG:
				print(node, '\n')
			if DEBUG_TERMINALS and not pushes:
				print(node, '\n')

			for a in pushes: # Iterates the possible actions

				x, y, dx, dy, _ = a  # a is an action : (x,y,dx,dy,d)
				
				# performs the action in the node state and generates a newstate for the new node
				newstate = self.problem.domain.result(node.state, node.keeper, a)	    # Map
				newboxes = [b if b != (x, y) else (x + dx, y + dy) for b in node.boxes] # Moves the box
				newkeeper = (x, y)		# Saves the keeper position

				# Hash with the boxes of the new state
				state_hash = hash(frozenset(newboxes))

				if state_hash in self.backtrack_states:
					# state already searched
					if any(
						breadth_first_search(newkeeper, (kx, ky), newstate) is not None
						for kx, ky in self.backtrack_states[state_hash]
					):	# keeper in a region already searched
						continue
					else:
						# keeper in a region not yet searched
						self.backtrack_states[state_hash].append(newkeeper)
				else:
					if self.problem.sym:
						self.backtrack_states[state_hash] = [newkeeper]
						
						self.problem.symmetric_backtrack(newboxes, newkeeper, self.backtrack_states)
					else:
						self.backtrack_states[state_hash] = [newkeeper]

				# Calculates the heuristic value of the new state
				heuristic = self.problem.domain.heuristic(newstate, newboxes)

				# If the heuristic is infinite, some box is not able to get to a goal so it is an impossible state
				# If limit is being used, nodes with higher depth won't be explored
				if (limit is None or node.depth + 1 <= limit) and heuristic != math.inf:
				
					# Creates a newnode with the attributes needed
					newnode = SearchNode(newstate, newboxes, newkeeper,
						parent=node, depth=(node.depth + 1), cost=(node.cost + 1),
						heuristic=heuristic, action=a)
					
					# adds it to the list of new nodes
					lnewnodes.append(newnode)
			
			# Adds the list to the open nodes according to the heuristic 		   
			self.add_to_open(lnewnodes) 
			
		# If no solution is found the function returns None  
		return None
		
	def add_to_open(self, lnewnodes):
		"""Add nodes to ``open_nodes`` according the strategy."""
		
		if self.strategy == 'breadth':
			self.open_nodes.extend(lnewnodes)
		elif self.strategy == 'depth':
			self.open_nodes[:0] = lnewnodes
		elif self.strategy == 'uniform':
			self.open_nodes.extend(lnewnodes)
			self.open_nodes.sort(key=lambda node: node.cost)
		elif self.strategy == 'greedy':
			self.open_nodes.extend(lnewnodes)
			self.open_nodes.sort(key=lambda node: node.heuristic)
		elif self.strategy == 'a*':
			self.open_nodes.extend(lnewnodes)
			self.open_nodes.sort(key=lambda node: node.cost + 2*node.heuristic)

