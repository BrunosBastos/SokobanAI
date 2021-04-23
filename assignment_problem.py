import re
import math


def hungarian(table, n):
	"""Solve the assignement problem given a table n x n with the
	Hungarian algorithm.
	"""
	m2 = [list(row) for row in table]
	
	# step 1: subtract minimum of every row
	for i in range(n):
		min_y = min(m2[i])
		if min_y == math.inf:
			return math.inf
		for x in range(n):
			m2[i][x] -= min_y

	# step 2: subtract minimum of every column
	for i in range(n):
		min_x = min(m2[y][i] for y in range(n))
		if min_x == math.inf:
			return math.inf
		for y in range(n):
			m2[y][i] -= min_x

	while True:
		# step 3: discover minimum number of horizontal and vertical
		# lines to cover all zeros using the minimum vertex cover
		left_edges = {}
		right_edges = {}
		for y in range(n):
			for x in range(n):
				if m2[y][x] == 0:
					row, col = 'R' + str(y), 'C' + str(x)
					if row in left_edges:
						left_edges[row].append(col)
					else:
						left_edges[row] = [col]
					if col in right_edges:
						right_edges[col].append(row)
					else:
						right_edges[col] = [row]
		mvc = min_vertex_cover(left_edges, right_edges)

		# step 4: check if the number of lines needed is less than `n`
		# If so, jump to step 6, else continue to step 5
		if len(mvc) == n:
			break

		# step 5: subtract the smallest uncovered entry from all
		# uncovered elements and add it to all elements that are
		# covered twice
		m3 = [[0]*n for _ in range(n)]
		for k in mvc.keys():
			if k[0] == 'R':
				for x in range(n):
					m3[int(k[1:])][x] += 1
			else:
				for y in range(n):
					m3[y][int(k[1:])] += 1

		min_uncovered = math.inf
		for y in range(n):
			for x in range(n):
				if m3[y][x] == 0:
					min_uncovered = min(min_uncovered, m2[y][x])

		if min_uncovered == math.inf:
			return math.inf

		for y in range(n):
			for x in range(n):
				if m3[y][x] == 0:
					m2[y][x] -= min_uncovered
				elif m3[y][x] == 2:
					m2[y][x] += min_uncovered

	# step 6: get optimal match
	matching = []
	m3 = [[0]*n for _ in range(n)]
	for y in range(n):
		for x in range(n):
			m3[y][x] = m2[y][x] == 0

	while True:
		# assign single zeros on rows or columns
		assigned = False
		for i in range(n):
			if m3[i].count(1) == 1:
				x = m3[i].index(1)
				matching.append((i, x, table[i][x]))
				for j in range(n):
					m3[i][j] = 0
					m3[j][x] = 0
				assigned = True
				break
			
			if [m3[y][i] for y in range(n)].count(1) == 1: # TODO: improve to avoid index
				y = [m3[y][i] for y in range(n)].index(1)
				matching.append((y, i, table[y][i]))
				for j in range(n):
					m3[y][j] = 0
					m3[j][i] = 0
				assigned = True
				break

		if not any(c for row in m3 for c in row): # finish
			break
		if assigned: # restart
			continue

		# assign arbitraly one zero to untie
		flag = False
		for y in range(n):
			for x in range(n):
				if m3[y][x] == 1:
					matching.append((y, x, table[y][x]))
					for i in range(n):
						m3[y][i] = 0
						m3[i][x] = 0
					flag = True
					break
			if flag:
				break
		
	return sum([m[2] for m in matching])


def bipartite_match(graph_edges):
	"""Find maximum cardinality matching of a bipartite graph (U, V, E).

	@param graph_edges: the dictionary mapping members of U to a list
		of their neighbors in V
	@return: (M, A, B), where:
		M is a dictionary mapping members of V to their matches in U
		A is the part of the maximum independent set in U
		B is the part of the MIS in V
	"""
	# make greedy matching (redundant, but faster than full search)
	matching = {}
	for u in graph_edges:
		for v in graph_edges[u]:
			if v not in matching:
				matching[v] = u
				break
	while True:
		# structure residual graph into layers
		# pred[u]: 	 neighbor in the previous layer for u in U
		# preds[v]:	 list of neighbors in the previous layer for v in V
		# unmatched: list of unmatched vertices in final layer of V

		preds = {}
		unmatched = []
		pred = dict([(u, unmatched) for u in graph_edges])
		for v in matching:
			del pred[matching[v]]
		layer = list(pred)
		
		# repeatedly extend layering structure by another pair of layers
		while layer and not unmatched:
			newLayer = {}
			for u in layer:
				for v in graph_edges[u]:
					if v not in preds:
						newLayer.setdefault(v,[]).append(u)
			layer = []
			for v in newLayer:
				preds[v] = newLayer[v]
				if v in matching:
					layer.append(matching[v])
					pred[matching[v]] = v
				else:
					unmatched.append(v)
		
		if not unmatched:
			# finish layering without finding any alternating paths
			unlayered = {}
			for u in graph_edges:
				for v in graph_edges[u]:
					if v not in preds:
						unlayered[v] = None
			return (matching,list(pred),list(unlayered))

		def recurse(v):
			"""Search backward through layers to find alternating paths.

			@return: return true if path found, false otherwise
			"""
			if v in preds:
				L = preds[v]
				del preds[v]
				for u in L:
					if u in pred:
						pu = pred[u]
						del pred[u]
						if pu is unmatched or recurse(pu):
							matching[v] = u
							return 1
			return 0

		for v in unmatched: 
			recurse(v)


def min_vertex_cover(left_edges, right_edges):
	"""Find a maximum matching or maximum independent set of a bipartite graph
	with Hopcroft-Karp algorithm.
	Then, find a minimum vertex cover by finding the complement of a
	maximum independent set.
	
	For example, given the following input
		left_edges = {1000: [2000], 1001: [2000]}
		right_edges = {2000: [1000, 1001]}
	the output or minimum vertex cover would be
		{2000: [1000, 1001]}
	with vertex 2000 being the minimum vertex cover.
	"""
	data_hk = bipartite_match(left_edges)
	left_mis = data_hk[1]
	right_mis = data_hk[2]
	mvc = left_edges.copy()
	mvc.update(right_edges)

	for v in left_mis:
		del(mvc[v])
	for v in right_mis:
		del(mvc[v])

	return mvc
