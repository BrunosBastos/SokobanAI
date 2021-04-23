from consts import Tiles, TILES
import math


def breadth_first_search(start, end, mapa, wall=0b1100, walls=None):
	
	hor_tiles, ver_tiles = mapa.size
	vis = [[0] * hor_tiles for _ in range(ver_tiles)]
	queue = [(start[0], start[1], '')]
	
	while queue:
		x, y, path = queue.pop(0)

		if vis[y][x] or mapa.get_tile((x, y)) & wall or (walls and (x, y) in walls):
			# already visited or blocked
			continue 
		vis[y][x] = 1

		if (x, y) == end:
			return path

		queue.append( (x, y - 1, path + 'w') )
		queue.append( (x - 1, y, path + 'a') )
		queue.append( (x, y + 1, path + 's') )
		queue.append( (x + 1, y, path + 'd') )
	
	if end is None:
		return vis
	return None


def manhattan(p1, p2):
	"""Return manhattan distance between 2 points."""
	return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def euclidian(p1, p2):
	"""Return euclidian distance between 2 points."""
	return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

