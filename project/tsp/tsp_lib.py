#coding:utf-8
import numpy as np
import math

__DIST_DEFINITION__ = "EUCREDIAN"
# __DIST_DEFINITION__ = "MAX"
INF = 10000000000

# 座標配列から距離テーブルを作成
def make_distance_table(x):
	num, dim = x.shape
	# table = np.ones((num, num))
	table = np.zeros((num, num))
	for i in range(num):
		for j in range(num):
			table[i][j] = distance(x[i], x[j])
	return table

def argmax(l):
	ret = 0
	tmp = -INF
	for i in range(len(l)):
		if(tmp < l[i]):
			tmp = l[i]
			ret = i
	return ret

# 座標から距離を計算
def distance(a, b):
	assert(len(a) == len(b))
	ret = 0

	if __DIST_DEFINITION__ == "EUCREDIAN":
		# それぞれの要素は同じ重み
		for i in range(len(a)):
			ret += (a[i] - b[i]) ** 2
		ret = math.sqrt(ret)
	elif __DIST_DEFINITION__ == "MAX":
		for i in range(len(a)):
			ret = max(ret, abs(a[i] - b[i]))
	return ret

def path_distance(path, distance_table):
	ret = 0
	size = len(path)
	for i in range(size - 1):
		ret += distance_table[path[i]][path[i+1]]
	ret += distance_table[path[size-1]][path[0]]
	return ret

# 隣接点の距離を0にする(片方を通ったら、必ずもう片方を通る必要がある制約を最適化の中で解く)
def neiborhood_zero_distance(distance_table, neiborhoods):
	for neiborhood in neiborhoods:
		a, b = neiborhood
		distance_table[a][b] = 0
		distance_table[b][a] = 0

# 隣接点を平均化して一つの点とみなす
def make_distance_table_neiborhood_merge(x):
	num, dim = x.shape
	assert(num % 2 == 1)
	# 0番目の点は隣接点なし
	num = int(num / 2) + 1
	# table = np.ones((num, num))
	table = np.zeros((num, num))
	for i in range(1, num):
		for j in range(num):
			table[i][j] = distance((x[i] + x[i+1]) / 2, (x[j] + x[j+1]) / 2)
	return table

# 経路の表示
def draw_path(c0, path):
	# pathはx, y座標のセット
    x0, y0 = path[0]
    for i in range(1, len(path)):
        x1, y1 = path[i]
        c0.create_line(x0, y0, x1, y1)
        x0, y0 = x1, y1
    c0.create_line(x0, y0, path[0][0], path[0][1])
    for x, y in path:
        c0.create_oval(x - 4, y - 4, x + 4, y + 4, fill = "green")

def points_to_path(points, point_table):
	size = len(points)
	ret = np.zeros(size, dtype=np.int32)
	for i in range(size):
		for j in range(len(point_table)):
			# if(points[i] == point_table[j]):
			if(np.equal(points[i], point_table[j]).all()):
				ret[i] = j
	return ret