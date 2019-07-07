#coding:utf-8
import numpy as np
import math
# 座標配列から距離テーブルを作成
def make_distance_table(x):
	num, dim = x.shape
	# table = np.ones((num, num))
	table = np.zeros((num, num))
	for i in range(num):
		for j in range(num):
			table[i][j] = distance(x[i], x[j])
	return table

# 座標から距離を計算
def distance(a, b):
	assert(len(a) == len(b))
	ret = 0
	# それぞれの要素は同じ重み
	for i in range(len(a)):
		ret += (a[i] - b[i]) ** 2
	ret = math.sqrt(ret)
	return ret


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

