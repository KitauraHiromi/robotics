from tsp_methods import *
from tsp_lib import *
import numpy as np
from tkinter import *
from pca import *

def draw_result(point_table, path):
	point_table *= 100
	point_table += 400
	# max_x = max(map(lambda x: x[0], point_table)) + 20
	# max_y = max(map(lambda x: x[1], point_table)) + 20
	max_x = 800
	max_y = 800
	root = Tk()
	c0 = Canvas(root, width = max_x, height = max_y, bg = "white")
	c0.pack()
	draw_path(c0, list(map(lambda x: point_table[x], path)))

	root.mainloop()

def main():
	x = np.loadtxt("ik_test_result.txt")
	distance_table = make_distance_table(x)
	print(distance_table)
	print(distance_table.shape)
	path_greedy0 = greedy0(0, distance_table)
	path_optimize1 = optimize1(path_greedy0, distance_table)
	path_optimize2 = optimize2(path_greedy0, distance_table)

	# matplotlibで書き直し
	# draw_result(x, path_greedy0)
	point_table = None
	if x.shape[1] != 2:
		# pcaは元の空間で近いサンプルは近く表示される一方で、
		# 射影空間で近いサンプルが近いかどうかは保証していない。(裏付けは？)
		# つまり、射影空間上でぱっと見もっとよさそうな経路が見えたとしても、それが元の空間でもよさそうな経路かはわからない。
		# だが、少なくともソルバで描かれた経路は短く見える利点がある
		point_table = pca_transform(x)
	else:
		point_table = x
	draw_result(point_table, path_optimize1)
	draw_result(point_table, path_optimize2)

if __name__ == "__main__":
	main()