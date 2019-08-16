from tsp_methods import *
from tsp_lib import *
import numpy as np
from tkinter import *
from pca import *
import sys
import matplotlib.pyplot as plt

# filename = "test1.txt"

def read_joint_angle_file(filename, n):

	# pattern = ".*?([-]?\d+[.]?\d*, [-]?\d+[.]?\d*, [-]?\d+[.]?\d*, [-]?\d+[.]?\d*, [-]?\d+[.]?\d*, [-]?\d+[.]?\d*).*"
	num_str = "[-]?\d+[.]?\d*"
	delim_str = ", "
	pattern = ""
	pattern += ".*?("
	for i in range(n-1):
		pattern += num_str + delim_str
	pattern += num_str
	pattern += ").*"
	x = []
	repatter = re.compile(pattern)

	with open(filename, "r") as f:
		for line in f:
			result = repatter.match(line)
			s = result.group(1)
			# print(s)
			x.append(list(map(float, s.split(","))))
	return np.array(x)

def draw_result(point_table, path):
	# matplotlib
	x = [point_table[i][0] for i in path]
	x.append(point_table[path[0]][0])
	y = [point_table[i][1] for i in path]
	y.append(point_table[path[0]][1])

	plt.clf()
	plt.scatter(x, y)
	plt.plot(x, y)
	plt.show()

	# tk
	# point_table *= 50
	# point_table += 200
	# # max_x = max(map(lambda x: x[0], point_table)) + 20
	# # max_y = max(map(lambda x: x[1], point_table)) + 20
	# max_x = 800
	# max_y = 800
	# root = Tk()
	# c0 = Canvas(root, width = max_x, height = max_y, bg = "white")
	# c0.pack()
	# draw_path(c0, list(map(lambda x: point_table[x], path)))

	# root.mainloop()


def test(x):
	distance_table = make_distance_table(x)
	# print(distance_table)
	# print(distance_table.shape)
	path_greedy0 = greedy0(0, distance_table)

	path_optimize1 = optimize1(path_greedy0, distance_table)

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

	print("path_greedy0: " + str(path_distance(path_greedy0, distance_table)) + ", " + str(path_greedy0))
	print("path_optimize1: " + str(path_distance(path_optimize1, distance_table)) + ", " + str(path_optimize1))
	draw_result(point_table, path_greedy0)
	draw_result(point_table, path_optimize1)


def neiborhood1(x):
	distance_table_neiborhood_merge = make_distance_table_neiborhood_merge(x)
	path_greedy0 = greedy0(0, distance_table_neiborhood_merge)
	path = optimize1(path_greedy0, distance_table_neiborhood_merge)


	expand_path = np.empty((path.shape[0] * 2 - 1), dtype=np.int32)
	expand_path[0] = 0
	expand_path[1::2] = path[1:] * 2 - 1
	expand_path[2::2] = path[1:] * 2

	distance_table = make_distance_table(x)
	path_neiborhood_opt = neiborhood_opt(expand_path, distance_table)

	def draw():
		draw_result(x, path_greedy0)
		point_table = None
		if x.shape[1] != 2:
			# pcaは元の空間で近いサンプルは近く表示される一方で、
			# 射影空間で近いサンプルが近いかどうかは保証していない。(裏付けは？)
			# つまり、射影空間上でぱっと見もっとよさそうな経路が見えたとしても、それが元の空間でもよさそうな経路かはわからない。
			# だが、少なくともソルバで描かれた経路は短く見える利点がある
			point_table = pca_transform(x)
		else:
			point_table = x

		draw_result(point_table, expand_path)

	print("path_greedy0: " + str(path_distance(path_greedy0, distance_table_neiborhood_merge)) + ", " + str(path_greedy0))
	print("path_optimize1: " + str(path_distance(path, distance_table_neiborhood_merge)) + ", " + str(path))
	print("----------------------------")
	print("path_expand: " + str(path_distance(expand_path, distance_table)) + ", " + str(expand_path))
	print("path_neiborhood_opt: " + str(path_distance(path_neiborhood_opt, distance_table)) + ", " + str(path_neiborhood_opt))


def divide_and_conquer(x):
	distance_table = make_distance_table(x)
	points = divide_merge(x)
	path_devide_and_conquer = points_to_path(points, x)
	path_optimize1 = optimize1(path_devide_and_conquer, distance_table)

	print("path_devide_and_conquer: " + str(path_distance(path_devide_and_conquer, distance_table)) + ", " + str(path_devide_and_conquer))
	print("path_optimize1: " + str(path_distance(path_optimize1, distance_table)) + ", " + str(path_optimize1))
	
	draw_result(x, path_devide_and_conquer)
	draw_result(x, path_optimize1)


def main():
	# x = np.loadtxt("ik_test_result.txt")
	argv = sys.argv
	argc = len(argv)
	filename = argv[1]
	x = read_joint_angle_file(filename, 2)
	# test(x)
	# neiborhood1(x)
	divide_and_conquer(x)
	test(x)

if __name__ == "__main__":
	main()