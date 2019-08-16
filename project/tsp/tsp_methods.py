#coding:utf-8
# tsp_methods.py : 巡回セールスマン問題
#
#           Copyright (C) 2012 Makoto Hiroi
# greedy0: 貪欲法
# greedy1: クラスカルのアルゴリズムの応用
# devide_merge: 分割統治法
from tsp_lib import *
from functools import cmp_to_key
import numpy as np
import copy


########################################
# 単純な欲張り法
########################################
def greedy0(start, distance_table):
    size = distance_table.shape[0]
    path = np.array([i for i in range(size)])
    path[0], path[start] = path[start], path[0]
    for i in range(size - 1):
        min_len = INF
        min_pos = 0
        for j in range(i + 1, size):
            len = distance_table[path[i]][path[j]]
            if len < min_len:
                min_len = len
                min_pos = j
        path[i + 1], path[min_pos] = path[min_pos], path[i + 1]
    return path


#######################################
# クラスカルのアルゴリズムの応用
#######################################

# # 辺の定義
# class Edge:
#     def __init__(self, p1, p2, weight):
#         self.p1 = p1
#         self.p2 = p2
#         self.weight = weight

#     def __cmp__(x, y):
#         return x.weight - y.weight

# # 辺のデータを作成
# def make_edge(size, distance_table):
#     edges = PQueue()
#     for i in range(0, size - 1):
#         for j in range(i + 1, size):
#             e = Edge(i, j, distance_table[i][j])
#             edges.push(e)
#     return edges

# # 巡路の生成
# def greedy1(size, edges):
#     edge_count = [0] * size
#     u = UnionFind(size)
#     i = 0
#     select_edge = []
#     while i < size:
#         e = edges.pop()
#         if edge_count[e.p1] < 2 and edge_count[e.p2] < 2 and (u.find(e.p1) != u.find(e.p2) or i == size - 1):
#             u.union(e.p1, e.p2)
#             edge_count[e.p1] += 1
#             edge_count[e.p2] += 1
#             select_edge.append(e)
#             i += 1
#     return select_edge

###########################################
# 分割統治法
###########################################


# 分割する次元を決定する
def divide_direction(buff):
    assert(len(buff) > 0)
    subd_list = []
    for i in range(len(buff[0])):
        subd_list.append(max(map(lambda x: x[i], buff)) - min(map(lambda x: x[i], buff)))
    # 分割される次元を表すintを返す
    return argmax(subd_list)

# 分割する
def divide(buff, comp):
    sorted(buff, key = cmp_to_key(comp))
    n = len(buff) // 2
    buff1 = buff[:n+1]
    buff2 = buff[n:]
    return buff[n], buff1, buff2

# 差分を計算する
def differ(p, c, q):
    return distance(p, c) + distance(c, q) - distance(p, q)

# 共有点を探す
def search(x, buff):
    for i in range(len(buff)):
        # if buff[i] == x:
        if np.array_equal(buff[i], x):
            if i == 0: return len(buff) - 1, i, i + 1
            if i == len(buff) - 1: return i - 1, i, 0
            return i - 1, i, i + 1

# 挿入するための新しい経路を作る
def make_new_path(buff, c, succ):
    path = []
    i = c + succ
    while True:
        if i < 0: i = len(buff) - 1
        elif i >= len(buff): i = 0
        if i == c: break
        path.append(buff[i])
        i += succ
    return path

# 併合する
# buff1 = [a, b, c, d, e]
# buff2 = [f, g, c, h, i]
# (1) b - g => [a, b, g, f, i, h, c, d, e]
# (2) d - h => [a, b, c, g, f, i, h, d, e]
# (3) b - h => [a, b, h, i, f. g. c, d, e]
# (4) d - g => [a, b. c. h, i, f, g, d, e]
def merge(buff1, buff2, p):
    # 共有ポイントを探す
    p1, i1, n1 = search(p, buff1)
    p2, i2, n2 = search(p, buff2)
    # 差分を計算
    d1 = differ(buff1[p1], p, buff2[p2])
    d2 = differ(buff1[n1], p, buff2[n2])
    d3 = differ(buff1[p1], p, buff2[n2])
    d4 = differ(buff1[n1], p, buff2[p2])
    # 差分が一番大きいものを選択
    d = max(d1, d2, d3, d4)
    if d1 == d:
        # (1)
        # buff1[i1:i1] = make_new_path(buff2, i2, -1)
        buff1 = np.insert(buff1, i1, make_new_path(buff2, i2, -1), axis=0)
    elif d2 == d:
        # (2)
        # buff1[n1:n1] = make_new_path(buff2, i2, -1)
        buff1 = np.insert(buff1, n1, make_new_path(buff2, i2, -1), axis=0)
    elif d3 == d:
        # (3)
        # buff1[i1:i1] = make_new_path(buff2, i2, 1)
        buff1 = np.insert(buff1, i1, make_new_path(buff2, i2, 1), axis=0)

    else:
        # (4)
        # buff1[n1:n1] = make_new_path(buff2, i2, 1)
        buff1 = np.insert(buff1, n1, make_new_path(buff2, i2, 1), axis=0)
    return buff1

# 分割統治法による解法
def divide_merge(buff):
    if len(buff) <= 3:
        # print buff
        return buff
    else:
        d = divide_direction(buff)
        p, b1, b2 = divide(buff, lambda x, y: x[d] - y[d])
        b3 = divide_merge(b1)
        b4 = divide_merge(b2)
        return merge(b3, b4, p)

###########################################
# 局所探索法
###########################################

# 2-opt 法
# O(N^2*M) M:変換が行われなくなるまでのループ回数
def opt_2(path, distance_table):
    _path = copy.deepcopy(path)
    size = distance_table.shape[0]
    total = 0
    while True:
        # 変換が行われた回数を格納。変換が行わていなければ、ループ終了。
        count = 0 
        for i in range(size - 2):
            i1 = i + 1
            for j in range(i + 2, size):
                if j == size - 1:
                    j1 = 0
                else:
                    j1 = j + 1
                if i != 0 or j1 != 0:
                    l1 = distance_table[_path[i]][_path[i1]]
                    l2 = distance_table[_path[j]][_path[j1]]
                    l3 = distance_table[_path[i]][_path[j]]
                    l4 = distance_table[_path[i1]][_path[j1]]
                    if l1 + l2 > l3 + l4:
                        # つなぎかえる
                        new_path = _path[i1:j+1]
                        _path[i1:j+1] = new_path[::-1]
                        count += 1
        total += count
        if count == 0: break
    # totalはoptimize用。optimizeとはreturnが異なるので、こちらをdrawするのはお勧めしない。
    return _path, total

# 2-optの隣接点だけバージョン。neiborhood1で使う。
def neiborhood_opt(path, distance_table):
    _path = copy.deepcopy(path)
    size = distance_table.shape[0]
    for i in range(1, size - 1, 2):
        i1 = i + 1
        i2 = i + 2
        if i2 > size - 1:
            i2 = 0
        l1 = distance_table[_path[i-1]][_path[i]]
        l2 = distance_table[_path[i1]][_path[i2]]
        l3 = distance_table[_path[i-1]][_path[i1]]
        l4 = distance_table[_path[i]][_path[i2]]
        if l1 + l2 > l3 + l4:
            # iとi1を反転させる
            new_path = _path[i:i+2]
            _path[i:i+2] = new_path[::-1]
    # totalはoptimize用。optimizeとはreturnが異なるので、こちらをdrawするのはお勧めしない。
    return _path

# or-opt 法 (簡略版)
# O(N^2*M) M:変換が行われなくなるまでのループ回数
def or_opt(path, distance_table):
    _path = copy.deepcopy(path)
    size = distance_table.shape[0]
    total = 0
    while True:
        # 変換が行われた回数を格納。変換が行わていなければ、ループ終了。
        count = 0 
        for i in range(size):
            # i 番目の都市を (j) - (j1) の経路に挿入する
            i0 = i - 1
            i1 = i + 1
            if i0 < 0: i0 = size - 1
            if i1 == size: i1 = 0
            for j in range(size):
                j1 = j + 1
                if j1 == size: j1 = 0
                if j != i and j1 != i:
                    l1 = distance_table[_path[i0]][_path[i]]  # i0 - i - i1
                    l2 = distance_table[_path[i]][_path[i1]]
                    l3 = distance_table[_path[j]][_path[j1]]  # j - j1
                    l4 = distance_table[_path[i0]][_path[i1]] # i0 - i1
                    l5 = distance_table[_path[j]][_path[i]]   # j - i - j1
                    l6 = distance_table[_path[i]][_path[j1]] 
                    if l1 + l2 + l3 > l4 + l5 + l6:
                        # つなぎかえる
                        p = _path[i]
                        # _path[i:i + 1] = []
                        _path = np.delete(_path, i)
                        if i < j:
                            # _path[j:j] = [p]
                            _path = np.insert(_path, j, p, axis=0)
                        else:
                            # _path[j1:j1] = [p]
                            _path = np.insert(_path, j1, p, axis=0)

                        count += 1
        total += count
        if count == 0: break
    # totalはoptimize用。optimizeとはreturnが異なるので、こちらをdrawするのはお勧めしない。
    return _path, total


# 組み合わせ
def optimize1(path, distance_table):
    while True:
        path, _ = opt_2(path, distance_table)
        path, flag = or_opt(path, distance_table)
        if flag == 0: return path

def optimize2(path, distance_table):
    while True:
        path, _ = or_opt(path, distance_table)
        path, flag = opt_2(path, distance_table)
        if flag == 0: return path

