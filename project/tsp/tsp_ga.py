# coding:utf-8

import random as rnd
import matplotlib.pyplot as plt
import math
import copy


def generator(num, pop_num):
    """ 都市を初期生成 """

    # 範囲指定
    x_range = 200
    y_range = 200

    x_coordinate = [rnd.randint(0, x_range) for _ in range(num)]
    y_coordinate = [rnd.randint(0, y_range) for _ in range(num)]

    coordinate = [[x_coordinate[i], y_coordinate[i]] for i in range(num)]

    # keyが都市の番号、valueが座標値の辞書作成。
    position_info = {}
    for i in range(num):
        position_info[i] = coordinate[i]

    # 初期の巡回順序生成
    select_num = [i for i in range(num)]
    all_route = [rnd.sample(select_num, num) for _ in range(pop_num)]

    return position_info, all_route


def evaluate(position_info, all_route, loop=0):
    """ ユークリッド距離の総和を計算し評価値とする。 """

    evaluate_value = []
    for i in range(len(all_route)):
        temp_evaluate_value = []
        x_coordinate = [position_info[all_route[i][x]][0] for x in range(len(all_route[i]))]
        y_coordinate = [position_info[all_route[i][y]][1] for y in range(len(all_route[i]))]
        for j in range(len(all_route[i])):
            if j == len(all_route[i]) - 1:
                distance = math.sqrt(
                    pow((x_coordinate[j] - x_coordinate[0]), 2) + pow((y_coordinate[j] - y_coordinate[0]), 2))
            else:
                distance = math.sqrt(
                    pow((x_coordinate[j] - x_coordinate[j + 1]), 2) + pow((y_coordinate[j] - y_coordinate[j + 1]), 2))
            temp_evaluate_value.append(distance)
        evaluate_value.append(sum(temp_evaluate_value))

    # 一番優秀な個体をmatplotで描画
    excellent_evaluate_value = min(evaluate_value)
    draw_pop_index = evaluate_value.index(excellent_evaluate_value)

    show_route(position_info, all_route[draw_pop_index],int(excellent_evaluate_value), loop=loop + 1)

    return evaluate_value


def selection(all_route, evaluate_value, tournament_select_num, tournament_size, elite_select_num, ascending=False):
    """ トーナメント選択とエリート保存を行う"""

    select_pop = []
    elite_pop = []
    # トーナメント選択
    while True:
        select = rnd.sample(evaluate_value, tournament_size)
        select.sort(reverse=ascending)
        for i in range(tournament_select_num):
            value = select[i]
            index = evaluate_value.index(value)
            select_pop.append(all_route[index])

        # 個体数の半数個選択するまで実行
        if len(select_pop) >= len(all_route) / 2:
            break

    # エリート保存
    sort_evaluate_value = copy.deepcopy(evaluate_value)
    sort_evaluate_value.sort(reverse=ascending)
    for i in range(elite_select_num):
        value = sort_evaluate_value[i]
        index = evaluate_value.index(value)
        elite_pop.append(all_route[index])

    return select_pop, elite_pop


def crossover(select_pop, crossover_prob):
    ''' 確率的に順序交叉を実行する '''

    cross_pop = rnd.sample(select_pop, 2)
    pop_1 = cross_pop[0]
    pop_2 = cross_pop[1]

    check_prob = rnd.randint(0, 100)
    if check_prob <= crossover_prob:

        # 順序交叉
        new_pop_1 = []
        cut_index = rnd.randint(1, len(pop_1) - 2)
        new_pop_1.extend(pop_1[:cut_index])
        for i in range(len(pop_1)):
            if pop_2[i] not in new_pop_1:
                new_pop_1.append(pop_2[i])

        new_pop_2 = []
        new_pop_2.extend(pop_1[cut_index:])
        for i in range(len(pop_1)):
            if pop_2[i] not in new_pop_2:
                new_pop_2.append(pop_2[i])

        return new_pop_1, new_pop_2

    else:
        return pop_1, pop_2


def mutation(pop, mutation_prob):
    """ 循環経路の順番をランダムで入れ替える """

    check_prob = rnd.randint(0, 100)

    if check_prob <= mutation_prob:
        select_num = [i for i in range(num)]
        select_index = rnd.sample(select_num, 2)

        a = pop[select_index[0]]
        b = pop[select_index[1]]
        pop[select_index[1]] = a
        pop[select_index[0]] = b

    return pop


def show_route(position_info, route, excellent_evaluate_value,  loop=0):
    """ matplotlibで循環経路を描画 """

    x_coordinate = [position_info[route[i]][0] for i in range(len(route))]
    y_coordinate = [position_info[route[i]][1] for i in range(len(route))]
    x_coordinate.append(position_info[route[0]][0])
    y_coordinate.append(position_info[route[0]][1])

    plt.scatter(x_coordinate, y_coordinate)
    plt.plot(x_coordinate, y_coordinate, label=excellent_evaluate_value)
    plt.title("Generation: {}".format(loop))
    plt.legend()

    plt.savefig("img/tsp{}".format(loop))  # pngとして保存。カレントディレクトリにimgフォルダを置く必要あり。
    plt.show()


# 初期生成時のパラメータ
num = 30  # 都市の数
pop_num = 200  # 個体数
generation_num = 200  # 世代数

# 選択のパラメータ
tournament_size = 10
tournament_select_num = 2
elite_select_num = 1

# 交叉の確率
crossover_prob = 50

# 突然変異の確率
mutation_prob = 3


# 初期生成
position_info, all_route = generator(num, pop_num)

# 評価
evaluate_value = evaluate(position_info, all_route)

for loop in range(generation_num):
    # 選択
    select_pop, elite_pop = selection(
        all_route, evaluate_value, tournament_select_num, tournament_size, elite_select_num, ascending=False)

    # 選択した個体の中から2個体選択し交叉や突然変異を適用する。
    next_pop = []
    while True:
        # 交叉
        pop_1, pop_2 = crossover(select_pop, crossover_prob)
        # 突然変異
        pop_1 = mutation(pop_1, mutation_prob)
        pop_2 = mutation(pop_2, mutation_prob)

        next_pop.append(pop_1)
        next_pop.append(pop_2)

        if len(next_pop) >= pop_num - elite_select_num:
            break

    # エリート主義。優良個体を次世代へ継承。
    next_pop.extend(elite_pop)

    # 評価
    evaluate_value = evaluate(position_info, next_pop, loop=loop + 1)

    # 更新
    all_route = next_pop