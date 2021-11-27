# -*- coding = utf-8 -*-
# @Time : 2021/11/5 14:16
# @Author : CDC
# @File : Q2.py
# @Software: PyCharm
import numpy as np
import copy
import skfuzzy as fuzz
import tqdm


def calculate_relation(arr_input):
    r = np.zeros([16, 16])
    width = len(arr_input[0])
    for i in range(len(arr_input)):
        for j in range(len(arr_input)):
            nominator = 0
            deno1 = 0
            deno2 = 0
            for m in range(width):
                nominator += arr_input[i][m] * arr_input[j][m]
                deno1 += arr_input[i][m] ** 2
                deno2 += arr_input[j][m] ** 2
            nominator = abs(nominator)
            denominator = (deno1 * deno2) ** 0.5
            r[i][j] = nominator / denominator

    return r


def iterate():
    R1 = calculate_relation(arr.T)
    r = np.zeros([16, 16])
    iterate_time = 0
    s = copy.deepcopy(R1)

    while 1:
        s_previous = copy.deepcopy(s)
        for m in range(len(R1)):
            a = R1.T[m]

            for i in range(len(R1)):  # row
                for j in range(len(R1[0])):
                    r[i][j] = min(s[i][j], a[i])

            for n in range(len(r.T)):
                s[m][n] = max(r.T[n])

        if (s == s_previous).all():
            return s
        else:
            iterate_time += 1


def iterate2():
    R1 = calculate_relation(arr.T)
    iterate_time = 0
    s = copy.deepcopy(R1)

    while 1:
        s_previous = s
        s = fuzz.maxmin_composition(R1, s)

        if (s == s_previous).all():
            return s
        else:
            iterate_time += 1


def cut(alfa, R):
    return (R > alfa).astype(int)
    # for i in range(len(R[0])):
    #     for j in range(len(R[0])):
    #         if R[i][j] >= alfa:
    #             R[i][j] = 1
    #         else:
    #             R[i][j] = 0
    # return R


def classify():
    for alfa in tqdm.tqdm(np.arange(0.8, 0.9, 0.0000001)):
        x = cut(alfa, R)
        class_num = len(np.unique(x, axis=0))
        if class_num == 3:
            print(alfa)
            print(x)
            break
    for alfa in tqdm.tqdm(np.arange(0.9, 0.8, -0.0000001)):
        x = cut(alfa, R)
        class_num = len(np.unique(x, axis=0))
        if class_num == 3:
            print(alfa)
            print(x)
            break


arr = np.array([[0.1, 0.0, 0.2, 0.8, 0.3, 0.0, 0.5, 0.6, 0.0, 0.1, 0.3, 0.1, 0.2, 0.2, 0.1, 0.2],
                [0.7, 0.5, 0.2, 0.1, 0.0, 0.4, 0.0, 0.3, 0.5, 0.6, 0.2, 0.5, 0.0, 0.6, 0.7, 0.4],
                [0.2, 0.5, 0.2, 0.0, 0.4, 0.0, 0.4, 0.0, 0.1, 0.0, 0.1, 0.4, 0.2, 0.1, 0.1, 0.2],
                [0.0, 0.0, 0.4, 0.1, 0.3, 0.6, 0.1, 0.1, 0.4, 0.3, 0.4, 0.0, 0.6, 0.1, 0.1, 0.2]])
arr2 = np.array([[1, 0.8, 0.4, 0.5, 0.2],
                 [0.8, 1, 0.4, 0.5, 0.9],
                 [0.4, 0.4, 1, 0.4, 0.4],
                 [0.5, 0.5, 0.4, 1, 0.5],
                 [0.8, 0.9, 0.4, 0.5, 1]])

# print(ak)
R = iterate()
#classify()
x = cut(0.9,R)
print(len(np.unique(x, axis=0)))
# classify()
# Answer 0.842665-0.857142
