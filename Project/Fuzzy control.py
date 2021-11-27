# -*- coding = utf-8 -*-
# @Time : 2021/11/4 21:08
# @Author : CDC
# @File : Fuzzy system.py
# @Software: PyCharm
import math
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting
import tqdm

v = 0.5
T = 0.1
L = 2.5


def ducktruck():
    x_yPosition = np.arange(-100, 100.01, 0.01)
    x_angleTheta = np.arange(-250, 250.01, 0.01)
    x_steerAngle = np.arange(-30, 30.01, 0.01)

    yPosition = ctrl.Antecedent(x_yPosition, 'y')
    angleTheta = ctrl.Antecedent(x_angleTheta, 'Theta')
    steerAngle = ctrl.Consequent(x_steerAngle, 'u')

    yPosition['BE'] = fuzz.trapmf(x_yPosition, [-100, -100, -40, -12.5])
    yPosition['BC'] = fuzz.trimf(x_yPosition, [-20, -10, 0])
    yPosition['CE'] = fuzz.trimf(x_yPosition, [-5, 0, 5])
    yPosition['AC'] = fuzz.trimf(x_yPosition, [0, 10, 20])
    yPosition['AB'] = fuzz.trapmf(x_yPosition, [12.5, 40, 100, 100])

    angleTheta['BO'] = fuzz.trapmf(x_angleTheta, [-181, -181, -120, -80])
    angleTheta['BR'] = fuzz.trimf(x_angleTheta, [-100, -65, -30])
    angleTheta['BH'] = fuzz.trimf(x_angleTheta, [-50, -25, 0])
    angleTheta['HZ'] = fuzz.trimf(x_angleTheta, [-10, 0, 10])
    angleTheta['AH'] = fuzz.trimf(x_angleTheta, [0, 25, 50])
    angleTheta['AR'] = fuzz.trimf(x_angleTheta, [30, 65, 100])
    angleTheta['AO'] = fuzz.trapmf(x_angleTheta, [80, 120, 181, 181])

    steerAngle['NB'] = fuzz.trimf(x_steerAngle, [-30, -30, -17])
    steerAngle['NM'] = fuzz.trimf(x_steerAngle, [-25, -15, -5])
    steerAngle['NS'] = fuzz.trimf(x_steerAngle, [-12.5, -6.25, 0])
    steerAngle['ZE'] = fuzz.trimf(x_steerAngle, [-5, 0, 5])
    steerAngle['PS'] = fuzz.trimf(x_steerAngle, [0, 6.25, 12.5])
    steerAngle['PM'] = fuzz.trimf(x_steerAngle, [5, 15, 25])
    steerAngle['PB'] = fuzz.trimf(x_steerAngle, [17, 30, 30])

    rule0 = ctrl.Rule(antecedent=((yPosition['BE'] & angleTheta['BO']) |
                                  (yPosition['BE'] & angleTheta['BR']) |
                                  (yPosition['BC'] & angleTheta['BO']) |
                                  (yPosition['BC'] & angleTheta['BR']) |
                                  (yPosition['BE'] & angleTheta['BH'])),
                      consequent=steerAngle['PB'], label='rule PB')

    rule1 = ctrl.Rule(antecedent=((yPosition['CE'] & angleTheta['BO']) |
                                  (yPosition['CE'] & angleTheta['BR']) |
                                  (yPosition['BC'] & angleTheta['HZ']) |
                                  (yPosition['BC'] & angleTheta['BH']) |
                                  (yPosition['BE'] & angleTheta['AH']) |
                                  (yPosition['BE'] & angleTheta['HZ']) |
                                  (yPosition['AC'] & angleTheta['BO'])),
                      consequent=steerAngle['PM'], label='rule PM')

    rule2 = ctrl.Rule(antecedent=((yPosition['BE'] & angleTheta['AR']) |
                                  (yPosition['BC'] & angleTheta['AH']) |
                                  (yPosition['CE'] & angleTheta['BH']) |
                                  (yPosition['AC'] & angleTheta['BR']) |
                                  (yPosition['AB'] & angleTheta['BO'])),
                      consequent=steerAngle['PS'], label='rule PS')

    rule3 = ctrl.Rule(antecedent=(yPosition['CE'] & angleTheta['HZ']),
                      consequent=steerAngle['ZE'], label='rule ZE')

    rule4 = ctrl.Rule(antecedent=((yPosition['AB'] & angleTheta['BR']) |
                                  (yPosition['AC'] & angleTheta['BH']) |
                                  (yPosition['CE'] & angleTheta['AH']) |
                                  (yPosition['BC'] & angleTheta['AR']) |
                                  (yPosition['BE'] & angleTheta['AO'])),
                      consequent=steerAngle['NS'], label='rule NS')

    rule5 = ctrl.Rule(antecedent=((yPosition['AB'] & angleTheta['BH']) |
                                  (yPosition['AB'] & angleTheta['HZ']) |
                                  (yPosition['AC'] & angleTheta['HZ']) |
                                  (yPosition['AC'] & angleTheta['AH']) |
                                  (yPosition['CE'] & angleTheta['AR']) |
                                  (yPosition['CE'] & angleTheta['AO']) |
                                  (yPosition['BC'] & angleTheta['AO'])),
                      consequent=steerAngle['NM'], label='rule NM')

    rule6 = ctrl.Rule(antecedent=((yPosition['AB'] & angleTheta['AH']) |
                                  (yPosition['AB'] & angleTheta['AR']) |
                                  (yPosition['AB'] & angleTheta['AO']) |
                                  (yPosition['AC'] & angleTheta['AR']) |
                                  (yPosition['AC'] & angleTheta['AO'])),
                      consequent=steerAngle['NB'], label='rule BN')

    system = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4, rule5, rule6])
    steerAnglePredict = ctrl.ControlSystemSimulation(system)
    return steerAnglePredict

def fuzzy3Dplot():
    steerAnglePredict = ducktruck()
    upsampled = np.linspace(-100, 100, 101)

    x, y = np.meshgrid(upsampled, upsampled)
    z = np.zeros_like(x)

    # Loop through the system 21*21 times to collect the control surface
    for i in tqdm.tqdm(range(101)):
        for j in range(101):
            steerAnglePredict.input['y'] = x[i, j]
            steerAnglePredict.input['Theta'] = y[i, j]
            steerAnglePredict.compute()
            z[i, j] = steerAnglePredict.output['u']



    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                           linewidth=0.4, antialiased=True)


    ax.view_init(30, 50)
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\BigMac.png", dpi = 500)

    plt.show()

def fuzzyCalculate(y,theta,x):
    time = 0
    steerAnglePredict = ducktruck()
    while 1:
        if 0.01 > y > -0.01 and 1 > theta > -1:
            print(time)
            print(y_list)
            print(x)
            return x
        time += T
        steerAnglePredict.input['y'] = y
        steerAnglePredict.input['Theta'] = theta
        steerAnglePredict.compute()
        u = steerAnglePredict.output['u']
        theta += math.degrees(v * T * math.tan(math.radians(u)) / L)
        x += v * T * math.cos(math.radians(theta))
        y += v * T * math.sin(math.radians(theta))
        x_list.append(x)
        y_list.append(y)
        time_list.append(time)
        theta_list.append(theta)
        u_list.append(u)


def classictruck(y, theta, x, m1):
    time = 0
    y_ini = y
    while 1:
        if 0.01 > y > -0.01 and 1 > theta > -1:
            return x, time
        time += T

        u = -30 * (m1 * (y / abs(y_ini)) + (1 - m1) * (theta / 180))
        theta += math.degrees(v * T * math.tan(math.radians(u)) / L)
        x += v * T * math.cos(math.radians(theta))
        y += v * T * math.sin(math.radians(theta))
        x_list_cla.append(x)
        y_list_cla.append(y)
        time_list_cla.append(time)
        theta_list_cla.append(theta)
        u_list_cla.append(u)


def condition1():
    ducktruck(40, 0, 20)

    fig = plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'r')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.axis([0, 191, -20, 15])

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'g')
    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([0, 191, -20, 50])

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b')
    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.axis([0, 191, -100, 30])

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'orange')
    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([20, 88, -10, 50])
    axins = inset_axes(ax4, width="40%", height="30%", loc='upper right',
                       bbox_to_anchor=(0, 0, 1, 1),
                       bbox_transform=ax4.transAxes)
    axins.plot(x_list, y_list, color='orange')
    axins.grid(linestyle='-.')
    zone_left = 1400
    zone_right = 1800

    x_ratio = 0.1  # x轴显示范围的扩展比例
    y_ratio = 0.1  # y轴显示范围的扩展比例

    # X轴的显示范围
    xlim0 = x_list[zone_left] - (x_list[zone_right] - x_list[zone_left]) * x_ratio
    xlim1 = x_list[zone_right] + (x_list[zone_right] - x_list[zone_left]) * x_ratio

    # Y轴的显示范围
    y = np.hstack(y_list[zone_left:zone_right])
    ylim0 = np.min(y) - (np.max(y) - np.min(y)) * y_ratio
    ylim1 = np.max(y) + (np.max(y) - np.min(y)) * y_ratio

    # 调整子坐标系的显示范围
    axins.set_xlim(xlim0, xlim1)
    axins.set_ylim(ylim0, ylim1)
    mark_inset(ax4, axins, loc1=3, loc2=1, fc="none", ec='k', lw=1)

    plt.suptitle('Condition 1', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F1.png")
    plt.show()


def condition2():
    ducktruck(-30, 90, 10)

    fig = plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'r')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.axis([0, 158, -15, 5])

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'g')
    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([0, 158, -30, 10])

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b')
    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.axis([0, 158, -15, 100])

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'orange')
    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([9, 65, -30, 10])
    axins = inset_axes(ax4, width="40%", height="30%", loc='lower right',
                       bbox_to_anchor=(0, 0, 1, 1),
                       bbox_transform=ax4.transAxes)
    axins.plot(x_list, y_list, color='orange')
    axins.grid(linestyle='-.')
    zone_left = 1200
    zone_right = 1500

    x_ratio = 0.1  # x轴显示范围的扩展比例
    y_ratio = 0.1  # y轴显示范围的扩展比例

    # X轴的显示范围
    xlim0 = x_list[zone_left] - (x_list[zone_right] - x_list[zone_left]) * x_ratio
    xlim1 = x_list[zone_right] + (x_list[zone_right] - x_list[zone_left]) * x_ratio

    # Y轴的显示范围
    y = np.hstack(y_list[zone_left:zone_right])
    ylim0 = np.min(y) - (np.max(y) - np.min(y)) * y_ratio
    ylim1 = np.max(y) + (np.max(y) - np.min(y)) * y_ratio

    # 调整子坐标系的显示范围
    axins.set_xlim(xlim0, xlim1)
    axins.set_ylim(ylim0, ylim1)
    mark_inset(ax4, axins, loc1=3, loc2=1, fc="none", ec='k', lw=1)

    plt.suptitle('Condition 2', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F2.png")
    plt.show()


def condition3():
    ducktruck(30, -140, 40)

    fig = plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'r')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.axis([0, 165, -5, 15])

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'g')
    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([0, 165, -5, 40])

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b')
    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.axis([0, 158, -150, 50])

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'orange')
    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([30, 87, -10, 30])
    axins = inset_axes(ax4, width="40%", height="30%", loc='upper right',
                       bbox_to_anchor=(0, 0, 1, 1),
                       bbox_transform=ax4.transAxes)
    axins.plot(x_list, y_list, color='orange')
    axins.grid(linestyle='-.')
    zone_left = 1200
    zone_right = 1600

    x_ratio = 0.1  # x轴显示范围的扩展比例
    y_ratio = 0.1  # y轴显示范围的扩展比例

    # X轴的显示范围
    xlim0 = x_list[zone_left] - (x_list[zone_right] - x_list[zone_left]) * x_ratio
    xlim1 = x_list[zone_right] + (x_list[zone_right] - x_list[zone_left]) * x_ratio

    # Y轴的显示范围
    y = np.hstack(y_list[zone_left:zone_right])
    ylim0 = np.min(y) - (np.max(y) - np.min(y)) * y_ratio
    ylim1 = np.max(y) + (np.max(y) - np.min(y)) * y_ratio

    # 调整子坐标系的显示范围
    axins.set_xlim(xlim0, xlim1)
    axins.set_ylim(ylim0, ylim1)
    mark_inset(ax4, axins, loc1=3, loc2=1, fc="none", ec='k', lw=1)

    plt.suptitle('Condition 3', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F3.png")
    plt.show()


def condition4():
    ducktruck(10, -10, 50)

    fig = plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'r')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.axis([0, 125, -10, 15])

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'g')
    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([0, 125, -2, 10])

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b')
    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.axis([0, 125, -50, 20])

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'orange')
    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([50, 110, -5, 10])
    axins = inset_axes(ax4, width="40%", height="30%", loc='upper right',
                       bbox_to_anchor=(0, 0, 1, 1),
                       bbox_transform=ax4.transAxes)
    axins.plot(x_list, y_list, color='orange')
    axins.grid(linestyle='-.')
    zone_left = 800
    zone_right = 1200

    x_ratio = 0.1  # x轴显示范围的扩展比例
    y_ratio = 0.1  # y轴显示范围的扩展比例

    # X轴的显示范围
    xlim0 = x_list[zone_left] - (x_list[zone_right] - x_list[zone_left]) * x_ratio
    xlim1 = x_list[zone_right] + (x_list[zone_right] - x_list[zone_left]) * x_ratio

    # Y轴的显示范围
    y = np.hstack(y_list[zone_left:zone_right])
    ylim0 = np.min(y) - (np.max(y) - np.min(y)) * y_ratio
    ylim1 = np.max(y) + (np.max(y) - np.min(y)) * y_ratio

    # 调整子坐标系的显示范围
    axins.set_xlim(xlim0, xlim1)
    axins.set_ylim(ylim0, ylim1)
    mark_inset(ax4, axins, loc1=3, loc2=1, fc="none", ec='k', lw=1)

    plt.suptitle('Condition 4', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F4.png")
    plt.show()


def classic_plot():
    plt.plot(x_list, y_list, 'orange')
    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.show()


def findm1():
    ptemp1 = list()
    ptimelist1 = list()
    ptimelist2 = list()
    ptimelist3 = list()
    ptimelist4 = list()
    minitime = 10000
    for i in np.arange(0.01, 0.59, 0.01):
        print(i)
        ptemp1.append(i)
        x1, time1 = classictruck(40, 0, 20, i)
        x2, time2 = classictruck(-30, 90, 10, i)
        x3, time3 = classictruck(30, -140, 40, i)
        x4, time4 = classictruck(10, -10, 50, i)
        ptimelist1.append(time1)
        ptimelist2.append(time2)
        ptimelist3.append(time3)
        ptimelist4.append(time4)

        if time1 < minitime:
            minitime = time1
            minii = i
    plt.plot(ptemp1, ptimelist1, 'r', label = 'Condition1')
    plt.plot(ptemp1, ptimelist2, 'g', label = 'Condition2')
    plt.plot(ptemp1, ptimelist3, 'b', label = 'Condition3')
    plt.plot(ptemp1, ptimelist4, 'orange', label = 'Condition4')
    plt.title("Time-p Relation")
    plt.grid(linestyle='-.')
    plt.xlabel("p", fontsize=12)
    plt.ylabel("time", fontsize=12)
    plt.axis([0, 0.6, 0, 10000])
    plt.legend()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\Relation.png")

    plt.show()

def classiccondition1():
    ducktruck(40, 0, 20)
    classictruck(40, 0, 20, 0.19)

    plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, u_list_cla, 'r', label = 'non-fuzzy')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.axis([0, 300, -20, 15])
    plt.legend()

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, y_list_cla, 'r', label = 'non-fuzzy')

    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([0, 300, -20, 50])
    plt.legend()

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, theta_list_cla, 'r', label = 'non-fuzzy')

    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.axis([0, 300, -100, 30])
    plt.legend()

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'b--', label = 'fuzzy')
    plt.plot(x_list_cla, y_list_cla, 'r', label = 'non-fuzzy')

    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.axis([20, 150, -10, 50])
    plt.legend()


    plt.suptitle('Condition 1', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F1C.png")
    plt.show()


def classiccondition3():
    ducktruck(30, -140, 40)
    classictruck(30, -140, 40, 0.19)

    plt.figure(figsize=(16, 12))
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)

    plt.sca(ax1)
    plt.plot(time_list, u_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, u_list_cla, 'r', label = 'non-fuzzy')
    plt.title("u(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("u/Degree", fontsize=20)
    plt.legend()

    plt.sca(ax2)
    plt.plot(time_list, y_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, y_list_cla, 'r', label = 'non-fuzzy')

    plt.title("y(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.legend()

    plt.sca(ax3)
    plt.plot(time_list, theta_list, 'b--', label = 'fuzzy')
    plt.plot(time_list_cla, theta_list_cla, 'r', label = 'non-fuzzy')

    plt.title("θ(t)", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("t/s", fontsize=20)
    plt.ylabel("θ/Degree", fontsize=20)
    plt.legend()

    plt.sca(ax4)
    plt.plot(x_list, y_list, 'b--', label = 'fuzzy')
    plt.plot(x_list_cla, y_list_cla, 'r', label = 'non-fuzzy')

    plt.title("Truck track", fontsize=20)
    plt.grid(linestyle='-.')
    plt.xlabel("x/Meter", fontsize=20)
    plt.ylabel("y/Meter", fontsize=20)
    plt.legend()


    plt.suptitle('Condition 3', fontsize=30)
    plt.tight_layout()
    plt.savefig(r"F:\Master\NTU\7207-Neural and Fuzzy Systems\Assignment\Assignment2\F3C.png")
    plt.show()
x_list = list()
y_list = list()
time_list = list()
theta_list = list()
u_list = list()
x_list_cla = list()
y_list_cla = list()
time_list_cla = list()
theta_list_cla = list()
u_list_cla = list()

fuzzy3Dplot()