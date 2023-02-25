# wczytanie potrzebnych podczas zajęć bibliotek:
import numpy as np
from spatialmath import *
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from roboticstoolbox.tools.trajectory import *
from roboticstoolbox.backends.swift import Swift
import time


# ...

# definicje funkcji:
def przykład_1():
    # wykres a)
    #traj = tpoly(0, 1, 50)  # trajektoria w zakresie od 0 do 1 z 50 próbek
    #traj.plot(traj)

    # wykres b)
    #traj = tpoly(0, 1, 50, 0.5)  # trajektoria z niezerową prędkością początkową
    #traj.plot(traj)

    #lspb
    #traj = lspb(0, 1, 50)
    #traj.plot(traj)

    # trajektoria wielomianowa
    #traj = jtraj([0, 2], [1, -1], 50)
    #rtb.qplot(traj.q)
    # lub
    #traj = mtraj(tpoly, [0, 2], [1, -1], 50)
    #traj.plot(traj)

    # trajektoria trapezoidalna
    #traj = mtraj(lspb, [0, 2], [1, -1], 50)
    #traj.plot(traj)

    #robot = rtb.models.DH.Panda()
    #traj = jtraj(robot.qz, robot.qr, 50)
    # wykres pozycji:
    #rtb.qplot(traj.q, block=True)
    # wykres prędkości:
    #rtb.qplot(traj.qd, block=True)
    # wykres przyspieszeń:
    #rtb.qplot(traj.qdd, block=True)

    # lub
    #traj = mtraj(tpoly, robot.qz, robot.qr, 50)
    #traj.plot(traj)

    # Przykład dla robota o 2 stopniach swobody, trajektoria składa się z 4 punktów
    #via_pt = np.array([[0, 0, 0], [1, 0.5, 0.8], [0.2, 2, 1.2], [0.5, 1, 0.5]])
    #traj = mstraj(via_pt, dt=0.02, tacc=0.2, qdmax=2.0)
    #rtb.qplot(traj.q, block=True)

    #T1 = SE3(0.4, 0.2, 0) * SE3.RPY([0, 0, 3])
    #T2 = SE3(-0.4, -0.2, 0.3) * SE3.RPY([-np.pi / 4, np.pi / 4, -np.pi / 2])
    #cart_traj = ctraj(T1, T2, 50)
    #print(cart_traj)

    # Swift
    #robot = rtb.models.Panda()
    #T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    #solution = robot.ikine_LM(T)
    #traj = jtraj(robot.qz, solution.q, 50)
    #robot.plot(traj.q, backend='swift', loop=True)

    # PyPlot
    #robot = rtb.models.DH.Panda()
    #T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    #solution = robot.ikine_LM(T)
    #traj = jtraj(robot.qz, solution.q, 50)
    #robot.plot(traj.q, backend='pyplot', limits=[-0.25, 1.25, -0.5, 0.5, 0, 1], movie='panda_pyplot.gif')

    # VPython
    #robot = rtb.models.DH.Panda()
    #T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
    #solution = robot.ikine_LM(T)
    #traj = jtraj(robot.qz, solution.q, 50)

    #env = VPython()
    #env.launch()
    #env.add(fig_num=0, dhrobot=robot)
    #for q in traj.q:
    #    time.sleep(0.02)
    #    env.step(dt=0.025, id=robot, q=q)
    pass

def zadanie_1():
    robot=rtb.models.Puma560()
    T_start=robot.fkine(robot.qz)
    #T_end=T_start*SE3(0.2,0,0)*SE3.RPY(0,90,0,unit='deg')
    #T_end = T_start * SE3(0.2, 0, 0) * SE3.OA([0,1,0],[1,0,0])
    #T_end = T_start * SE3(0.2, 0, 0) * SE3.OA([0, 0, -1], [0, 1, 0])
    T_end = T_start * SE3(0.2, 0, 0) * SE3.OA([0, -1, 0], [0, 0, -1])
    solution=robot.ikine_LM(T_end)
    traj = jtraj(robot.qz, solution.q, 50)
    #robot.plot(traj.q, backend='swift', loop=True)
    print(traj)

def zadanie_2():
    pass

def zadanie_3():
    #wygenerowanie punktów na okręgu
    r=0.1
    x_c=0.65
    y_c=0.2
    robot = rtb.models.Panda()
    number_of_points = 20
    angles=np.linspace(0,2*np.pi,number_of_points)
    x=r*np.cos(angles)+x_c
    y=r*np.sin(angles)+y_c
    z=np.ones(number_of_points)*0.15
    #plt.plot(x,y)
    #plt.show()
    #exit()
    via_points=[[[0 for k in range(6)] for j in range(50)] for i in range(number_of_points)]

    #utworzenie macierzy transformacji
    for i in range(number_of_points):
        T=SE3(x[i],y[i],z[i])*SE3.RPY(180,0,0,unit='deg')
        via_points[i]=robot.ikine_LM(T).q

    T_start= np.array([robot.qz])
    viapoints = np.vstack([T_start, via_points])

    traj = mstraj(np.array(viapoints), dt=0.02, tacc=0.2, qdmax=2.0)
    #print(traj.q)
    robot.plot(traj.q, backend='swift', loop='True')


# wykonywanie wybranej funkcji
if __name__ == '__main__':
    przykład_1()
    zadanie_1()
    zadanie_2()
    zadanie_3()