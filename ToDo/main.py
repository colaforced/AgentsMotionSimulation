from polygon import Polygon
from agent import Agent
from random import randint, shuffle
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np


class MultiMotionSimulate:
    def __init__(self, n, m, r, h, cx, cy):
        self.pgn = Polygon(m, r, h, cx, cy)

        self.n = n  # 初始化多n个智能体
        self.s = [i for i in range(n)]  # 随机启动顺序
        xd, xh = 0, 100
        yd, yh = 0, 100
        self.agents = [Agent(i, randint(xd, xh), randint(yd, yh), 0) for i in range(n)]

        self.fig, self.ax, self.sc = None, None, None
        self.init_show()

        for a in self.agents:
            a.pgn = self.pgn
            a.agents = self.agents

        self.isStop = False

    def init_show(self):
        """初始画布,"""
        self.fig = plt.figure()
        self.ax = plt.gca(projection='3d')
        self.ax.set_xlim(0, 100)
        self.ax.set_xlabel('X')
        self.ax.set_ylim(0, 100)
        self.ax.set_ylabel('Y')
        self.ax.set_zlim(0, 400)
        self.ax.set_zlabel('Z')
        vxs, vys, vzs = [v[0] for v in self.pgn.vertexs], [v[1] for v in self.pgn.vertexs], [self.pgn.h] * len(
            self.pgn.vertexs)  # 多边形顶点坐标
        vxs.append(vxs[0]);
        vys.append(vys[0]);
        vzs.append(vzs[0])  # 线条首尾相连
        self.ax.plot(vxs, vys, vzs, color='black', linestyle=':')
        pxs, pys, pzs = [p.x for p in self.agents], [p.y for p in self.agents], [p.z for p in self.agents]  # agent坐标
        self.sc = self.ax.scatter3D(pxs, pys, pzs, color='r', alpha=0.7)

    def observe(self):
        """观察所有agents的状态"""
        while not self.isStop:
            shuffle(self.s)
            for c in self.s:
                c = randint(0, self.n - 1)
                self.agents[c].limited_state_machine()
            yield None

    def deal_3d(self, agents):
        """3d画图数据设置"""
        pzs = [p.z for p in self.agents]
        temp = np.array([[p.x, p.y, p.z] for p in self.agents])
        self.sc.set_offsets(temp[:, :-1])
        self.sc.set_3d_properties(pzs, zdir='z')
        return self.sc

    def show(self):
        """画图"""
        ani = animation.FuncAnimation(self.fig, func = self.deal_3d, frames = self.observe, interval= 1)
        plt.show()


if __name__ == '__main__':
    # 初始化目标多边形，边形为5，半径30，高度200, 中心坐标为50，50
    mms = MultiMotionSimulate(10, 5, 30, 200, 50, 50)
    mms.show()
