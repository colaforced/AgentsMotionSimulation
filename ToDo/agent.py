import random
import numpy as np

class Agent:
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.v_1 = 5; self.error_1 = 2
        self.v_2 = 3; self.error_2 = 1

        
        self.rp = 0  # 高度错开随机参数
        self.tph = 0  # 高度错临时高度
        self.state = 0
        self.isStop = 0
        self.agents = None
        self.pgn = None
        self.flag_1 = False; self.flag_2 = False; self.flag_3 = False
        self.flag_4 = False; self.flag_5 = False; self.flag_6 = False
        self.edge_pos_arr = []

        # 抵达多边形参数
        self.state_2 = 0

    def move_in_longitude_1(self):
        """爬升至多边形设定高度"""
        if self.flag_1:
            return self.gen_random_p()
        d = self.pgn.h - self.z
        if abs(d) < self.error_1:
            self.z = self.pgn.h
            self.flag_1 = True
        else:
            self.z += self.v_1 if d > 0 else -self.v_1

    def gen_random_p(self):
        """生成排序所用随机数"""
        if self.flag_2:
            return self.wait_for_height_sort()
        self.rp = random.random()
        self.flag_2 = True

    def wait_for_height_sort(self):
        """等待所有智能体进入高度排序状态"""
        self.flag_5 = True
        if self.flag_6: return self.gen_temp_height()

        if self.check_all_flag_5():
            self.set_all_flag_6()


    def gen_temp_height(self):
        if self.flag_3:
            return self.move_in_longitude_2()
        sagents = sorted(self.agents, key=lambda x:x.rp)
        idx = sagents.index(self.agents[self.id])
        self.tph = self.pgn.h + (idx - len(self.agents)//2) * (350/len(self.agents))
        self.flag_3 = True

    def move_in_longitude_2(self):
        if self.flag_4: return
        d = self.tph - self.z
        if abs(d) < self.error:
            self.z = self.tph
            self.flag_4 = True
            if self.check_all_flag_4():
                self.set_all_agents_state(1)
        else:
            self.z += self.v if d > 0 else -self.v

    def move_to_vertex(self):
        ix = self.cal_nearest_vertex()
        if not ix:
            return self.move_to_edge()



    def move_to_edge(self):
        """移动到多边性边上"""
        pass

    def move_on_edge(self):
        """多边形上移动"""
        pass

    def update_forward(self, aim):
        x = np.array([aim[0] - self.x, aim[1] - self.y])  # 方向向量
        y = np.array([1, 0])  # x轴方向
        mx = np.sqrt(x.dot(x))  # x.dot(x) 点乘自己，相当于向量模平方
        my = np.sqrt(y.dot(y))
        if mx > self.v_2:
            cos_angle = x.dot(y) / (mx * my)
            angle = np.arccos(cos_angle)  # 0.....pi
            if x[0] >= 0 and x[1] >= 0:
                self.x = self.x + self.v_2 * abs(np.cos(angle))
                self.y = self.y + self.v_2 * np.sin(angle)
            elif x[0] <= 0 and x[1] >= 0:
                self.x = self.x - self.v_2 * abs(np.cos(angle))
                self.y = self.y + self.v_2 * np.sin(angle)
            elif x[0] <= 0 and x[1] <= 0:
                self.x = self.x - self.v_2 * abs(np.cos(angle))
                self.y = self.y - self.v_2 * np.sin(angle)
            else:
                self.x = self.x + self.v_2 * abs(np.cos(angle))
                self.y = self.y - self.v_2 * np.sin(angle)
        else:
            self.x = aim[0]
            self.y = aim[1]



    def limited_state_machine(self):
        """运动决策有限状态机"""
        if self.isStop: return
        """0~3表示三种运动决策"""
        if self.state == 0:
            self.move_in_longitude_1()
        elif self.state == 1:
            self.move_to_vertex()
        elif self.state == 2:
            self.move_to_edge()
        elif self.state == 3:
            self.move_on_edge()

    def get_id(self):
        return self.id

    def get_pos(self):
        return self.x, self.y, self.z

    def check_all_flag_2(self):
        for a in self.agents:
            if a.flag_2 == False:
                return False
        return True

    def check_all_flag_4(self):
        for a in self.agents:
            if a.flag_4 == False:
                return False
        return True

    def set_all_agents_state(self, n):
        for a in self.agents:
            a.state = n

    def check_all_flag_5(self):
        for a in self.agents:
            if a.flag_5 == False:
                return False
        return True

    def set_all_flag_6(self):
        for a in self.agents:
            a.flag_6 = True

    def cal_distance_2d(self, a, b):
        return (a[0]-b[0])**2 + (a[1]-b[1])**2

    def cal_nearest_vertex(self):
        temp = []
        for i in range(len(self.pgn.vertexs)):
            d = self.cal_distance_2d([self.pgn.vertexs[i][0],self.pgn.vertexs[i][1]], \
                                     [self.x,self.y])
            temp.append([i,d])
        temp.sort(temp, key=lambda x:x[1])
        for cb in temp:
            if not self.pgn.isOccupied[cb[0]]:
                return cb[0]
        else:
            return False

    def inquire_edge_pos(self, id):
        return []







