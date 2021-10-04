import copy
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

"""无人机设置"""
n = 8  # 个数

"""多边形参数设置"""
m = 5  # 边数
r = 30  # 多边形半径
cx, cy, cz = 50, 50, 200  # 中心位置

"""高度错开收敛所需参数"""
hv = 2  # 高度错开收敛时，纵轴运动速度
hgap = 380 / n

"""抵达边界所需参数"""
step_len = 0.3  # 移动步长
Detect_Error = 0.1  # 允许误差

"""边界目标位置调整所需参数"""
max_n = (n - m) / m if ((n - m) / m) % 1 == 0 else int((n - m) / m) + 1
unit_gap = 2 * r * np.sin(np.pi / m) / (max_n + 1)  # 2 * R * np.sin(np.pi/n)是边界长度， unit_gap是调整单位距离

"""均匀化参数"""
Stop_Gap_NUM = 3  # 齿链上，若所有结点两两之间仅仅相小于3个节点，则判断为编队完成
Move_Gap_Num = 20  # 20 * 0.01 =0.2 齿链上，步长转换成节点数

"""随机生成无人机初始位置,记录为Point_List"""
pxs = np.random.randint(1, 100, n)
pys = np.random.randint(1, 100, n)
pzs = [0 for i in range(n)]
Point_List = [[pxs[i], pys[i], 0] for i in range(n)]

""" 计算多边形顶点位置"""
Vertexs_List = [[50 + r * np.sin(i * 2 * np.pi / m), 50 + r * np.cos(i * 2 * np.pi / m), cz] for i in range(1, m + 1)]
verxs, verys, verzs = [v[0] for v in Vertexs_List], [v[1] for v in Vertexs_List], [v[2] for v in Vertexs_List]
verxs.append(Vertexs_List[0][0])
verys.append(Vertexs_List[0][1])
verzs.append(Vertexs_List[0][2])

"""初始化图像"""
fig = plt.figure()
ax = plt.gca(projection='3d')
ax.set_xlim(0, 100)
ax.set_xlabel('X')
ax.set_ylim(0, 100)
ax.set_ylabel('Y')
ax.set_zlim(0, 400)
ax.set_zlabel('Z')
sc = ax.scatter3D(pxs, pys, pzs, color='r', alpha=0.7)
ax.plot(verxs, verys, verzs, color='black', linestyle=':')

"""均匀化时,链表化节点参数"""
d = 2 * r * np.sin(np.pi / m)
num = int(d / 0.01)
tooth_distance = d / num
Vertexs_Tooth_Chain = []

"""链表化多边形"""
for i in range(0, m):
    Vertexs_Tooth_Chain.append(Vertexs_List[i])
    base_pos = copy.deepcopy(Vertexs_List[i])
    if i == len(Vertexs_List) - 1:
        nxt = Vertexs_List[0]
    else:
        nxt = Vertexs_List[i + 1]
    x = np.array([nxt[0] - Vertexs_List[i][0], nxt[1] - Vertexs_List[i][1]])  # 方向向量
    y = np.array([1, 0])  # x轴方向
    xmd = np.sqrt(x.dot(x))  # x.dot(x) 点乘自己，相当于向量模平方
    ymd = np.sqrt(y.dot(y))
    cos_angle = x.dot(y) / (xmd * ymd)
    angle = np.arccos(cos_angle)
    if x[0] >= 0 and x[1] >= 0:
        for j in range(1, num):
            a = base_pos[0] + j * tooth_distance * abs(np.cos(angle))
            b = base_pos[1] + j * tooth_distance * abs(np.sin(angle))
            Vertexs_Tooth_Chain.append([a, b, cz])
    elif x[0] <= 0 and x[1] >= 0:
        for j in range(1, num):
            a = base_pos[0] - j * tooth_distance * abs(np.cos(angle))
            b = base_pos[1] + j * tooth_distance * abs(np.sin(angle))
            Vertexs_Tooth_Chain.append([a, b, cz])
    elif x[0] <= 0 and x[1] <= 0:
        for j in range(1, num):
            a = base_pos[0] - j * tooth_distance * abs(np.cos(angle))
            b = base_pos[1] - j * tooth_distance * abs(np.sin(angle))
            Vertexs_Tooth_Chain.append([a, b, cz])
    else:
        for j in range(1, num):
            a = base_pos[0] + j * tooth_distance * abs(np.cos(angle))
            b = base_pos[1] - j * tooth_distance * abs(np.sin(angle))
            Vertexs_Tooth_Chain.append([a, b, cz])

"""------------------------分割线---------------------------------------------"""
"""由于智能体策略相对复杂，一个类写下所有状态下相应决策，代码过长，便按照当前状态的不同策略拆分成两个类"""


class Point_1():
    """第一阶段智能体模型表示: 高度错开 + 移动到多边形上"""

    def __init__(self, id):
        self.id = id
        self.random = np.random.random()  # 随机数，用于高度错开
        self.aim_hight = None  # 当前阶段运动目标位置
        self.aim_adjusted = None  # 多边形顶点位置被占领后，重新确定的目标
        self.max_n = max_n

    def decide0(self):
        """抵达目标高度"""
        my_pos = copy.deepcopy(Point_List[self.id])
        if abs(cz - my_pos[2]) < hv:
            Point_List[self.id][2] = cz
        else:
            aim = my_pos[2] + hv
            Point_List[self.id][2] = aim

    def decide1(self):
        """高度错开"""
        if self.aim_hight == None:
            li1 = [[Obj_List[i].send_random(), i] for i in range(n)]

            li1.sort(key=lambda x: x[0])
            my_index = li1.index([self.random, self.id])
            self.aim_hight = 10 + (my_index + 1) * hgap  # 因为my_index从0 开始取
        # 有目标高度后开始更新位置
        my_pos = copy.deepcopy(Point_List[self.id])
        if abs(my_pos[2] - self.aim_hight) < hv:
            Point_List[self.id][2] = self.aim_hight
        elif my_pos[2] < self.aim_hight:
            Point_List[self.id][2] = my_pos[2] + hv
        else:
            Point_List[self.id][2] = my_pos[2] - hv

    def decide2(self, list=copy.deepcopy(Vertexs_List)):
        """抵达多边形主要逻辑： 抵达顶点 + 顶点被占领后调整目标位置"""
        if self.aim_adjusted == None:
            nearest = self.detect_nearest(list)  # 检测最近顶点
            idx = self.occupy(nearest)
            if idx == self.id:
                self.update(nearest)
            elif idx == None:
                self.update(nearest)
            else:
                self.aim_adjusted = self.adjust(idx)
                if self.aim_adjusted:  # 调整成功
                    self.update(self.aim_adjusted)
                else:
                    list2 = copy.deepcopy(list)
                    list2.remove(nearest)
                    return self.decide2(list2)
        else:
            self.update(self.aim_adjusted)

    def send_random(self):
        return self.random

    def detect_nearest(self, list):
        """检测最近的多边形顶点"""
        init_distance = self.distance_calculate(Point_List[self.id], list[0])
        count, i = 0, 0
        for each in list:
            D = self.distance_calculate(Point_List[self.id], each)
            if D < init_distance:
                init_distance = D
                count = i
            i += 1
        return list[count]

    def distance_calculate(self, A, B):  # [1,1,?],[2,2,?] 得1.4142135623730951
        return pow(pow(abs(A[0] - B[0]), 2) + pow(abs(A[1] - B[1]), 2), 0.5)

    def occupy(self, nearest):
        """检测是谁占领的该多边形顶点，返回其id"""
        for each in Point_List:
            d = self.distance_calculate(each, nearest)
            if d < Detect_Error:
                id = Point_List.index(each)
                return id
        return None

    def update(self, aim):
        """朝着目标位置运动，更新坐标"""
        self_pot = copy.deepcopy(Point_List[self.id])
        x = np.array([aim[0] - self_pot[0], aim[1] - self_pot[1]])  # 方向向量
        y = np.array([1, 0])  # x轴方向
        xmd = np.sqrt(x.dot(x))  # x.dot(x) 点乘自己，相当于向量模平方
        ymd = np.sqrt(y.dot(y))
        if xmd > step_len:
            cos_angle = x.dot(y) / (xmd * ymd)
            angle = np.arccos(cos_angle)  # 0.....pi
            if x[0] >= 0 and x[1] >= 0:
                self_pot[0] = self_pot[0] + step_len * abs(np.cos(angle))
                self_pot[1] = self_pot[1] + step_len * np.sin(angle)
            elif x[0] <= 0 and x[1] >= 0:
                self_pot[0] = self_pot[0] - step_len * abs(np.cos(angle))
                self_pot[1] = self_pot[1] + step_len * np.sin(angle)
            elif x[0] <= 0 and x[1] <= 0:
                self_pot[0] = self_pot[0] - step_len * abs(np.cos(angle))
                self_pot[1] = self_pot[1] - step_len * np.sin(angle)
            else:
                self_pot[0] = self_pot[0] + step_len * abs(np.cos(angle))
                self_pot[1] = self_pot[1] - step_len * np.sin(angle)
            Point_List[self.id] = self_pot
        else:
            Point_List[self.id][0] = aim[0]
            Point_List[self.id][1] = aim[1]

    def adjust(self, idx):
        """多边形顶点被占领后，向占领者询问自己的目标位置"""
        order = Obj_List[idx].send_order()
        if order == None: return None
        for each in Vertexs_List:
            d = self.distance_calculate(each, Point_List[idx])
            if d < Detect_Error:
                identity = Vertexs_List.index(each)
        aim = copy.deepcopy(Vertexs_List[identity])
        count = self.max_n - order  # 1,2
        if identity == 0:
            pre = Vertexs_List[-1]
            nxt = Vertexs_List[identity + 1]
        elif identity == len(Vertexs_List) - 1:
            pre = Vertexs_List[identity - 1]
            nxt = Vertexs_List[0]
        else:
            pre = Vertexs_List[identity - 1]
            nxt = Vertexs_List[identity + 1]

        if count % 2 == 0:  # 偶数顺时针
            x = np.array([pre[0] - aim[0], pre[1] - aim[1]])  # 方向向量
        else:  # 奇数逆时针
            x = np.array([nxt[0] - aim[0], nxt[1] - aim[1]])  # 方向向量
        count2 = count / 2 if count % 2 == 0 else int(count / 2) + 1
        y = np.array([1, 0])  # x轴方向
        xmd = np.sqrt(x.dot(x))  # x.dot(x) 点乘自己，相当于向量模平方
        ymd = np.sqrt(y.dot(y))
        cos_angle = x.dot(y) / (xmd * ymd)
        angle = np.arccos(cos_angle)
        if x[0] >= 0 and x[1] >= 0:

            aim[0] = aim[0] + count2 * unit_gap * abs(np.cos(angle))
            aim[1] = aim[1] + count2 * unit_gap * np.sin(angle)
        elif x[0] <= 0 and x[1] >= 0:
            aim[0] = aim[0] - count2 * unit_gap * abs(np.cos(angle))
            aim[1] = aim[1] + count2 * unit_gap * np.sin(angle)
        elif x[0] <= 0 and x[1] <= 0:
            aim[0] = aim[0] - count2 * unit_gap * abs(np.cos(angle))
            aim[1] = aim[1] - count2 * unit_gap * np.sin(angle)
        else:
            aim[0] = aim[0] + count2 * unit_gap * abs(np.cos(angle))
            aim[1] = aim[1] - count2 * unit_gap * np.sin(angle)
        return aim

    def send_order(self):
        if self.max_n <= 0:
            return None  # 告诉询问着索要失败
        else:
            self.max_n -= 1
            return self.max_n


"""------------------------分割线---------------------------------------------"""


class Point2():
    """第二阶段智能体模型： 均匀化分布 + 高度收敛"""
    next_dis = len(Vertexs_Tooth_Chain) + 1

    def __init__(self, id):
        """初始化,绑定前后智能体关系"""
        self.id = id
        my_id = PointAddIndexSorted.index(Point_addIndex[self.id])
        if my_id == 0:
            self.pre_id = Point_addIndex.index(PointAddIndexSorted[n - 1])
            self.next_id = Point_addIndex.index(PointAddIndexSorted[1])
        elif my_id == n - 1:
            self.next_id = Point_addIndex.index(PointAddIndexSorted[0])
            self.pre_id = Point_addIndex.index(PointAddIndexSorted[n - 2])
        else:
            self.pre_id = Point_addIndex.index(PointAddIndexSorted[my_id - 1])
            self.next_id = Point_addIndex.index(PointAddIndexSorted[my_id + 1])

    def decide1(self):
        """均匀化，往前后节点的中间位置移动； 再有拐角的多边形上，怎么计算中间位置？
            答案： 把多边形等距划开，链表化，看成一条尺链，计算出每一个节点的坐标，
                  计算中间位置，就只需要计算所需要移动的节点的索引

            补充：这只是编程实现手法，现实情况，一个agent，定位前后agent位置，
                 加上多边形位置已知，往其中点移动使可实现的，所以不矛盾。
        """
        pre_chain_index = Point_addIndex[self.pre_id][3]
        next_chain_index = Point_addIndex[self.next_id][3]
        self_chain_index = Point_addIndex[self.id][3]
        if pre_chain_index < next_chain_index:  # 正常情况
            self.next_dis = next_chain_index - self_chain_index
            mmid = ((next_chain_index + pre_chain_index) / 2 + self_chain_index) / 2
        else:
            self.next_dis = next_chain_index - self_chain_index + len(Vertexs_Tooth_Chain)
            if self.next_dis >= len(Vertexs_Tooth_Chain):
                self.next_dis -= len(Vertexs_Tooth_Chain)
            mmid = ((next_chain_index + len(Vertexs_Tooth_Chain) + pre_chain_index) / 2 + self_chain_index) / 2

        if abs(mmid - self_chain_index) <= Stop_Gap_NUM:
            if mmid % 1 == 0:
                self.move(int(mmid))
            elif self_chain_index > mmid:  # 在目标顺时针方向
                self.move(int(mmid) + 1)
            else:
                self.move(int(mmid))
        elif mmid > self_chain_index:
            self.move(self_chain_index + Move_Gap_Num)
        else:
            self.move(self_chain_index - Move_Gap_Num)

    def decide2(self):
        """高度收敛"""
        my_hight = Point_List[self.id][2]
        if abs(my_hight - cz) < hv:
            Point_List[self.id][2] = cz
        elif my_hight > cz:
            Point_List[self.id][2] = my_hight - hv
        else:
            Point_List[self.id][2] = my_hight + hv

    def move(self, aim):
        if aim >= len(Vertexs_Tooth_Chain): aim -= len(Vertexs_Tooth_Chain)
        li = copy.deepcopy(Vertexs_Tooth_Chain[aim])
        Point_List[self.id][0] = copy.deepcopy(Vertexs_Tooth_Chain[aim])[0]
        Point_List[self.id][1] = copy.deepcopy(Vertexs_Tooth_Chain[aim])[1]
        li.append(aim)
        Point_addIndex[self.id] = li


"""实例化多个对象"""
Obj_List = [Point_1(i) for i in range(0, n)]  # 返回生成的n个对象的列表

"""排序后带索引的节点列表"""
PointAddIndexSorted = []

"""---------------------------分割线------------------------"""


def gen():
    """仿真数据源"""
    global PointAddIndexSorted
    global Point_addIndex
    global Vertexs_Tooth_Chain
    state_flag = 0
    Point_List2 = []  # 用于比较
    init_flag = 0

    def distance_calculate_2D(A, B):
        return pow(pow(A[0] - B[0], 2) + pow(A[1] - B[1], 2), 0.5)

    while True:
        JudgeList = []
        if state_flag == 0:
            for i in range(n):
                Obj_List[i].decide0()
        elif state_flag == 1:
            for i in range(n):
                Obj_List[i].decide1()
        elif state_flag == 2:
            for i in range(n):
                Obj_List[i].decide2()
        elif state_flag == 3:

            if init_flag == 0:
                Point_addIndex = []
                for i in Point_List:
                    for j in Vertexs_Tooth_Chain:
                        if distance_calculate_2D(i, j) <= tooth_distance / 2:
                            li = copy.deepcopy(i)
                            li.append(Vertexs_Tooth_Chain.index(j))
                            Point_addIndex.append(li)
                            break

                PointAddIndexSorted = copy.deepcopy(Point_addIndex)
                PointAddIndexSorted.sort(key=lambda x: x[3])
                Obj_List2 = [Point2(i) for i in range(0, n)]  # 返回生成的n个对象的列表
                init_flag = 1

            for i in range(n):
                Obj_List2[i].decide1()
                JudgeList.append(Obj_List2[i].next_dis)
            if judge(JudgeList):
                state_flag += 1
        elif state_flag == 4:
            for i in range(n):
                Obj_List2[i].decide2()

        else:
            print("最终编队完成, 10s后退出")
            time.sleep(10)
            exit()
        if Point_List2 == Point_List:
            state_flag += 1
        else:
            pass
        Point_List2 = copy.deepcopy(Point_List)
        yield Point_List


def judge(arg_list):
    d = len(Vertexs_Tooth_Chain) / n
    for each in arg_list:
        if abs(each - d) > 100:
            return False
    return True


def update(arg_list):
    """刷新动画"""
    li = np.array(arg_list)
    sx, sy, sz = [], [], []
    for each in arg_list:
        sx.append(each[0])
        sy.append(each[1])
        sz.append(each[2])
    sc.set_offsets(li[:, :-1])
    sc.set_3d_properties(sz, zdir='z')
    return sc


ani = animation.FuncAnimation(fig, update, frames=gen, interval=1)
plt.show()
