import math
from dataclasses import dataclass
from enum import IntEnum


@dataclass(frozen=True)
class Params:
    """所有智能体共用的运动参数。"""
    v_z: float = 4.0              # 每次激活的竖直移动距离
    v_xy: float = 1.0             # 每次激活的水平移动距离
    stagger_span: float = 350.0   # 高度错开时各高度层的总跨度
    uniform_tol: float = 0.05     # 均匀化完成时，相邻间距与理想值的容差


class Phase(IntEnum):
    """编队流程（见 docs/flowchart.png）。所有个体都完成当前阶段后，才会进入下一阶段。"""
    CLIMB = 0       # 从地面爬升到编队高度
    STAGGER = 1     # 高度错开：各占一个高度层，水平移动时互不碰撞
    TO_POLYGON = 2  # 抵达多边形：占领最近的空顶点，或向顶点占领者排队
    UNIFORM = 3     # 均匀化：沿边界向"前后邻居中点"与自身位置的中点移动
    CONVERGE = 4    # 高度收敛到编队高度
    DONE = 5


def approach(cur, aim, step):
    """一维上朝 aim 移动至多 step。"""
    d = aim - cur
    return aim if abs(d) <= step else cur + math.copysign(step, d)


class Agent:
    """同质、匿名的智能体：每次被激活，根据当前观察到的其他个体状态做一步决策。

    swarm 是全体智能体列表（含自己），对应"每个个体都能知道其他个体位置并与之通信"的假设。
    决策只读取其他个体的位置、阶段和对询问的应答，不依赖 ID。
    """

    def __init__(self, x, y, z, pgn, swarm, params, rng):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.pgn = pgn
        self.swarm = swarm
        self.params = params

        self.phase = Phase.CLIMB
        self.phase_done = False

        self.rp = rng.random()        # 高度错开：按这个随机数排序分配高度层
        self.layer_h = None           # 高度错开：分到的高度层
        self.slot = None              # 抵达多边形：排队拿到的边上位置
        self.occupied_vertex = None   # 抵达多边形：占领的顶点下标
        self.orders_issued = 0        # 作为顶点占领者已分出的排队序号数

    @property
    def pos(self):
        return self.x, self.y, self.z

    def request_order(self):
        """被其他个体询问排队序号时应答：依次分出 1, 2, ...，名额用完返回 None。"""
        if self.orders_issued >= self.pgn.queue_capacity(len(self.swarm)):
            return None
        self.orders_issued += 1
        return self.orders_issued

    def activate(self):
        """被激活一次：如果大家都完成了当前阶段就进入下一阶段，然后执行当前阶段的一步。"""
        if self.phase == Phase.DONE:
            return
        if self.phase_done and self._everyone_done():
            self.phase = Phase(self.phase + 1)
            self.phase_done = False
            if self.phase == Phase.DONE:
                return
        self._ACTIONS[self.phase](self)

    def _everyone_done(self):
        return all(a.phase > self.phase or (a.phase == self.phase and a.phase_done) for a in self.swarm)

    def _climb(self):
        self.z = approach(self.z, self.pgn.h, self.params.v_z)
        self.phase_done = self.z == self.pgn.h

    def _stagger(self):
        if self.layer_h is None:
            n = len(self.swarm)
            rank = sorted(a.rp for a in self.swarm).index(self.rp)
            self.layer_h = self.pgn.h + (rank - (n - 1) / 2) * self.params.stagger_span / n
        self.z = approach(self.z, self.layer_h, self.params.v_z)
        self.phase_done = self.z == self.layer_h

    def _to_polygon(self):
        if self.phase_done:
            return
        aim, vertex = (self.slot, None) if self.slot is not None else self._choose_vertex()
        if self._move_xy(aim):
            self.occupied_vertex = vertex
            self.phase_done = True

    def _choose_vertex(self):
        """由近到远查看顶点：空顶点就飞过去；已被占领就向占领者要排队序号，拿到后目标固定为排队位置；
        名额用完则看下一个顶点。返回 (目标点, 目标顶点下标或 None)。"""
        n = len(self.swarm)
        for k in self.pgn.vertices_by_distance((self.x, self.y)):
            owner = next((a for a in self.swarm if a.occupied_vertex == k), None)
            if owner is None:
                return self.pgn.vertexs[k], k
            order = owner.request_order()
            if order is not None:
                self.slot = self.pgn.queue_slot(k, order, n)
                return self.slot, None
        raise RuntimeError("所有顶点的排队名额都已用完")  # 容量为 ceil((n-m)/m) 时不会发生

    def _uniform(self):
        L = self.pgn.perimeter
        s = self.pgn.project((self.x, self.y))
        others = [self.pgn.project((a.x, a.y)) for a in self.swarm if a is not self]
        if not others:
            self.phase_done = True
            return
        gap_next = min((o - s) % L for o in others)  # 到前方最近个体的弧长
        gap_pre = min((s - o) % L for o in others)   # 到后方最近个体的弧长
        ideal, tol, v = L / len(self.swarm), self.params.uniform_tol, self.params.v_xy
        self.phase_done = abs(gap_next - ideal) < tol and abs(gap_pre - ideal) < tol
        # 前后邻居中点在 s + (gap_next - gap_pre)/2，目标取它与自身的中点
        delta = max(-v, min(v, (gap_next - gap_pre) / 4))
        self.x, self.y = (float(c) for c in self.pgn.point_at(s + delta))

    def _converge(self):
        self.z = approach(self.z, self.pgn.h, self.params.v_z)
        self.phase_done = self.z == self.pgn.h

    def _move_xy(self, aim):
        """水平方向朝 aim 移动至多 v_xy，到达返回 True。"""
        v = self.params.v_xy
        dx, dy = aim[0] - self.x, aim[1] - self.y
        dist = math.hypot(dx, dy)
        if dist <= v:
            self.x, self.y = float(aim[0]), float(aim[1])
            return True
        self.x += v * dx / dist
        self.y += v * dy / dist
        return False

    _ACTIONS = {
        Phase.CLIMB: _climb,
        Phase.STAGGER: _stagger,
        Phase.TO_POLYGON: _to_polygon,
        Phase.UNIFORM: _uniform,
        Phase.CONVERGE: _converge,
    }
