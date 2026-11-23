import itertools
import math
import unittest

import numpy as np

from formation import Phase, Polygon, Simulation


class PolygonTest(unittest.TestCase):
    def test_arc_length_roundtrip(self):
        pgn = Polygon(5, 30, 200)
        for s in np.linspace(0, pgn.perimeter, 97, endpoint=False):
            self.assertAlmostEqual(pgn.project(pgn.point_at(s)), s, places=9)

    def test_vertices_on_circle(self):
        pgn = Polygon(6, 20, 100, cx=10, cy=-5)
        r = np.linalg.norm(pgn.vertexs - [10, -5], axis=1)
        np.testing.assert_allclose(r, 20)

    def test_queue_slots_distinct_and_off_vertices(self):
        for m, n in [(3, 25), (4, 9), (5, 10), (5, 8), (6, 20)]:
            pgn = Polygon(m, 30, 200)
            cap = pgn.queue_capacity(n)
            self.assertGreaterEqual(m * (cap + 1), n)
            pts = [pgn.vertexs[k] for k in range(m)]
            pts += [pgn.queue_slot(k, o, n) for k in range(m) for o in range(1, cap + 1)]
            pts = np.array(pts)
            d = np.linalg.norm(pts[:, None] - pts[None], axis=2) + np.eye(len(pts)) * 1e9
            self.assertGreater(d.min(), 1e-6, f"m={m}, n={n} 有重叠的排队位置")


def min_pair_distance(p):
    d = np.linalg.norm(p[:, None] - p[None], axis=2)
    return (d + np.eye(len(p)) * 1e9).min()


class FormationTest(unittest.TestCase):
    COLLISION_DIST = 1.0

    def check(self, n, m, seed):
        sim = Simulation(n=n, m=m, seed=seed)
        closest = math.inf
        while not sim.done and sim.rounds < 20000:
            sim.step()
            if n > 1:
                closest = min(closest, min_pair_distance(sim.positions()))
        tag = f"n={n}, m={m}, seed={seed}"
        self.assertTrue(sim.done, f"{tag} 未在 20000 轮内完成")
        self.assertGreater(closest, self.COLLISION_DIST, f"{tag} 过程中两机距离过近")

        pgn, p = sim.pgn, sim.positions()
        np.testing.assert_allclose(p[:, 2], pgn.h, err_msg=f"{tag} 高度未收敛")
        for x, y, _ in p:
            on_edge = np.linalg.norm(pgn.point_at(pgn.project((x, y))) - (x, y))
            self.assertLess(on_edge, 1e-9, f"{tag} 有个体不在多边形边界上")
        s = np.sort([pgn.project((x, y)) for x, y, _ in p])
        gaps = np.diff(np.append(s, s[0] + pgn.perimeter))
        tol = sim.agents[0].uniform_tol
        self.assertLess(np.abs(gaps - pgn.perimeter / n).max(), tol, f"{tag} 分布不均匀: {gaps}")

    def test_formations(self):
        for n, m in itertools.product([1, 2, 3, 5, 8, 10, 16, 25], [3, 4, 5, 6, 8]):
            for seed in range(5):
                with self.subTest(n=n, m=m, seed=seed):
                    self.check(n, m, seed)

    def test_phase_barrier(self):
        """任何时刻各个体所处阶段至多相差 1。"""
        sim = Simulation(n=12, m=5, seed=7)
        while not sim.done:
            sim.step()
            phases = [a.phase for a in sim.agents]
            self.assertLessEqual(max(phases) - min(phases), 1)

    def test_reproducible_with_seed(self):
        a, b = Simulation(seed=42), Simulation(seed=42)
        a.run(), b.run()
        self.assertEqual(a.rounds, b.rounds)
        np.testing.assert_array_equal(a.positions(), b.positions())
        self.assertTrue(all(x.phase == Phase.DONE for x in a.agents))


if __name__ == '__main__':
    unittest.main()
