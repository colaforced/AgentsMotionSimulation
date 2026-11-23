import itertools
import math
import unittest

import numpy as np

from formation import Phase, Simulation


def min_pair_distance(p):
    d = np.linalg.norm(p[:, None] - p[None], axis=2)
    return (d + np.eye(len(p)) * 1e9).min()


class SimulationTest(unittest.TestCase):
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
        tol = sim.params.uniform_tol
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
