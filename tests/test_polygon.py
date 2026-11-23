import unittest

import numpy as np

from formation import Polygon


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


if __name__ == '__main__':
    unittest.main()
