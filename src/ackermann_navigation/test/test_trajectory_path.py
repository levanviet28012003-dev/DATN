"""Kiểm tra hình học quỹ đạo và việc giữ đúng nhánh tại giao điểm."""

import importlib.util
import math
from pathlib import Path
import unittest


source = Path(__file__).resolve().parents[1] / 'scripts' / 'trajectory_controller.py'
spec = importlib.util.spec_from_file_location('trajectory_controller', source)
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)


class ReferencePathTests(unittest.TestCase):
    def test_circle_uses_current_position_and_heading(self):
        path = controller.ReferencePath('circle', 1.0, 1, (2.0, 3.0, math.pi/2))
        self.assertEqual(path.points[0], (2.0, 3.0))
        self.assertAlmostEqual(path.total_length, 2*math.pi, places=3)
        self.assertAlmostEqual(path.points[150][0], 1.0)
        self.assertAlmostEqual(path.points[150][1], 4.0)
        self.assertLess(math.dist(path.points[0], path.points[-1]), 1e-10)

    def test_initial_position_does_not_skip_to_loop_end(self):
        path = controller.ReferencePath('circle', 1.0, 2, (0.0, 0.0, 0.0))
        progress, error = path.project(0.0, 0.0, 0.0, 0.8)
        self.assertAlmostEqual(progress, 0.0)
        self.assertAlmostEqual(error, 0.0)

    def test_figure8_preserves_order_across_intersections_and_laps(self):
        path = controller.ReferencePath('figure8', 2.5, 2, (0.0, 0.0, 0.0))
        progress = 0.0
        for i, (x, y) in enumerate(path.points):
            progress, error = path.project(x, y, progress, 0.8)
            self.assertLess(error, 1e-8)
            self.assertAlmostEqual(progress, path.lengths[i], places=6)

    def test_target_clamps_at_end_of_finite_path(self):
        path = controller.ReferencePath('circle', 1.0, 1, (0.0, 0.0, 0.0))
        self.assertEqual(path.target(-1.0), path.points[0])
        self.assertLess(math.dist(path.target(100.0), path.points[-1]), 1e-10)


if __name__ == '__main__':
    unittest.main()
