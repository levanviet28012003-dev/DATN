"""Kiểm tra phép chiếu ô ảnh và model mặt đất ảnh vệ tinh, không cần mạng."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

package_dir = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


satellite = load('generate_satellite_ground', package_dir / 'scripts' / 'generate_satellite_ground.py')
outdoor = load('outdoor_launch', package_dir / 'launch' / 'outdoor.launch.py')


class TileMathTests(unittest.TestCase):
    def test_global_pixel_matches_tile_index(self):
        # Ô zoom 18 chứa (10°, 106°): x = floor(286/360 · 2^18) = floor(208258,8) = 208258,
        # y = 123752 (công thức Web Mercator).
        x, y = satellite.global_pixel(10.0, 106.0, 18)
        self.assertEqual((int(x // 256), int(y // 256)), (208258, 123752))
        self.assertEqual(satellite.global_pixel(0.0, 0.0, 0), (128.0, 128.0))

    def test_meters_per_pixel(self):
        self.assertAlmostEqual(satellite.meters_per_pixel(0.0, 0), 156543.03, places=1)
        self.assertAlmostEqual(satellite.meters_per_pixel(10.0, 18), 0.5881, places=3)

    def test_crop_window_is_centered_and_sized(self):
        left, top, right, bottom = satellite.crop_window(10.0, 106.0, 18, 160.0)
        cx, cy = satellite.global_pixel(10.0, 106.0, 18)
        self.assertAlmostEqual((left + right) / 2, cx)
        self.assertAlmostEqual((top + bottom) / 2, cy)
        self.assertAlmostEqual((right - left) * satellite.meters_per_pixel(10.0, 18), 160.0)
        x0, x1, y0, y1 = satellite.tile_range((left, top, right, bottom))
        self.assertLessEqual(x0 * 256, left)
        self.assertGreaterEqual((x1 + 1) * 256, right)
        self.assertLessEqual(y0 * 256, top)
        self.assertGreaterEqual((y1 + 1) * 256, bottom)

    def test_texture_size_is_power_of_two(self):
        self.assertEqual(satellite.texture_size(100), 256)
        self.assertEqual(satellite.texture_size(272), 512)
        self.assertEqual(satellite.texture_size(10000), 4096)

    def test_placeholder_detection(self):
        from PIL import Image, ImageDraw
        gray = Image.new('RGB', (256, 256), (204, 204, 204))
        ImageDraw.Draw(gray).text((60, 120), 'Map data not yet available', fill=(255, 255, 255))
        self.assertTrue(satellite.is_placeholder(gray))
        noisy = Image.effect_noise((256, 256), 60).convert('RGB')
        self.assertFalse(satellite.is_placeholder(noisy))


class ModelTests(unittest.TestCase):
    def test_quad_spans_size_with_north_up_uv(self):
        lines = satellite.obj_quad(100.0).splitlines()
        vertices = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith('v ')]
        uvs = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith('vt ')]
        self.assertEqual(vertices[0][:2], (-50.0, -50.0))
        self.assertEqual(vertices[2][:2], (50.0, 50.0))
        # Góc Tây Nam ứng với UV (0, 0), góc Đông Bắc với (1, 1); đã kiểm bằng camera nhìn xuống.
        self.assertEqual(uvs[0], (0.0, 0.0))
        self.assertEqual(uvs[2], (1.0, 1.0))

    def test_model_is_visual_only(self):
        root = ElementTree.fromstring(satellite.model_sdf('/a/ground.obj', '/a/satellite.png'))
        self.assertEqual(root.findtext('model/static'), 'true')
        self.assertIsNone(root.find('.//collision'))
        self.assertEqual(root.findtext('.//albedo_map'), '/a/satellite.png')

    def test_launch_uses_imagery_only_for_the_same_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            model, reason = outdoor.satellite_ground(10.0, 106.0, directory)
            self.assertIsNone(model)
            (Path(directory) / 'model.sdf').write_text('<sdf/>')
            (Path(directory) / 'meta.yaml').write_text('latitude: 10.0\nlongitude: 106.0\n')
            model, reason = outdoor.satellite_ground(10.0, 106.0, directory)
            self.assertEqual(model, str(Path(directory) / 'model.sdf'))
            model, reason = outdoor.satellite_ground(21.0, 105.8, directory)
            self.assertIsNone(model)
            self.assertIn('khác mốc', reason)

    def test_world_origin_matches_launch(self):
        world = package_dir / 'worlds' / 'outdoor_world.sdf'
        self.assertEqual(satellite.world_origin(world), outdoor.world_origin(world)[:2])


if __name__ == '__main__':
    unittest.main()
