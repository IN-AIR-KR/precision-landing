import math
import unittest

import numpy as np

from bag_reader import relative_ground_truth


def pose(position, quaternion=(1., 0., 0., 0.), stamp=1):
    return {
        'source_stamp_ns': stamp,
        **dict(zip(('pose_position_' + axis for axis in 'xyz'), position)),
        **dict(zip(('pose_orientation_' + axis for axis in 'wxyz'), quaternion)),
    }


class RelativeGroundTruthTest(unittest.TestCase):
    def test_identity_and_translation(self):
        result = relative_ground_truth([(10, pose((5, 6, 7)))], [(11, pose((7, 9, 3)))])
        self.assertEqual(result, [(11, {'position_m_0': 2., 'position_m_1': -3., 'position_m_2': 4.})])

    def test_inverse_rotation_and_frd(self):
        q = (math.sqrt(.5), 0., 0., math.sqrt(.5))
        result = relative_ground_truth([(10, pose((0, 0, 0), q))], [(11, pose((2, 0, -3)))])
        np.testing.assert_allclose(list(result[0][1].values()), [0., 2., 3.], atol=1e-12)

    def test_pair_source_time_despite_receipt_delay(self):
        vehicles = [(20, pose((1, 0, 0), stamp=2)), (10, pose((0, 0, 0), stamp=1))]
        markers = [(12, pose((4, 0, 0), stamp=2))]
        result = relative_ground_truth(vehicles, markers)
        self.assertEqual(result[0][0], 12)
        self.assertEqual(result[0][1]['position_m_0'], 3.)

    def test_missing_or_invalid_truth_creates_gap(self):
        for vehicle in [[], [(10, pose((0, 0, 0), stamp=2))],
                        [(10, pose((0, 0, 0), quaternion=(0, 0, 0, 0)))]]:
            result = relative_ground_truth(vehicle, [(11, pose((2, 0, 0)))])
            self.assertEqual(list(result[0][1].values()), [None, None, None])


if __name__ == '__main__':
    unittest.main()
