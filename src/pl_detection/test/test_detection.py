from array import array
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from pl_detection.aruco_tracker import to_bgr, Detector, quaternion_xyzw


@pytest.mark.parametrize('encoding,pixel,expected', [
    ('bgr8', [3, 5, 7], [3, 5, 7]), ('rgb8', [3, 5, 7], [7, 5, 3]),
    ('rgba8', [3, 5, 7, 9], [7, 5, 3]), ('bgra8', [3, 5, 7, 9], [3, 5, 7]),
    ('mono8', [17], [17, 17, 17])])
@pytest.mark.parametrize('bigendian', [0, 1])
def test_padding_and_owned_output(encoding, pixel, expected, bigendian):
    row = pixel*2 + [255, 254, 253]
    msg = SimpleNamespace(encoding=encoding, width=2, height=2, step=len(row),
                          data=array('B', row*2), is_bigendian=bigendian)
    result = to_bgr(msg)
    np.testing.assert_array_equal(result, np.tile(expected, (2, 2, 1)))
    result[:] = 0
    assert list(msg.data) == row*2


@pytest.mark.parametrize('encoding,width,height,step,data', [
    ('mono16', 1, 1, 2, [0, 0]), ('rgb8', 2, 1, 5, [0]*5),
    ('bgr8', 1, 2, 3, [0]*3), ('mono8', 0, 1, 0, [])])
def test_bad_images_rejected(encoding, width, height, step, data):
    with pytest.raises(ValueError):
        to_bgr(SimpleNamespace(encoding=encoding, width=width, height=height,
                              step=step, data=array('B', data)))


@pytest.mark.parametrize('axis', np.eye(3).tolist()+[[0, 0, 0]])
def test_quaternion_rotation(axis):
    rotation = cv2.Rodrigues(np.asarray(axis, dtype=float)*3.0)[0]
    x, y, z, w = quaternion_xyzw(rotation)
    reconstructed = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    np.testing.assert_allclose(reconstructed, rotation, atol=1e-14)


def test_known_marker_pose_and_missing_target():
    image = np.full((600, 800), 255, np.uint8)
    image[200:400, 300:500] = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(2), 0, 200)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    camera = np.array([[600., 0, 399.5], [0, 600., 299.5], [0, 0, 1.]])
    position, quaternion = Detector(2, 0, 1.).process(image.copy(), camera, np.zeros(5))
    unannotated = image.copy()
    plain_position, plain_quaternion = Detector(2, 0, 1.).process(unannotated, camera, np.zeros(5), draw=False)
    np.testing.assert_array_equal(unannotated, image)
    np.testing.assert_array_equal(plain_position, position)
    np.testing.assert_array_equal(plain_quaternion, quaternion)
    np.testing.assert_allclose(position, [0, 0, 600/199], atol=1e-6)
    assert abs(quaternion[0]) == pytest.approx(1.)
    assert Detector(2, 1, 1.).process(image.copy(), camera, np.zeros(5)) is None
    assert Detector(2, 0, 1.).process(image.copy(), None, None) is None
