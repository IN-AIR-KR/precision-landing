import numpy as np
import pytest
from pl_camera import autofocus as af


def test_lens_validates_range_and_closes_on_failed_setup(monkeypatch):
    closed = []
    monkeypatch.setattr(af.os, 'open', lambda *args: 99)
    monkeypatch.setattr(af.os, 'close', closed.append)
    def fail(*args):
        raise OSError('bus unavailable')
    monkeypatch.setattr(af.fcntl, 'ioctl', fail)
    with pytest.raises(OSError):
        af.Lens()
    assert closed == [99]


def test_search_refines_peak_and_once_holds(monkeypatch):
    class Motor:
        def __init__(self, bus):
            self.position = 512
        def move(self, position):
            assert 0 <= position <= 1023
            self.position = position
        def close(self):
            pass
    monkeypatch.setattr(af, 'Lens', Motor)
    focus = af.Autofocus(mode='once')
    monkeypatch.setattr(af, 'sharpness', lambda frame: 1 / (1 + ((focus.lens.position-432)/30)**2))
    for _ in range(200):
        focus.ready_at = 0
        focus.update(None)
        if focus.completed:
            break
    assert focus.completed
    assert abs(focus.lens.position - 432) <= 8
    position = focus.lens.position
    focus.update(None)
    assert focus.lens.position == position


def test_blank_frame_restores_original_focus(monkeypatch):
    class Motor:
        position = 480
        def __init__(self, bus):
            pass
        def move(self, position):
            self.position = position
        def close(self):
            pass
    monkeypatch.setattr(af, 'Lens', Motor)
    focus = af.Autofocus(mode='once')
    for _ in range(200):
        focus.ready_at = 0
        focus.update(np.zeros((60, 80), dtype=np.uint8))
        if focus.completed:
            break
    assert focus.completed
    assert focus.lens.position == 480
