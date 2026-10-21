from types import SimpleNamespace

import numpy as np
import pytest

from search_perception.image_processing import HitStreak, image_to_bgr, pink_fraction, red_fraction, resolve_target


def test_padded_rgb_image_and_channel_order():
    message = SimpleNamespace(width=2, height=2, step=8, encoding='rgb8',
                              data=bytes([255, 0, 0, 0, 255, 0, 99, 99,
                                          0, 0, 255, 255, 255, 255, 99, 99]))
    assert image_to_bgr(message).tolist() == [[[0, 0, 255], [0, 255, 0]],
                                            [[255, 0, 0], [255, 255, 255]]]


@pytest.mark.parametrize('encoding,step,data', [('rgb8', 2, bytes(6)),
                                              ('rgb8', 6, bytes(5)),
                                              ('16UC1', 4, bytes(4))])
def test_bad_images_are_rejected(encoding, step, data):
    with pytest.raises(ValueError):
        image_to_bgr(SimpleNamespace(width=2, height=1, step=step, data=data, encoding=encoding))


def test_target_resolution_is_exact_and_detection_only_does_not_select_a_class():
    names = {0: 'person', 1: 'suitcase'}
    assert resolve_target(' Pink suitcase ', names) == (1, 'pink')
    assert resolve_target('person', names) == (0, None)
    assert resolve_target('', names) == (None, None)
    for bad in ('case', 'red suitcase', 'pink', 'suitcase person'):
        with pytest.raises(ValueError):
            resolve_target(bad, names)


def test_red_gray_and_empty_crops_do_not_match_pink():
    assert pink_fraction(np.full((4, 4, 3), [255, 0, 255], dtype=np.uint8)) == 1.0
    assert pink_fraction(np.full((4, 4, 3), [0, 0, 255], dtype=np.uint8)) == 0.0
    assert pink_fraction(np.full((4, 4, 3), 100, dtype=np.uint8)) == 0.0
    assert pink_fraction(np.empty((0, 0, 3), dtype=np.uint8)) == 0.0


def test_confirmation_needs_distinct_frames_and_misses_reset_it():
    streak = HitStreak()
    assert streak.update(True, 1, 1.0, 3.0) == 1
    assert streak.update(True, 1, 1.1, 3.0) == 1
    assert streak.update(True, 2, 1.2, 3.0) == 2
    assert streak.update(False, 3, 1.3, 3.0) == 0
    assert streak.update(True, 4, 1.4, 3.0) == 1


def test_hits_separated_by_a_long_camera_gap_do_not_confirm():
    streak = HitStreak()
    assert streak.update(True, 1, 1.0, 3.0) == 1
    assert streak.update(True, 2, 1.1, 3.0) == 2
    assert streak.update(True, 3, 5.0, 3.0) == 1


def test_red_tennis_ball_maps_to_sports_ball_class_and_requires_red():
    names = {0: 'person', 28: 'sports ball', 31: 'suitcase'}
    assert resolve_target(' Red Tennis Ball ', names) == (28, 'red')
    with pytest.raises(ValueError):
        resolve_target('red tennis ball', {0: 'person'})


def test_red_filter_accepts_both_hue_bands_and_rejects_other_colors():
    import cv2
    for hue in (0, 5, 175, 179):
        hsv = np.full((8, 8, 3), [hue, 255, 255], dtype=np.uint8)
        assert red_fraction(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)) == 1.0
    for color in ([255, 0, 0], [0, 255, 0], [255, 0, 255], [100, 100, 100]):
        assert red_fraction(np.full((8, 8, 3), color, dtype=np.uint8)) == 0.0
    assert red_fraction(np.empty((0, 0, 3), dtype=np.uint8)) == 0.0
