"""The filter that decides whether a detection may release the payload.

No camera and no servo: these are the pure decisions, which is the part that
must not be wrong. The boxes marked "from the bench log" are real outliers the
IMX500 produced during a run on the aircraft's own Pi.
"""

import pytest

from pad_drop_test import centre_of, plausible, settled


@pytest.mark.parametrize("box, reason", [
    ((0.40, 0.40, 0.60, 0.60), "square and central"),
    ((0.62, 0.05, 0.80, 0.22), "small, off to one side, still square"),
    ((0.10, 0.10, 0.88, 0.88), "nearly fills the frame without touching it"),
])
def test_accepts_pad_shaped_boxes(box, reason):
    ok, why = plausible(box)
    assert ok, f"{reason} should be accepted, got {why!r}"


@pytest.mark.parametrize("box, expect_in_reason", [
    ((0.000, 0.118, 0.133, 0.447), "edge"),      # from the bench log
    ((0.947, 0.026, 0.994, 0.078), "edge"),      # from the bench log
    ((0.30, 0.45, 0.90, 0.52), "aspect"),        # 1:8, no pad makes that shape
    ((0.40, 0.40, 0.40, 0.60), "empty"),         # zero width
    ((0.40, 0.60, 0.60, 0.40), "empty"),         # inverted corners
])
def test_rejects_impossible_boxes(box, expect_in_reason):
    ok, why = plausible(box)
    assert not ok
    assert expect_in_reason in why


def test_aspect_limit_is_inclusive_either_way():
    """A pad rotated 90 degrees is the same pad, so width and height are symmetric."""
    wide = (0.10, 0.45, 0.70, 0.55)
    tall = (0.45, 0.10, 0.55, 0.70)
    assert plausible(wide)[0] == plausible(tall)[0]


def test_settled_accepts_a_tracked_pad():
    """Consecutive frames from the bench log, where the pad barely moves."""
    assert settled([(0.637, 0.131), (0.637, 0.132), (0.638, 0.133)])


def test_settled_rejects_a_centre_that_jumps():
    """A real pad does not cross the frame and come back within three frames."""
    assert not settled([(0.63, 0.13), (0.06, 0.41), (0.64, 0.13)])


def test_settled_is_measured_against_the_first_frame():
    """A centre that drifts steadily away must not creep past the limit unnoticed."""
    creeping = [(0.10, 0.50), (0.25, 0.50), (0.40, 0.50)]
    assert not settled(creeping)


def test_centre_of_is_the_box_centre():
    assert centre_of((0.40, 0.40, 0.60, 0.60)) == (0.5, 0.5)
    assert centre_of((0.00, 0.00, 1.00, 0.50)) == (0.5, 0.25)
