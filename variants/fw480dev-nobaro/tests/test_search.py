"""
test_search.py
==============
Unit tests for the search patterns (search.py). Pure geometry, no SITL.
"""

import math

from config import Config
from search import ExpandingSquare, Lawnmower, make_search_pattern


def test_expanding_square_starts_at_launch_and_spirals_out():
    pattern = ExpandingSquare(step=1.0, max_radius=6.0)
    wps = pattern.waypoints()

    assert len(wps) >= 4
    # First leg goes East by one step from the launch point.
    assert wps[0] == (0.0, 1.0)
    # Stays within (just past) the max radius and grows monotonically in reach.
    assert all(max(abs(n), abs(e)) <= 6.0 + 1.0 for n, e in wps)
    assert max(max(abs(n), abs(e)) for n, e in wps) > 2.0  # it really expanded


def test_expanding_square_visits_a_point_near_a_known_target():
    # The default sim target (2, 2) must be reachable: some corner within 1.5 m.
    wps = ExpandingSquare(step=1.0, max_radius=6.0).waypoints()
    assert any(math.hypot(2.0 - n, 2.0 - e) <= 1.5 for n, e in wps)


def test_lawnmower_covers_rectangle_in_boustrophedon():
    wps = Lawnmower(width=4.0, height=4.0, step=2.0).waypoints()

    # Rows at north = 0, 2, 4 -> 3 rows, 2 waypoints each.
    norths = sorted({n for n, _ in wps})
    assert norths == [0.0, 2.0, 4.0]
    assert len(wps) == 6
    # Row 0 sweeps east (0 -> width), row 1 sweeps back (width -> 0).
    assert wps[0] == (0.0, 0.0) and wps[1] == (0.0, 4.0)
    assert wps[2] == (2.0, 4.0) and wps[3] == (2.0, 0.0)


def test_factory_selects_pattern_by_config():
    spiral = make_search_pattern(Config(search_pattern="spiral"))
    lawn = make_search_pattern(Config(search_pattern="lawnmower"))
    assert isinstance(spiral, ExpandingSquare)
    assert isinstance(lawn, Lawnmower)
