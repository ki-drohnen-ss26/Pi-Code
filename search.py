"""
search.py
=========
Search patterns for the indoor delivery: the target (landing pad) position is not
known in advance, so the drone flies a pattern and looks for it with the camera.

A pattern produces a list of (north, east) waypoints in **local NED metres relative
to the launch point**. The mission flies them in order (via Drone.goto_local) and
polls the camera. The two patterns:

    ExpandingSquare  - spirals outward from launch; needs only a step size and a max
                       radius, not the hall dimensions. Default.
    Lawnmower        - back-and-forth over a known rectangle; even full coverage.

Selected by name through Config (see make_search_pattern).
"""

from typing import List, Protocol, Tuple

from config import Config

Waypoint = Tuple[float, float]  # (north, east) in metres relative to launch


class SearchPattern(Protocol):
    """A search pattern yields waypoints to visit, in order."""

    def waypoints(self) -> List[Waypoint]:
        ...


class ExpandingSquare:
    """Expanding-square spiral around the launch point (0, 0).

    Legs go East, North, West, South (turning left), with the leg length growing
    every two legs: 1, 1, 2, 2, 3, 3, ... times `step`. Generation stops at the FIRST
    waypoint beyond `max_radius` (Chebyshev distance) - and that waypoint is still
    included, so the pattern reaches up to one `step` PAST `max_radius`. With the
    defaults (step 1.0, max_radius 6.0) the last point is (-6.0, 7.0).

    Size `max_radius` with that overshoot in mind and against the actual hall: NO fence
    is configured indoors (`FENCE_ENABLE=0` in the flight set, owned by Mission Planner
    since 2026-08-24), so nothing stops a horizontal excursion. Even an altitude fence
    would only guard the ceiling, not the walls - the horizontal guard is software:
    failsafe.position_implausible().

    Schematic (N = up, E = right; S = launch at 0,0). One continuous path that
    spirals outward; the boxes show how each loop is `step` wider:

        +-----------------------+
        |   +---------------+   |
        |   |   +-------+   |   |
        |   |   |   S   |   |   |     fly outward, check the camera
        |   |   +-------+   |   |     at each corner, until a corner
        |   +---------------+   |     passes max_radius
        +-----------------------+
    """

    # (d_north, d_east) for East, North, West, South
    _DIRS = [(0, +1), (+1, 0), (0, -1), (-1, 0)]

    def __init__(self, step: float, max_radius: float):
        if step <= 0:
            raise ValueError("step must be > 0")
        self.step = step
        self.max_radius = max_radius

    def waypoints(self) -> List[Waypoint]:
        pts: List[Waypoint] = []
        north = east = 0.0
        i = 0
        length = 1
        while True:
            for _ in range(2):  # two legs per length increment
                dn, de = self._DIRS[i % 4]
                north += dn * self.step * length
                east += de * self.step * length
                pts.append((round(north, 3), round(east, 3)))
                if max(abs(north), abs(east)) > self.max_radius:
                    return pts
                i += 1
            length += 1


class Lawnmower:
    """Boustrophedon (back-and-forth) coverage of a rectangle whose near corner is
    the launch point. Rows are spaced by `step` along north; each row is swept east
    then the next row west, so the path is continuous.

    Schematic (N = up, E = right; S = launch at the near corner):

        S >------------------>+   row 0  (north = 0)
        +<------------------<-+   row 1  (north = step)
        +>------------------>-+   row 2  (north = 2*step)
        +<------------------<-+   ...   (rows spaced by `step`)
    """

    def __init__(self, width: float, height: float, step: float):
        if step <= 0:
            raise ValueError("step must be > 0")
        self.width = width    # east extent
        self.height = height  # north extent
        self.step = step

    def waypoints(self) -> List[Waypoint]:
        pts: List[Waypoint] = []
        rows = int(self.height / self.step) + 1
        for r in range(rows):
            north = round(r * self.step, 3)
            if r % 2 == 0:
                pts.append((north, 0.0))
                pts.append((north, round(self.width, 3)))
            else:
                pts.append((north, round(self.width, 3)))
                pts.append((north, 0.0))
        return pts


def make_search_pattern(config: Config) -> SearchPattern:
    """Build the search pattern selected in the config."""
    name = config.search_pattern.lower()
    if name == "spiral":
        return ExpandingSquare(config.search_step_m, config.search_max_radius_m)
    if name == "lawnmower":
        return Lawnmower(config.search_area_w_m, config.search_area_h_m, config.search_step_m)
    raise ValueError(f"Unknown search_pattern '{config.search_pattern}' (use 'spiral' or 'lawnmower')")
