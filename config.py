"""
config.py
=========
Central configuration. This holds the ONLY difference between "test against
SITL" and "real flight on the Pi": the connection_string.

Everything else (drone.py, mission.py, ...) stays exactly the same, whether SITL
or a real flight controller is behind it.
"""

from dataclasses import dataclass


@dataclass
class Config:
    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    # SITL: sim_vehicle.py outputs MAVLink on port 14550 by default. Our script
    # is the RECEIVER, so it must bind/listen -> use "udpin:".
    #   connection_string = "udpin:127.0.0.1:14550"   (SITL native on the Mac)
    #   connection_string = "udpin:0.0.0.0:14550"     (SITL in a container;
    #     listen on all interfaces, SITL sends with --out udp:<YOUR-MAC-IP>:14550)
    #
    # To run alongside QGroundControl (which takes 14550), add a second output in
    # the MAVProxy console with `output add 127.0.0.1:14551` and use port 14551.
    #
    # Real Pi on the flight controller (serial UART link):
    #   connection_string = "/dev/serial0"   and baud = 921600 (or 57600)
    #   Alternatively via USB-UART adapter: "/dev/ttyUSB0"
    connection_string: str = "udpin:127.0.0.1:14550"
    baud: int = 115200            # only used for a serial connection
    gcs_system_id: int = 255      # the ID we announce as the ground station

    # ------------------------------------------------------------------
    # Mission / target
    # ------------------------------------------------------------------
    # Default target = the SITL start point (CMAC, Canberra) slightly offset.
    # Replace with real coordinates for an actual flight.
    target_lat: float = -35.36272
    target_lon: float = 149.16523
    cruise_alt: float = 10.0       # flight altitude in metres (relative to start)
    arrival_radius_m: float = 1.5  # target counts as reached within this distance

    # ------------------------------------------------------------------
    # Payload / release
    # ------------------------------------------------------------------
    drop_servo: int = 9            # servo output (SERVO9 = AUX OUT 1 on many FCs)
    drop_pwm: int = 1900           # PWM to release (hatch open)
    neutral_pwm: int = 1100        # PWM at rest (hatch closed)

    # ------------------------------------------------------------------
    # Failsafe / safety
    # ------------------------------------------------------------------
    battery_min_voltage: float = 10.8   # abort threshold, voltage [V] (3S example)
    battery_min_percent: int = 20       # abort threshold, remaining capacity [%]
    phase_timeout_s: float = 60.0       # max duration per mission phase
    geofence_enable: bool = True        # set FENCE_ENABLE?
    # FENCE_TYPE bitmask. 1 = max altitude only — works WITHOUT a horizontal position,
    # so it is indoor-safe (a circle/polygon fence would need GPS/position). Outdoor you
    # can add the circle bit (2) / polygon bit (4).
    fence_type: int = 1
    # Size the altitude fence to the flight: indoor ~ search_altitude (default 2 m).
    # NOTE: for the GPS path raise this above cruise_alt (10 m) or it trips on takeoff.
    fence_alt_max_m: float = 4.0        # max altitude for the fence [m]
    heartbeat_timeout_s: float = 3.0    # no FC heartbeat within this -> LINK_LOSS
    telemetry_max_misses: int = 5       # consecutive missing telemetry reads -> NO_TELEMETRY

    # ------------------------------------------------------------------
    # Target alignment (visual servoing over the target before the drop)
    # ------------------------------------------------------------------
    centre_tolerance: float = 0.15  # |dx|,|dy| below this counts as "centred"
    approach_gain: float = 0.5      # body metres to nudge per unit of image offset
    max_nudge_m: float = 0.3        # clamp for a single correction step [m]
    nudge_settle_s: float = 0.5     # wait after each nudge so the move settles
    approach_lost_max: int = 5      # consecutive "target lost" frames before back to SEARCH

    # ------------------------------------------------------------------
    # Navigation mode (Phase 2 — indoor, GPS-denied)
    # ------------------------------------------------------------------
    # gps_denied=True: position comes from optical flow (MTF-01P), navigation is in
    # local NED, pre-arm waits for the RELATIVE EKF position. False = the GPS/Phase-1
    # path (global lat/lon, absolute EKF position).
    gps_denied: bool = True
    search_altitude: float = 2.0        # indoor cruise/search height [m]
    local_arrival_radius_m: float = 0.5  # local-NED waypoint reached within this [m]

    # EKF origin (Phase 3). Indoors there is no GPS to seed the EKF origin/home, so the
    # companion sends SET_GPS_GLOBAL_ORIGIN before arming. Any sensible reference works —
    # it only anchors the local NED frame + home. Enable for a truly GPS-off run; leave
    # off when SITL still has GPS (which seeds the origin itself).
    set_origin_on_start: bool = True
    # Reference origin = the real hall (from Google Maps). Indoor nav is purely relative,
    # so only the magnetic declination + map display depend on this value; using the true
    # location gives the correct compass heading. NOTE for SITL: the simulator's compass is
    # modelled at its own home (CMAC/Canberra by default), so launch sim_vehicle at the same
    # spot - sim_vehicle.py ... --custom-location=50.131196,8.692972,112,0 - or expect a
    # small yaw offset (the mission still works, the local frame is just rotated a few deg).
    origin_lat: float = 50.13119602511582   # hall latitude  [deg]
    origin_lon: float = 8.692972038286195   # hall longitude [deg]
    origin_alt: float = 112.0               # approx hall altitude [m AMSL]; indoor height comes from LiDAR

    # ------------------------------------------------------------------
    # Search pattern (Phase 2)
    # ------------------------------------------------------------------
    search_pattern: str = "spiral"   # "spiral" (expanding square, default) or "lawnmower"
    search_step_m: float = 1.0       # spacing between search legs / rings [m]
    search_max_radius_m: float = 6.0  # spiral: stop expanding beyond this [m]
    search_area_w_m: float = 6.0     # lawnmower: rectangle width (east) [m]
    search_area_h_m: float = 6.0     # lawnmower: rectangle height (north) [m]
    detection_cadence: str = "stop_and_look"  # "stop_and_look" (default) or "continuous"
    look_settle_s: float = 0.5       # hover time before detecting at a waypoint [s]

    # ------------------------------------------------------------------
    # Simulation target (SimCamera pretends the pad is here, local NED [m])
    # ------------------------------------------------------------------
    sim_target_north: float = 2.0
    sim_target_east: float = 2.0
    sim_fov_radius_m: float = 1.5    # camera "sees" the target within this ground radius

    # ------------------------------------------------------------------
    # Camera selection (which Camera implementation main.py wires in)
    # ------------------------------------------------------------------
    # "auto" = SimCamera when gps_denied else MockCamera; "sim" / "mock" force one;
    # "timed" = TimedCamera (finds the target after a set time, no real detection) to
    # flight-test the search pattern + drop without the AI camera (Phase 3).
    camera_source: str = "auto"
    timed_camera_after_s: float = 20.0  # TimedCamera: declare "found" after this many seconds

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    log_dir: str = "logs"          # directory for timestamped mission log files

    # ------------------------------------------------------------------
    # Convenience constructors
    # ------------------------------------------------------------------
    @classmethod
    def sitl(cls, host: str = "127.0.0.1", port: int = 14550) -> "Config":
        """Preset for testing against SITL. Use host=0.0.0.0 if SITL runs in a container."""
        return cls(connection_string=f"udpin:{host}:{port}")

    @classmethod
    def pi_serial(cls, device: str = "/dev/serial0", baud: int = 921600) -> "Config":
        """Preset for the real Pi on the flight controller (UART)."""
        return cls(connection_string=device, baud=baud)