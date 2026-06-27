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
    heartbeat_timeout_s: float = 3.0    # no FC heartbeat within this -> LINK_LOSS
    telemetry_max_misses: int = 5       # consecutive missing telemetry reads -> NO_TELEMETRY

    # ------------------------------------------------------------------
    # Target alignment (visual servoing over the target before the drop)
    # ------------------------------------------------------------------
    centre_tolerance: float = 0.15  # |dx|,|dy| below this counts as "centred"
    approach_gain: float = 0.5      # body metres to nudge per unit of image offset
    max_nudge_m: float = 0.3        # clamp for a single correction step [m]
    nudge_settle_s: float = 0.5     # wait after each nudge so the move settles

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