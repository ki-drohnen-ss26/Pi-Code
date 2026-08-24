"""
release.py
==========
The payload release mechanism, hidden behind a small protocol so the mission does
NOT care where the drop servo is physically wired. This mirrors the Camera /
SearchPattern protocols: the state machine programs against the interface, and
config.py picks the implementation.

    FcServo  - servo on a FLIGHT-CONTROLLER output, driven over MAVLink
               (DO_SET_SERVO). Used in SITL (no real servo exists on the Mac) and
               whenever the servo hangs off an FC AUX output.
    PiServo  - servo on a RASPBERRY PI GPIO pin, driven with PWM directly from the
               Pi (gpiozero). Used indoors where the drop servo is wired to the Pi,
               so the flight controller is not involved in the release at all.

Both expose the same four calls the mission uses:
    setup()   - one-time init (FC: SERVOx_FUNCTION=0; Pi: attach the pin, go neutral)
    reset()   - hatch closed (neutral position)
    drop()    - hatch open  (release position)
    confirm() - True if the release is believed to have happened
"""

import logging
from typing import Protocol

log = logging.getLogger(__name__)


class ReleaseMechanism(Protocol):
    """Interface the mission programs against. FcServo and PiServo satisfy it."""

    def setup(self) -> None: ...
    def reset(self) -> None: ...
    def drop(self) -> None: ...
    def confirm(self) -> bool: ...


class FcServo:
    """Drop servo on a flight-controller output, commanded over MAVLink. Delegates
    to the Drone's servo helpers, so the FC-side MAVLink code stays in drone.py.
    This is the SITL / test path (the sim just echoes the servo value back)."""

    def __init__(self, drone, config):
        self._drone = drone
        self._c = config

    def setup(self) -> None:
        # SERVOx_FUNCTION = 0 ("Disabled") so the FC lets DO_SET_SERVO through.
        # Note this write is PERMANENT on the FC (deliberately not restored: the output
        # must stay MAVLink-controlled across runs so the hatch can be re-armed). On a
        # real FC-servo build, check SERVOx_FUNCTION was not carrying another function
        # before pointing the drop mechanism at it.
        self._drone.configure_drop_servo()

    def reset(self) -> None:
        self._drone.reset_servo()

    def drop(self) -> None:
        self._drone.drop()

    def confirm(self) -> bool:
        """Read the servo value back from the FC telemetry and compare."""
        value = self._drone.read_servo(expected=self._c.drop_pwm)
        return value is not None and abs(value - self._c.drop_pwm) <= 50


class PiServo:
    """Drop servo wired directly to a Raspberry Pi GPIO pin. The Pi generates the
    ~50 Hz servo PWM itself (via gpiozero); the flight controller is not involved.

    HARDWARE (important):
      * Power the servo from a SEPARATE 5 V BEC, NOT the Pi's 5 V pin - a servo's
        stall/inrush current can brown out a Pi Zero 2 W and reboot it mid-flight.
      * Only the SIGNAL wire goes to the GPIO pin (`config.drop_gpio_pin`, BCM
        numbering; default 18 = physical pin 12), with a COMMON GROUND between the
        servo's BEC and the Pi.
      * Install with `sudo apt install python3-gpiozero python3-lgpio`. gpiozero then
        selects LGPIOFactory by default and drives the pin through the kernel's GPIO
        character device - no daemon, no extra configuration.
        DO NOT install pigpio or export `GPIOZERO_PIN_FACTORY=pigpio`. The pigpio
        package was REMOVED from Debian 13 (trixie), which our Raspberry Pi OS is
        based on: `apt install pigpio` fails outright, and forcing that pin factory
        makes Servo() raise BadPinFactory - inside setup(), which the mission calls
        from IDLE, so the aircraft would die on the ground before ever arming.
        Older guides still recommend pigpio; they predate trixie.

    The release is OPEN-LOOP (a GPIO output has no read-back), so confirm() trusts
    the commanded pulse.
    """

    # gpiozero maps value in [-1, 1] onto [min_pulse_width, max_pulse_width].
    # We use the standard 1000-2000 us servo band and map the config's PWM values.
    _MIN_US = 1000.0
    _MAX_US = 2000.0

    def __init__(self, config):
        self._c = config
        self._servo = None  # gpiozero.Servo, created lazily in setup()

    def setup(self) -> None:
        try:
            from gpiozero import Servo
        except ImportError as exc:  # not on a Pi / lib missing
            raise RuntimeError(
                "PiServo needs gpiozero (Raspberry Pi only). Install it on the Pi: "
                "sudo apt install python3-gpiozero python3-lgpio. "
                "For simulation run `python main.py --sim`, which selects "
                "release_mechanism='fc'."
            ) from exc

        self._servo = Servo(
            self._c.drop_gpio_pin,
            min_pulse_width=self._MIN_US / 1e6,
            max_pulse_width=self._MAX_US / 1e6,
            frame_width=20 / 1e3,  # 20 ms period = 50 Hz
        )
        log.info(f"[DROP] PiServo on GPIO{self._c.drop_gpio_pin} (BCM)")
        self.reset()

    def reset(self) -> None:
        self._servo.value = self._pw_to_value(self._c.neutral_pwm)
        log.info(f"[DROP] Servo neutral ({self._c.neutral_pwm} us)")

    def drop(self) -> None:
        self._servo.value = self._pw_to_value(self._c.drop_pwm)
        log.info(f"[DROP] Servo release ({self._c.drop_pwm} us)")

    def confirm(self) -> bool:
        return True  # open-loop: no read-back on a GPIO output

    @classmethod
    def _pw_to_value(cls, pw_us: float) -> float:
        """Map a pulse width in microseconds onto gpiozero's [-1, 1], clamped."""
        span = (cls._MAX_US - cls._MIN_US) / 2.0     # 500 us
        mid = (cls._MAX_US + cls._MIN_US) / 2.0      # 1500 us
        return max(-1.0, min(1.0, (pw_us - mid) / span))
