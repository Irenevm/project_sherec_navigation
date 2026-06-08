"""
telemetry_logger.py

Logs flight telemetry to CSV during reactive navigation sessions.
Hooks into DroneInterfaceBase public properties (position, speed, info)
via a ROS2 timer — no modifications to aerostack2 core required.

CSV is written to /root/sherec_nav/ inside the container, which maps
to the project_sherec_navigation/ folder on the host via Docker volume.
"""

import csv
import math
import os
import threading
from datetime import datetime


class TelemetryLogger:
    """
    10 Hz flight telemetry recorder.

    Usage:
        logger = TelemetryLogger(drone)
        path = logger.start(goal_x, goal_y, goal_z, 'my_session')
        logger.mark_event('NAV_START')
        ...
        logger.mark_event('NAV_END')
        csv_path = logger.stop()   # returns path for analysis
    """

    RATE_HZ: float = 10.0

    def __init__(self, drone, output_dir: str = '/root/sherec_nav/') -> None:
        """
        :param drone: DroneInterfaceBase instance (needs .position, .speed, .info, ROS2 timer API)
        :param output_dir: directory where CSV files are stored
        """
        self._drone = drone
        self._output_dir = output_dir

        self._lock = threading.Lock()       # guards _pending_event and CSV writes
        self._pending_event: str = ''
        self._goal: tuple = (0.0, 0.0, 0.0)

        self._timer = None
        self._csv_file = None
        self._csv_writer = None
        self._file_path: str = ''
        self._running: bool = False

    # ──────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────

    def start(self, goal_x: float, goal_y: float, goal_z: float,
              session_label: str = '') -> str:
        """
        Open a new CSV file and start recording at RATE_HZ.
        If already recording, the previous session is closed first.

        :returns: absolute path of the new CSV file
        """
        if self._running:
            self.stop()

        os.makedirs(self._output_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        label_part = f'_{session_label}' if session_label else ''
        filename = f'sherec_nav_{ts}{label_part}.csv'
        self._file_path = os.path.join(self._output_dir, filename)

        self._goal = (float(goal_x), float(goal_y), float(goal_z))
        self._pending_event = ''
        self._running = True

        self._csv_file = open(self._file_path, 'w', newline='', buffering=1)
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp_s',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'speed_norm',
            'platform_state',
            'event',
            'goal_x', 'goal_y', 'goal_z',
            'dist_to_goal',
        ])

        self._timer = self._drone.create_timer(
            1.0 / self.RATE_HZ, self._tick)

        self._drone.get_logger().info(
            f'[TelemetryLogger] Recording started → {self._file_path}')
        return self._file_path

    def mark_event(self, label: str) -> None:
        """
        Tag the next CSV row with an event label.
        Thread-safe: may be called from any thread.
        """
        with self._lock:
            self._pending_event = label

    def stop(self) -> str:
        """
        Stop recording, flush and close the CSV.
        Safe to call from any thread (including a monitoring background thread).

        :returns: absolute path of the saved CSV, or '' if not recording
        """
        with self._lock:
            if not self._running:
                return self._file_path
            self._running = False

        # Cancel the ROS2 timer (safe to call from outside the callback)
        if self._timer is not None:
            self._timer.cancel()
            try:
                self._drone.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None

        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

        self._drone.get_logger().info(
            f'[TelemetryLogger] Recording saved → {self._file_path}')
        return self._file_path

    # ──────────────────────────────────────────────────────────────────────
    # Internal timer callback  (runs inside the ROS2 executor thread)
    # ──────────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        with self._lock:
            if not self._running:
                return

            # Consume the pending event label (one-shot per tick)
            event = self._pending_event
            self._pending_event = ''

            try:
                pos = self._drone.position      # [x, y, z]  m
                vel = self._drone.speed         # [vx, vy, vz]  m/s
                info = self._drone.info         # dict with 'state' key
            except Exception as exc:
                self._drone.get_logger().warn(
                    f'[TelemetryLogger] Failed to read drone data: {exc}')
                return

            speed_norm = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
            gx, gy, gz = self._goal
            dist_to_goal = math.sqrt(
                (pos[0] - gx) ** 2 +
                (pos[1] - gy) ** 2 +
                (pos[2] - gz) ** 2
            )

            # Use ROS clock so timestamp matches sim-time when use_sim_time=True
            timestamp_s = (
                self._drone.get_clock().now().nanoseconds * 1e-9
            )

            self._csv_writer.writerow([
                f'{timestamp_s:.6f}',
                f'{pos[0]:.4f}', f'{pos[1]:.4f}', f'{pos[2]:.4f}',
                f'{vel[0]:.4f}', f'{vel[1]:.4f}', f'{vel[2]:.4f}',
                f'{speed_norm:.4f}',
                info.get('state', ''),
                event,
                f'{gx:.4f}', f'{gy:.4f}', f'{gz:.4f}',
                f'{dist_to_goal:.4f}',
            ])
