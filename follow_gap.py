import math
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class FollowGap(Node):

    def __init__(self):
        super().__init__('follow_gap_controller')

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Odometry subscriber for lap tracking.
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Controller parameters
        self.max_steering = 0.42
        self.wheelbase = 0.3302
        self.lookahead_min = 0.90
        self.lookahead_max = float(self.declare_parameter('lookahead_max', 1.60).value)
        self.lookahead_base = 0.80
        self.lookahead_speed_gain = float(self.declare_parameter('lookahead_speed_gain', 0.25).value)
        self.cruise_speed = float(self.declare_parameter('cruise_speed', 2.8).value)
        self.p_lat = float(self.declare_parameter('max_lateral_accel', 2.5).value)
        self.p_acc = float(self.declare_parameter('max_acceleration', 1.5).value)
        self.p_dec = float(self.declare_parameter('max_deceleration', 2.5).value)
        self.p_comf = float(self.declare_parameter('comfortable_decel', 3.0).value)
        self.get_logger().info(f'Speed params: cruise={self.cruise_speed}, lat={self.p_lat}, acc={self.p_acc}, dec={self.p_dec}, comf={self.p_comf}, la_max={self.lookahead_max}, la_gain={self.lookahead_speed_gain}')
        self.vehicle_width = 0.30
        self.safety_margin = 0.15
        self.max_look_angle = 1.22
        self.max_processed_range = 10.0
        self.min_valid_obstacle_range = 0.10
        self.gap_threshold = 1.20

        self.last_scan_time = None
        self.last_steering = 0.0
        self.last_target_angle = 0.0
        self.last_speed = 0.0
        self.last_speed_update_time = time.monotonic()

        # Lap counter and lap timing.
        self.lap_count = 0
        self.lap_times = []
        self.last_lap_time = None
        self.fastest_lap_time = None
        self.fastest_lap_number = None
        self.lap_start_time = None

        # Start/finish gate is initialized from the first odometry pose.
        self._lap_gate_initialized = False
        self._lap_departed = False
        self._lap_origin_x = 0.0
        self._lap_origin_y = 0.0
        self._lap_origin_yaw = 0.0
        self._lap_prev_s = None
        self._lap_prev_l = None
        self._lap_prev_yaw = None
        self._lap_prev_time = None
        self._lap_last_crossing_time = None

        # Gate validation settings.
        self._lap_gate_half_width = 3.0
        self._lap_heading_tolerance = 0.8
        self._lap_departure_distance = 0.5
        self._lap_cooldown = 5.0
        # --- lap fix: gate direction from real motion ---
        self._lap_gate_ready = False
        self._lap_gate_yaw = 0.0
        self._lap_gate_setup_distance = 2.5
        self._lap_motion_threshold = 0.05

        # Display lap dashboard once per second.
        self.lap_dashboard_timer = self.create_timer(
            1.0,
            self.lap_dashboard_callback
        )

        # Stop the car if LiDAR data becomes unavailable.
        self.watchdog = self.create_timer(
            0.2,
            self.watchdog_callback
        )

        self.publish_command(0.0, 0.0)

        self.get_logger().info(
            'Follow-the-Gap controller started. Waiting for LiDAR...'
        )

    def publish_command(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'

        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)

        self.drive_pub.publish(msg)
        self.last_speed = float(speed)
        self.last_speed_update_time = time.monotonic()


    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _format_lap_time(seconds):
        if seconds is None:
            return "--:--.---"

        minutes = int(seconds // 60)
        remaining = seconds - minutes * 60

        return f"{minutes:02d}:{remaining:06.3f}"

    def odom_callback(self, msg):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x = position.x
        y = position.y

        # Convert quaternion to yaw.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            orientation.y ** 2
            + orientation.z ** 2
        )

        yaw = math.atan2(sin_yaw, cos_yaw)
        now = time.monotonic()

        # Step 1: remember the spawn position (origin of the gate).
        if not self._lap_gate_initialized:

            self._lap_origin_x = x
            self._lap_origin_y = y
            self._lap_origin_yaw = yaw

            self._lap_prev_s = 0.0
            self._lap_prev_l = 0.0
            self._lap_prev_yaw = yaw
            self._lap_prev_time = now

            # The lap timer starts only when the car really moves.
            self.lap_start_time = None
            self._lap_gate_ready = False
            self._lap_gate_yaw = yaw
            self._lap_gate_initialized = True

            self.get_logger().info(
                "Lap tracking initialized. Waiting for the car to move..."
            )

            return

        dx = x - self._lap_origin_x
        dy = y - self._lap_origin_y
        distance_from_origin = math.hypot(dx, dy)

        # Step 2: start the lap timer when the car leaves the spawn point.
        if (
            self.lap_start_time is None
            and distance_from_origin >= self._lap_motion_threshold
        ):
            self.lap_start_time = now

        # Step 3: orient the gate along the REAL driving direction
        # (the spawn yaw may not match the direction of the track).
        if not self._lap_gate_ready:

            if distance_from_origin >= self._lap_gate_setup_distance:

                self._lap_gate_yaw = math.atan2(dy, dx)
                self._lap_gate_ready = True
                self._lap_departed = True

                self.get_logger().info(
                    "Lap gate armed. Driving direction "
                    f"{math.degrees(self._lap_gate_yaw):.1f} deg "
                    f"(spawn yaw {math.degrees(self._lap_origin_yaw):.1f} deg)."
                )

                self._lap_prev_s = distance_from_origin
                self._lap_prev_l = 0.0
            else:
                self._lap_prev_s = None
                self._lap_prev_l = None

            self._lap_prev_yaw = yaw
            self._lap_prev_time = now
            return

        cos_gate = math.cos(self._lap_gate_yaw)
        sin_gate = math.sin(self._lap_gate_yaw)

        # Distance along the driving direction of the first meters.
        signed_s = dx * cos_gate + dy * sin_gate

        # Lateral distance from the start/finish gate.
        lateral = -dx * sin_gate + dy * cos_gate

        previous_s = self._lap_prev_s

        # Detect crossing from the approach side to the finish side.
        if (
            self._lap_departed
            and previous_s is not None
            and previous_s < 0.0
            and signed_s >= 0.0
        ):

            denominator = signed_s - previous_s

            if denominator > 1e-9:

                fraction = -previous_s / denominator

                crossing_lateral = (
                    self._lap_prev_l
                    + fraction * (lateral - self._lap_prev_l)
                )

                yaw_delta = self._wrap_angle(
                    yaw - self._lap_prev_yaw
                )

                crossing_yaw = self._wrap_angle(
                    self._lap_prev_yaw + fraction * yaw_delta
                )

                heading_error = self._wrap_angle(
                    crossing_yaw - self._lap_gate_yaw
                )

                crossing_time = (
                    self._lap_prev_time
                    + fraction * (now - self._lap_prev_time)
                )

                valid_lateral = (
                    abs(crossing_lateral)
                    <= self._lap_gate_half_width
                )

                valid_heading = (
                    abs(heading_error)
                    <= self._lap_heading_tolerance
                )

                cooldown_ok = (
                    self._lap_last_crossing_time is None
                    or crossing_time - self._lap_last_crossing_time
                    >= self._lap_cooldown
                )

                accepted = valid_lateral and valid_heading and cooldown_ok

                # Diagnostic: shows why a lap was counted or rejected.
                self.get_logger().info(
                    f"Gate crossing: lateral={crossing_lateral:+.2f} m "
                    f"(max {self._lap_gate_half_width:.1f}), "
                    f"heading_error={math.degrees(heading_error):+.1f} deg "
                    f"(max {math.degrees(self._lap_heading_tolerance):.0f}), "
                    f"cooldown_ok={cooldown_ok} -> "
                    f"{'COUNTED' if accepted else 'REJECTED'}"
                )

                if accepted:
                    self.record_completed_lap(crossing_time)

        self._lap_prev_s = signed_s
        self._lap_prev_l = lateral
        self._lap_prev_yaw = yaw
        self._lap_prev_time = now

    def record_completed_lap(self, crossing_time):

        if self.lap_start_time is None:
            self.lap_start_time = crossing_time
            return

        lap_time = crossing_time - self.lap_start_time

        if lap_time <= 0.0:
            return

        self.lap_count += 1
        self.last_lap_time = lap_time
        self.lap_times.append(lap_time)

        # Update fastest lap.
        if (
            self.fastest_lap_time is None
            or lap_time < self.fastest_lap_time
        ):
            self.fastest_lap_time = lap_time
            self.fastest_lap_number = self.lap_count

        self._lap_last_crossing_time = crossing_time

        # Start timing the next lap.
        self.lap_start_time = crossing_time

        # Print completed lap and full lap history.
        lines = [
            "+------+------------+",
            "| LAP  | LAP TIME   |",
            "+------+------------+",
        ]

        for index, elapsed in enumerate(self.lap_times, start=1):
            marker = " *" if index == self.fastest_lap_number else ""
            lines.append(
                f"| {index:4d} | "
                f"{self._format_lap_time(elapsed):<10} |{marker}"
            )

        lines.append("+------+------------+")
        lines.append("* = FASTEST LAP")

        self.get_logger().info(
            f"COMPLETED LAP {self.lap_count}: "
            f"{self._format_lap_time(lap_time)}\n"
            + "\n".join(lines)
        )

    def lap_dashboard_callback(self):

        if self.lap_start_time is None:
            return

        current_time = time.monotonic()

        current_lap_time = (
            current_time - self.lap_start_time
        )

        lines = [
            "",
            "+--------------------------------------+",
            "|          F1TENTH LAP DASHBOARD       |",
            "+--------------------------------------+",
            f"| Completed laps : {self.lap_count:<17}|",
            f"| Current lap    : {self.lap_count + 1:<17}|",
            f"| Lap timer      : {self._format_lap_time(current_lap_time)}     |",
            f"| Last lap       : {self._format_lap_time(self.last_lap_time)}     |",
            f"| Fastest lap    : {self._format_lap_time(self.fastest_lap_time)}     |",
            f"| Fastest lap #  : {str(self.fastest_lap_number or '--'):<17}|",
            "+--------------------------------------+",
        ]

        self.get_logger().info("\n".join(lines))

    def watchdog_callback(self):
        if self.last_scan_time is None:
            self.publish_command(0.0, 0.0)
            return

        elapsed = (
            self.get_clock().now() - self.last_scan_time
        ).nanoseconds / 1e9

        if elapsed > 0.5:
            self.publish_command(0.0, 0.0)

    def scan_callback(self, scan):

        self.last_scan_time = self.get_clock().now()

        if not scan.ranges or scan.angle_increment <= 0.0:
            self.publish_command(0.0, 0.0)
            return

        # 1. Restrict LiDAR to the forward field of view.
        start = max(
            0,
            int(math.ceil(
                (-self.max_look_angle - scan.angle_min)
                / scan.angle_increment
            ))
        )

        end = min(
            len(scan.ranges) - 1,
            int(math.floor(
                (self.max_look_angle - scan.angle_min)
                / scan.angle_increment
            ))
        )

        if end <= start:
            self.publish_command(0.0, 0.0)
            return

        # 2. Preprocess LiDAR measurements.
        processed = []

        for value in scan.ranges[start:end + 1]:

            if math.isnan(value):
                processed.append(0.0)

            elif math.isinf(value):
                processed.append(
                    min(scan.range_max, self.max_processed_range)
                )

            elif value <= 0.0:
                processed.append(0.0)

            elif value < self.min_valid_obstacle_range:
                # Mark very small readings as blocked/invalid.
                processed.append(0.0)

            else:
                processed.append(
                    min(value, scan.range_max, self.max_processed_range)
                )

        if len(processed) < 5:
            self.publish_command(0.0, 0.0)
            return

        local_angles = [
            scan.angle_min + (start + i) * scan.angle_increment
            for i in range(len(processed))
        ]

        # Measure the nearest obstacle in front of the car.
        front_distances = [
            processed[i]
            for i in range(len(processed))
            if abs(local_angles[i]) < 0.35
        ]

        front_min = (
            min(front_distances)
            if front_distances
            else 0.0
        )

        # 3. Find the closest valid obstacle.
        valid_indices = [
            i for i, distance in enumerate(processed)
            if distance > 0.0
        ]

        if not valid_indices:
            self.publish_command(0.0, 0.0)
            return

        closest_index = min(
            valid_indices,
            key=lambda i: processed[i]
        )

        closest_distance = processed[closest_index]

        # 4. Create a safety bubble around the closest obstacle.
        bubble_radius = math.atan2(
            (self.vehicle_width / 2.0) + self.safety_margin,
            max(closest_distance, 0.05)
        )

        bubble_points = int(
            math.ceil(bubble_radius / scan.angle_increment)
        )

        bubble_start = max(0, closest_index - bubble_points)
        bubble_end = min(
            len(processed) - 1,
            closest_index + bubble_points
        )

        for i in range(bubble_start, bubble_end + 1):
            processed[i] = 0.0

        # 5. Find all continuous free gaps.
        gaps = []
        i = 0

        while i < len(processed):
            if processed[i] >= self.gap_threshold:
                gap_start = i

                while i < len(processed) and processed[i] >= self.gap_threshold:
                    i += 1

                gap_end = i - 1
                gap_length = gap_end - gap_start + 1
                gaps.append((gap_start, gap_end, gap_length))
            else:
                i += 1

        if not gaps:
            self.publish_command(0.0, 0.0)
            return

        longest_length = max(gap[2] for gap in gaps)

        # Keep gaps that are at least 85% as wide as the largest one.
        candidates = [
            gap for gap in gaps
            if gap[2] >= 0.90 * longest_length
        ]

        if longest_length < 5:
            self.publish_command(0.0, 0.0)
            return

        # Select a gap using forward alignment and target continuity.
        def gap_score(gap):
            center_index = (gap[0] + gap[1]) // 2

            candidate_angle = (
                scan.angle_min
                + (start + center_index) * scan.angle_increment
            )

            continuity_cost = abs(
                candidate_angle - self.last_target_angle
            )

            forward_cost = abs(candidate_angle)

            return (
                0.80 * continuity_cost
                + 0.20 * forward_cost
            )

        best_start, best_end, best_length = min(
            candidates,
            key=gap_score
        )

        # 6. Select the center of the chosen free gap.
        target_index = (best_start + best_end) // 2

        target_angle = (
            scan.angle_min
            + (start + target_index) * scan.angle_increment
        )
        self.last_target_angle = target_angle

        # Pure Pursuit steering calculation.
        # Adaptive lookahead based on the previous commanded speed.
        lookahead_distance = (
            self.lookahead_base
            + self.lookahead_speed_gain * self.last_speed
        )

        lookahead_distance = max(
            self.lookahead_min,
            min(self.lookahead_max, lookahead_distance)
        )

        # Never aim beyond the measured free distance on this LiDAR ray.
        target_distance = min(
            lookahead_distance,
            processed[target_index]
        )
        target_distance = max(target_distance, 0.10)

        curvature = (
            2.0 * math.sin(target_angle)
            / target_distance
        )

        steering = math.atan(
            self.wheelbase * curvature
        )

        steering = max(
            -self.max_steering,
            min(self.max_steering, steering)
        )

        # Adaptive steering response with corner-exit recovery.
        corner_factor = min(abs(steering) / 0.25, 1.0)

        leaving_corner = (
            abs(target_angle) < 0.10
            and abs(self.last_steering) > 0.04
        )

        if leaving_corner:
            # Release residual steering faster after a corner.
            alpha = 0.18
            max_step = 0.035
        else:
            # Preserve smooth steering during normal driving.
            alpha = 0.08 + 0.08 * corner_factor
            max_step = 0.025 + 0.015 * corner_factor

        smoothed = (
            alpha * steering
            + (1.0 - alpha) * self.last_steering
        )
        delta = smoothed - self.last_steering
        delta = max(-max_step, min(max_step, delta))

        steering = self.last_steering + delta
        self.last_steering = steering

        # 7. Smooth Cornering Speed Control.
        desired_speed = self.cruise_speed

        # Predict corner speed using Pure Pursuit curvature.
        max_lateral_accel = self.p_lat

        if abs(curvature) > 1e-6:
            corner_speed = math.sqrt(
                max_lateral_accel / abs(curvature)
            )
            desired_speed = min(desired_speed, corner_speed)

        # Obstacle-based speed limitation.
        stop_distance = 0.35
        comfortable_decel = self.p_comf

        if front_min <= stop_distance:
            desired_speed = 0.0
        else:
            available_distance = front_min - stop_distance

            obstacle_speed = math.sqrt(
                2.0 * comfortable_decel * available_distance
            )

            desired_speed = min(desired_speed, obstacle_speed)

        # Calculate elapsed time between speed commands.
        now = time.monotonic()

        dt = max(
            0.001,
            min(now - self.last_speed_update_time, 0.1)
        )

        # Emergency stop overrides acceleration smoothing.
        if front_min <= stop_distance:
            speed = 0.0

        else:
            max_acceleration = self.p_acc
            max_deceleration = self.p_dec

            # Allow stronger braking when an obstacle is close.
            if front_min < 1.40:
                max_deceleration = max(4.0, self.p_dec, self.p_comf)

            speed_change = desired_speed - self.last_speed

            if speed_change > 0.0:
                max_change = max_acceleration * dt
            else:
                max_change = max_deceleration * dt

            speed_change = max(
                -max_change,
                min(max_change, speed_change)
            )

            speed = max(0.0, self.last_speed + speed_change)

        # 8. Publish Ackermann steering command.
        self.publish_command(steering, speed)


stop_requested = False


def request_stop(signum, frame):
    global stop_requested
    stop_requested = True


def main(args=None):
    global stop_requested
    stop_requested = False

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    node = FollowGap()

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        node.get_logger().info('Stopping vehicle...')

        for _ in range(5):
            node.publish_command(0.0, 0.0)
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
