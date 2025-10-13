#!/usr/bin/env python3
"""
AprilTag Tracker and Follower - Robot tracks and follows AprilTag
Keeps the tag centered in its camera view and maintains distance
"""

from controller import Robot, Supervisor
import numpy as np
import cv2
import sys
import math
import os  # <-- added

# policy utils (logging / model)  <-- added
from common_policy import init_logger, log_row, features_from_detection, load_policy

# Add the AprilTag library to path (harmless if unused)
sys.path.insert(0, '../../../apriltag/build')

# ---- Use pupil_apriltags (macOS ARM-friendly) ----
try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
    print("AprilTag library loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import apriltag library: {e}")
    APRILTAG_AVAILABLE = False
# ---------------------------------------------------

class AprilTagTrackerFollower:
    def __init__(self):
        # Initialize as Supervisor to move the cube
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.center_x = self.width / 2
        print(f"Camera initialized: {self.width}x{self.height}")
        print(f"Camera center: {self.center_x}")

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Initialize AprilTag detector
        self.detector = None
        if APRILTAG_AVAILABLE:
            try:
                self.detector = Detector(families="tag36h11")
                print("AprilTag detector initialized with tag36h11 family")
            except Exception as e:
                print(f"Could not initialize apriltag detector: {e}")

        # ---- policy mode setup (logging/inference)  <-- added
        self.policy_mode = os.environ.get("APRILTAG_POLICY", "").lower()
        self.policy_model = None
        if self.policy_mode in ("log", "1", "true"):
            init_logger()
            print("Policy logging: ON (APRILTAG_POLICY=log)")
        elif self.policy_mode == "use":
            self.policy_model = load_policy()
            if self.policy_model is None:
                print("⚠️  APRILTAG_POLICY=use set but no model found; falling back to PD.")
                self.policy_mode = ""
            else:
                print("Policy inference: ON (APRILTAG_POLICY=use)")

        # Get the AprilTag cube node for moving it
        self.cube_node = self.robot.getFromDef("MOVING_TAG")
        if self.cube_node:
            print("Got cube node for movement control")

        # Tracking parameters (rotation)
        self.max_rotation_speed = 6.0
        self.dead_zone = 10
        self.kp_rotation = 0.03
        self.kd_rotation = 0.005
        self.last_rotation_error = 0

        # Following parameters (forward/backward)
        self.target_tag_size = 150
        self.size_dead_zone = 5
        self.max_forward_speed = 6.0
        self.kp_distance = 0.4
        self.kd_distance = 0.15
        self.brake_factor = 1.5
        self.epuck_max_speed = 6.28
        self.last_distance_error = 0
        self.min_tag_size = 20
        self.max_tag_size = 200

        # Smoothing parameters
        self.smoothed_center_x = None
        self.smoothed_size = None
        self.smoothing_factor = 0.7

        # Movement pattern for cube (constant speed left/right)
        self.cube_time = 0
        self.cube_x = 0.5
        self.cube_y = 0.0
        self.cube_min_y = -0.4
        self.cube_max_y = 0.4
        self.cube_speed = 0.1
        self.cube_direction = 1

        print("\n=== AprilTag Tracker & Follower Initialized ===")
        print("Robot will track and follow AprilTag")
        print(f"Target tag size: {self.target_tag_size} pixels (maintains close distance)")
        print("Cube moves left/right at constant speed")

    def detect_apriltag(self, gray_image):
        """Detect AprilTag in image"""
        if self.detector is None:
            return None
        try:
            # Ensure contiguous uint8 buffer (required by some builds)
            gray_c = np.ascontiguousarray(gray_image)
            results = self.detector.detect(gray_c)
            if results:
                r = results[0]
                return {
                    'id': int(getattr(r, 'tag_id', 0)),
                    'center': tuple(r.center),                         # (x, y)
                    'corners': np.array(r.corners, dtype=np.float32), # 4x2
                    'margin': float(getattr(r, 'decision_margin', 0.0))
                }
        except Exception as e:
            print(f"Detection error: {e}")
        return None

    def calculate_tag_size(self, detection):
        """Calculate apparent size of tag from corners"""
        if detection is None:
            return None
        corners = np.array(detection['corners'])
        edge1 = np.linalg.norm(corners[1] - corners[0])
        edge2 = np.linalg.norm(corners[2] - corners[1])
        edge3 = np.linalg.norm(corners[3] - corners[2])
        edge4 = np.linalg.norm(corners[0] - corners[3])
        return (edge1 + edge2 + edge3 + edge4) / 4

    def calculate_control(self, detection):
        """Return (rotation_speed, forward_speed)"""
        if detection is None:
            self.last_rotation_error = 0
            self.last_distance_error = 0
            self.smoothed_center_x = None
            self.smoothed_size = None
            return 0, 0

        # === ROTATION ===
        tag_center_x = detection['center'][0]
        self.smoothed_center_x = (tag_center_x if self.smoothed_center_x is None
                                  else self.smoothing_factor * tag_center_x
                                       + (1 - self.smoothing_factor) * self.smoothed_center_x)
        tag_center_x = self.smoothed_center_x
        rotation_error = tag_center_x - self.center_x

        if abs(rotation_error) < self.dead_zone:
            rotation_speed = self.kp_rotation * rotation_error * 0.3
        else:
            error_derivative = rotation_error - self.last_rotation_error
            rotation_speed = (self.kp_rotation * rotation_error +
                              self.kd_rotation * error_derivative)
        self.last_rotation_error = rotation_error
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # === DISTANCE ===
        tag_size = self.calculate_tag_size(detection)
        if tag_size is None:
            return rotation_speed, 0

        self.smoothed_size = (tag_size if self.smoothed_size is None
                              else self.smoothing_factor * tag_size
                                   + (1 - self.smoothing_factor) * self.smoothed_size)
        distance_error = self.smoothed_size - self.target_tag_size
        error_derivative = distance_error - self.last_distance_error
        changing_direction = (distance_error * self.last_distance_error < 0)

        if abs(distance_error) < self.size_dead_zone:
            forward_speed = -(self.kp_distance * distance_error * 0.2 +
                              self.kd_distance * error_derivative * 2.0)
        else:
            forward_speed = -(self.kp_distance * distance_error +
                              self.kd_distance * error_derivative)
            if changing_direction and abs(error_derivative) > 5:
                forward_speed *= self.brake_factor

        self.last_distance_error = distance_error
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)

        # Combined safety cap
        max_combined = abs(forward_speed) + abs(rotation_speed)
        if max_combined > self.epuck_max_speed:
            scale = self.epuck_max_speed / max_combined * 0.95
            forward_speed *= scale
            rotation_speed *= scale

        return rotation_speed, forward_speed

    def apply_motor_speeds(self, rotation_speed, forward_speed):
        left_speed = forward_speed + rotation_speed
        right_speed = forward_speed - rotation_speed

        if abs(forward_speed) > 0.2:
            if 0.1 < abs(left_speed) < 0.8:
                left_speed = 0.8 if left_speed > 0 else -0.8
            if 0.1 < abs(right_speed) < 0.8:
                right_speed = 0.8 if right_speed > 0 else -0.8

        left_speed = np.clip(left_speed, -self.epuck_max_speed, self.epuck_max_speed)
        right_speed = np.clip(right_speed, -self.epuck_max_speed, self.epuck_max_speed)

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def move_cube(self):
        if self.cube_node:
            dt = self.timestep / 1000.0
            self.cube_time += dt
            self.cube_y += self.cube_direction * self.cube_speed * dt

            if self.cube_y >= self.cube_max_y:
                self.cube_y = self.cube_max_y
                self.cube_direction = -1
                print(f"\n[CUBE] Hit right boundary {self.cube_max_y:.2f}m, reversing to left")
            elif self.cube_y <= self.cube_min_y:
                self.cube_y = self.cube_min_y
                self.cube_direction = 1
                print(f"\n[CUBE] Hit left boundary {self.cube_min_y:.2f}m, reversing to right")

            x = self.cube_x
            z = 0.0275
            if int(self.cube_time * 10) % 20 == 0 and int((self.cube_time - dt) * 10) % 20 != 0:
                direction_str = "right" if self.cube_direction > 0 else "left"
                print(f"\n[CUBE] Position Y: {self.cube_y:.3f}m, Moving: {direction_str} at {self.cube_speed:.3f}m/s")
            translation_field = self.cube_node.getField("translation")
            if translation_field:
                translation_field.setSFVec3f([x, self.cube_y, z])

    def run(self):
        step = 0
        last_detection = None

        while self.robot.step(self.timestep) != -1:
            step += 1
            self.move_cube()

            image_data = self.camera.getImage()
            if not image_data:
                continue

            image = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

            detection = self.detect_apriltag(gray)

            if detection:
                rotation_speed, forward_speed = self.calculate_control(detection)

                # ---- policy branch (logging/inference)  <-- added
                # Build features once if needed
                if self.policy_mode in ("log", "1", "true") or (self.policy_mode == "use" and self.policy_model is not None):
                    feat = features_from_detection(self, detection)

                if self.policy_mode == "use" and self.policy_model is not None:
                    # Use learned policy to set wheel speeds directly
                    left_speed, right_speed = self.policy_model.predict([feat])[0]
                    left_speed  = float(np.clip(left_speed,  -self.epuck_max_speed, self.epuck_max_speed))
                    right_speed = float(np.clip(right_speed, -self.epuck_max_speed, self.epuck_max_speed))
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                    # skip PD application/printing to avoid mixing policies
                    last_detection = detection
                    continue
                elif self.policy_mode in ("log", "1", "true"):
                    # Log the PD action that we're about to take
                    left_log  = float(np.clip(forward_speed + rotation_speed, -self.epuck_max_speed, self.epuck_max_speed))
                    right_log = float(np.clip(forward_speed - rotation_speed, -self.epuck_max_speed, self.epuck_max_speed))
                    t = self.robot.getTime()
                    err_x, err_x_d, size_err, size_err_d = feat
                    log_row(t, err_x, err_x_d, size_err, size_err_d,
                            rotation_speed, forward_speed, left_log, right_log)
                # ---- end policy branch

                self.apply_motor_speeds(rotation_speed, forward_speed)

                if step % 20 == 0:
                    raw_x = detection['center'][0]
                    smooth_x = self.smoothed_center_x if self.smoothed_center_x is not None else raw_x
                    rotation_error = smooth_x - self.center_x
                    raw_size = self.calculate_tag_size(detection)
                    smooth_size = self.smoothed_size if self.smoothed_size is not None else raw_size
                    left_speed = forward_speed + rotation_speed
                    right_speed = forward_speed - rotation_speed

                    print(f"[Step {step}] Tag ID: {detection['id']}")
                    print(f"  Tag Size: {smooth_size:.1f} / Target: {self.target_tag_size}")
                    print(f"  Forward Speed: {forward_speed:.2f}, Rotation: {rotation_speed:.2f}")
                    print(f"  Motor L: {left_speed:.2f}, R: {right_speed:.2f}")
                    if abs(forward_speed) > 0.2:
                        print("  --> Moving FORWARD (tag too small/far)" if forward_speed > 0
                              else "  --> Moving BACKWARD (tag too large/close)")

            else:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                if step % 50 == 0 and last_detection:
                    print(f"[Step {step}] Lost tracking - stopping")
                    last_detection = None

            # Optional visualization every 100 steps
            if detection and step % 100 == 0:
                vis_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                corners = np.array(detection['corners'], dtype=np.int32)
                cv2.polylines(vis_img, [corners], True, (0, 255, 0), 2)
                center = tuple(map(int, detection['center']))
                cv2.circle(vis_img, center, 5, (0, 0, 255), -1)
                cv2.line(vis_img, (int(self.center_x), 0),
                         (int(self.center_x), self.height), (255, 0, 0), 1)
                if self.smoothed_size:
                    y_pos = self.height // 2 + 50
                    x_start = int(self.center_x - self.smoothed_size / 2)
                    x_end = int(self.center_x + self.smoothed_size / 2)
                    cv2.line(vis_img, (x_start, y_pos), (x_end, y_pos), (0, 255, 255), 2)
                    x_start_target = int(self.center_x - self.target_tag_size / 2)
                    x_end_target = int(self.center_x + self.target_tag_size / 2)
                    cv2.line(vis_img, (x_start_target, y_pos + 10),
                             (x_end_target, y_pos + 10), (255, 255, 0), 1)
                cv2.putText(vis_img, f"Rot: {rotation_speed:.2f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(vis_img, f"Fwd: {forward_speed:.2f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if self.smoothed_size:
                    cv2.putText(vis_img, f"Size: {self.smoothed_size:.0f}/{self.target_tag_size}",
                                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                filename = f"tracking_follow_{step:06d}.png"
                cv2.imwrite(filename, vis_img)
                print(f"  Saved: {filename}")

            last_detection = detection

if __name__ == "__main__":
    tracker = AprilTagTrackerFollower()
    tracker.run()
