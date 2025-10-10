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

# Add the AprilTag library to path
sys.path.insert(0, '../../../apriltag/build')

try:
    import apriltag
    APRILTAG_AVAILABLE = True
    print("AprilTag library loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import apriltag library: {e}")
    APRILTAG_AVAILABLE = False

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
                self.detector = apriltag.apriltag("tag36h11")
                print("AprilTag detector initialized with tag36h11 family")
            except Exception as e:
                print(f"Could not initialize apriltag detector: {e}")

        # Get the AprilTag cube node for moving it
        self.cube_node = self.robot.getFromDef("MOVING_TAG")
        if self.cube_node:
            print("Got cube node for movement control")

        # Tracking parameters (rotation)
        self.max_rotation_speed = 6.0  # Maximum rotation speed (near e-puck max)
        self.dead_zone = 10  # Pixels from center to consider "centered"
        self.kp_rotation = 0.03  # Proportional gain for tracking (much faster response)
        self.kd_rotation = 0.005  # Derivative gain for smoother tracking
        self.last_rotation_error = 0

        # Following parameters (forward/backward)
        self.target_tag_size = 150  # Target apparent size in pixels (close but not too close)
        self.size_dead_zone = 5  # Size tolerance (larger for stability at close range)
        self.max_forward_speed = 6.0  # Maximum forward/backward speed (near e-puck max)
        self.kp_distance = 0.4  # Proportional gain for distance (aggressive)
        self.kd_distance = 0.15  # Derivative gain for braking/momentum compensation
        self.brake_factor = 1.5  # Multiplier for opposite direction when stopping
        self.epuck_max_speed = 6.28  # E-puck motor maximum (rad/s)
        self.last_distance_error = 0
        self.min_tag_size = 20  # Minimum size to consider valid
        self.max_tag_size = 200  # Maximum size to consider valid

        # Smoothing parameters
        self.smoothed_center_x = None  # For exponential moving average
        self.smoothed_size = None  # For size smoothing
        self.smoothing_factor = 0.7  # 0.7 = use 70% of new value, 30% of old

        # Movement pattern for cube (constant speed left/right)
        self.cube_time = 0
        self.cube_x = 0.5  # Keep constant forward distance
        self.cube_y = 0.0  # Current lateral position
        self.cube_min_y = -0.4  # Minimum y position (left)
        self.cube_max_y = 0.4  # Maximum y position (right)
        self.cube_speed = 0.1  # Constant speed in m/s
        self.cube_direction = 1  # 1 = right, -1 = left

        print("\n=== AprilTag Tracker & Follower Initialized ===")
        print("Robot will track and follow AprilTag")
        print(f"Target tag size: {self.target_tag_size} pixels (maintains close distance)")
        print("Cube moves left/right at constant speed")

    def detect_apriltag(self, gray_image):
        """Detect AprilTag in image"""
        if self.detector is None:
            return None

        try:
            detections = self.detector.detect(gray_image)
            if len(detections) > 0:
                # Return first detection
                det = detections[0]
                return {
                    'id': det['id'],
                    'center': det['center'],
                    'corners': det['lb-rb-rt-lt'],
                    'margin': det['margin']
                }
        except Exception as e:
            print(f"Detection error: {e}")

        return None

    def calculate_tag_size(self, detection):
        """Calculate apparent size of tag from corners"""
        if detection is None:
            return None

        corners = np.array(detection['corners'])
        # Calculate average edge length
        edge1 = np.linalg.norm(corners[1] - corners[0])  # Bottom edge
        edge2 = np.linalg.norm(corners[2] - corners[1])  # Right edge
        edge3 = np.linalg.norm(corners[3] - corners[2])  # Top edge
        edge4 = np.linalg.norm(corners[0] - corners[3])  # Left edge

        avg_size = (edge1 + edge2 + edge3 + edge4) / 4
        return avg_size

    def calculate_control(self, detection):
        """
        Calculate both rotation and forward speeds to track and follow tag
        Returns (rotation_speed, forward_speed)
        """
        if detection is None:
            # No detection - stop smoothly
            self.last_rotation_error = 0
            self.last_distance_error = 0
            self.smoothed_center_x = None
            self.smoothed_size = None
            return 0, 0

        # === ROTATION CONTROL ===
        # Get horizontal position of tag center
        tag_center_x = detection['center'][0]

        # Apply exponential moving average smoothing
        if self.smoothed_center_x is None:
            self.smoothed_center_x = tag_center_x
        else:
            self.smoothed_center_x = (self.smoothing_factor * tag_center_x +
                                     (1 - self.smoothing_factor) * self.smoothed_center_x)

        # Use smoothed position for control
        tag_center_x = self.smoothed_center_x

        # Calculate rotation error from camera center
        rotation_error = tag_center_x - self.center_x

        # Rotation control (PD)
        if abs(rotation_error) < self.dead_zone:
            rotation_speed = self.kp_rotation * rotation_error * 0.3  # Small correction
        else:
            error_derivative = rotation_error - self.last_rotation_error
            rotation_speed = (self.kp_rotation * rotation_error +
                            self.kd_rotation * error_derivative)

        self.last_rotation_error = rotation_error
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # === DISTANCE CONTROL ===
        # Calculate apparent size
        tag_size = self.calculate_tag_size(detection)

        if tag_size is None:
            return rotation_speed, 0

        # Smooth the size measurement
        if self.smoothed_size is None:
            self.smoothed_size = tag_size
        else:
            self.smoothed_size = (self.smoothing_factor * tag_size +
                                 (1 - self.smoothing_factor) * self.smoothed_size)

        # Calculate distance error (negative = too far, positive = too close)
        distance_error = self.smoothed_size - self.target_tag_size

        # Distance control (PD) - INVERTED LOGIC
        # If tag is SMALLER than target, move FORWARD (negative error -> negative speed)
        # If tag is LARGER than target, move BACKWARD (positive error -> positive speed)

        error_derivative = distance_error - self.last_distance_error

        # Check if we're changing direction (for aggressive braking)
        changing_direction = (distance_error * self.last_distance_error < 0)

        if abs(distance_error) < self.size_dead_zone:
            # In dead zone - apply strong damping to stop quickly
            forward_speed = -(self.kp_distance * distance_error * 0.2 +
                            self.kd_distance * error_derivative * 2.0)
        else:
            # Normal PD control with momentum compensation
            forward_speed = -(self.kp_distance * distance_error +
                             self.kd_distance * error_derivative)

            # Aggressive braking when changing direction
            if changing_direction and abs(error_derivative) > 5:
                forward_speed *= self.brake_factor

        self.last_distance_error = distance_error
        forward_speed = np.clip(forward_speed, -self.max_forward_speed, self.max_forward_speed)

        # Final safety check - ensure combined speeds won't exceed motor limits
        # This accounts for when rotation and forward speeds combine
        max_combined = abs(forward_speed) + abs(rotation_speed)
        if max_combined > self.epuck_max_speed:
            # Scale both speeds proportionally
            scale = self.epuck_max_speed / max_combined * 0.95  # 95% to leave margin
            forward_speed *= scale
            rotation_speed *= scale

        return rotation_speed, forward_speed

    def apply_motor_speeds(self, rotation_speed, forward_speed):
        """Apply combined rotation and forward speeds to motors"""
        # Differential drive: combine rotation and forward motion
        left_speed = forward_speed + rotation_speed
        right_speed = forward_speed - rotation_speed

        # Ensure minimum movement threshold for responsiveness
        # But allow complete stops (don't force minimum when very close to zero)
        if abs(forward_speed) > 0.2:  # Only apply minimum if we're trying to move
            if abs(left_speed) < 0.8 and abs(left_speed) > 0.1:
                left_speed = 0.8 if left_speed > 0 else -0.8
            if abs(right_speed) < 0.8 and abs(right_speed) > 0.1:
                right_speed = 0.8 if right_speed > 0 else -0.8

        # Cap to e-puck maximum speed
        left_speed = np.clip(left_speed, -self.epuck_max_speed, self.epuck_max_speed)
        right_speed = np.clip(right_speed, -self.epuck_max_speed, self.epuck_max_speed)

        # Apply to motors
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def move_cube(self):
        """Move the AprilTag cube with constant speed left and right"""
        if self.cube_node:
            # Update time
            dt = self.timestep / 1000.0  # Convert to seconds
            self.cube_time += dt

            # Move cube at constant speed laterally
            self.cube_y += self.cube_direction * self.cube_speed * dt

            # Check boundaries and reverse direction instantly
            if self.cube_y >= self.cube_max_y:
                self.cube_y = self.cube_max_y
                self.cube_direction = -1
                print(f"\n[CUBE] Hit right boundary {self.cube_max_y:.2f}m, reversing to left")
            elif self.cube_y <= self.cube_min_y:
                self.cube_y = self.cube_min_y
                self.cube_direction = 1
                print(f"\n[CUBE] Hit left boundary {self.cube_min_y:.2f}m, reversing to right")

            x = self.cube_x  # Keep constant forward distance
            z = 0.0275  # Keep at same height (half of cube size)

            # Debug cube position every 2 seconds
            if int(self.cube_time * 10) % 20 == 0 and int((self.cube_time - dt) * 10) % 20 != 0:
                direction_str = "right" if self.cube_direction > 0 else "left"
                print(f"\n[CUBE] Position Y: {self.cube_y:.3f}m, Moving: {direction_str} at {self.cube_speed:.3f}m/s")

            # Set position
            translation_field = self.cube_node.getField("translation")
            if translation_field:
                translation_field.setSFVec3f([x, self.cube_y, z])

    def run(self):
        """Main control loop"""
        step = 0
        last_detection = None

        while self.robot.step(self.timestep) != -1:
            step += 1

            # Move the cube
            self.move_cube()

            # Process camera image
            image_data = self.camera.getImage()
            if image_data:
                # Convert to numpy array
                image = np.frombuffer(image_data, np.uint8).reshape(
                    (self.height, self.width, 4))

                # Convert to grayscale
                gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

                # Detect AprilTag
                detection = self.detect_apriltag(gray)

                if detection:
                    # Calculate control signals
                    rotation_speed, forward_speed = self.calculate_control(detection)

                    # Apply to motors
                    self.apply_motor_speeds(rotation_speed, forward_speed)

                    # Print status
                    if step % 20 == 0:  # Every 20 steps
                        raw_x = detection['center'][0]
                        smooth_x = self.smoothed_center_x if self.smoothed_center_x else raw_x
                        rotation_error = smooth_x - self.center_x

                        raw_size = self.calculate_tag_size(detection)
                        smooth_size = self.smoothed_size if self.smoothed_size else raw_size
                        distance_error = smooth_size - self.target_tag_size

                        # Calculate actual motor speeds
                        left_speed = forward_speed + rotation_speed
                        right_speed = forward_speed - rotation_speed

                        print(f"[Step {step}] Tag ID: {detection['id']}")
                        print(f"  Tag Size: {smooth_size:.1f} / Target: {self.target_tag_size}")
                        print(f"  Forward Speed: {forward_speed:.2f}, Rotation: {rotation_speed:.2f}")
                        print(f"  Motor L: {left_speed:.2f}, R: {right_speed:.2f}")
                        if abs(forward_speed) > 0.2:
                            if forward_speed > 0:
                                print(f"  --> Moving FORWARD (tag too small/far)")
                            else:
                                print(f"  --> Moving BACKWARD (tag too large/close)")

                    # Save visualization periodically
                    if step % 100 == 0:
                        vis_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

                        # Draw detection
                        corners = np.array(detection['corners'], dtype=np.int32)
                        cv2.polylines(vis_img, [corners], True, (0, 255, 0), 2)

                        # Draw center point
                        center = tuple(map(int, detection['center']))
                        cv2.circle(vis_img, center, 5, (0, 0, 255), -1)

                        # Draw vertical center line
                        cv2.line(vis_img, (int(self.center_x), 0),
                                (int(self.center_x), self.height), (255, 0, 0), 1)

                        # Draw horizontal size reference line
                        if self.smoothed_size:
                            y_pos = self.height // 2 + 50
                            x_start = int(self.center_x - self.smoothed_size / 2)
                            x_end = int(self.center_x + self.smoothed_size / 2)
                            cv2.line(vis_img, (x_start, y_pos), (x_end, y_pos), (0, 255, 255), 2)

                            # Draw target size reference
                            x_start_target = int(self.center_x - self.target_tag_size / 2)
                            x_end_target = int(self.center_x + self.target_tag_size / 2)
                            cv2.line(vis_img, (x_start_target, y_pos + 10),
                                   (x_end_target, y_pos + 10), (255, 255, 0), 1)

                        # Add text
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

                else:
                    # No detection - stop
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)

                    if step % 50 == 0 and last_detection:
                        print(f"[Step {step}] Lost tracking - stopping")
                        last_detection = None

if __name__ == "__main__":
    tracker = AprilTagTrackerFollower()
    tracker.run()