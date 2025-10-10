#!/usr/bin/env python3
"""
AprilTag Tracker - Robot stays in place but rotates to track AprilTag
Keeps the tag centered in its camera view
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

class AprilTagTracker:
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

        # Motors - only for rotation
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

        # Tracking parameters
        self.max_rotation_speed = 1.5  # Maximum rotation speed (reduced to prevent overshoot)
        self.dead_zone = 10  # Pixels from center to consider "centered" (reduced)
        self.kp = 0.008  # Proportional gain for tracking (reduced for smoother motion)
        self.kd = 0.001  # Derivative gain for smoother tracking
        self.last_error = 0  # For derivative calculation

        # Smoothing parameters
        self.smoothed_center_x = None  # For exponential moving average
        self.smoothing_factor = 0.7  # 0.7 = use 70% of new value, 30% of old

        # Movement pattern for cube (simple sine wave)
        self.cube_time = 0
        self.cube_amplitude = 0.25  # Move 0.25m left and right
        self.cube_frequency = 0.08  # Oscillation frequency (even slower for smooth tracking)

        print("\n=== AprilTag Tracker Initialized ===")
        print("Robot will rotate to keep AprilTag centered in view")
        print("AprilTag cube will move left and right")

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

    def track_tag(self, detection):
        """
        Calculate rotation speed to keep tag centered
        Positive speed = turn right, Negative = turn left
        """
        if detection is None:
            # No detection - stop smoothly
            self.last_error = 0
            self.smoothed_center_x = None
            return 0

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

        # Calculate error from camera center
        error = tag_center_x - self.center_x

        # If within dead zone, reduce speed but don't stop completely
        if abs(error) < self.dead_zone:
            # Still apply small correction within dead zone
            self.last_error = error
            return self.kp * error * 0.3  # Small correction

        # PD control for smoother tracking
        error_derivative = error - self.last_error
        rotation_speed = (self.kp * error) + (self.kd * error_derivative)

        # Update last error
        self.last_error = error

        # Limit maximum rotation speed
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        return rotation_speed

    def move_cube(self):
        """Move the AprilTag cube in a sine wave pattern"""
        if self.cube_node:
            # Calculate new position
            self.cube_time += self.timestep / 1000.0  # Convert to seconds

            # Sine wave movement
            x = 0.3  # Keep constant forward distance
            y = self.cube_amplitude * math.sin(2 * math.pi * self.cube_frequency * self.cube_time)
            z = 0.0275  # Keep at same height (half of cube size)

            # Set position
            translation_field = self.cube_node.getField("translation")
            if translation_field:
                translation_field.setSFVec3f([x, y, z])

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
                    # Calculate tracking error
                    rotation_speed = self.track_tag(detection)

                    # Apply differential drive for rotation
                    # To turn left (negative speed): left wheel backward, right wheel forward
                    # To turn right (positive speed): left wheel forward, right wheel backward
                    self.left_motor.setVelocity(rotation_speed)
                    self.right_motor.setVelocity(-rotation_speed)

                    # Print status
                    if step % 20 == 0:  # Every 20 steps
                        raw_x = detection['center'][0]
                        smooth_x = self.smoothed_center_x if self.smoothed_center_x else raw_x
                        error = smooth_x - self.center_x
                        print(f"[Step {step}] Tag ID: {detection['id']}, "
                              f"X: {smooth_x:.0f}({raw_x:.0f})/{self.center_x:.0f}, Error: {error:.0f}, "
                              f"Speed: {rotation_speed:.2f}")

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

                        # Draw error arrow
                        if abs(error) > self.dead_zone:
                            arrow_start = (int(self.center_x), self.height // 2)
                            arrow_end = (int(smooth_x), self.height // 2)
                            cv2.arrowedLine(vis_img, arrow_start, arrow_end,
                                          (0, 255, 255), 2, tipLength=0.2)

                        # Add text
                        cv2.putText(vis_img, f"Error: {error:.0f}px", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.putText(vis_img, f"Speed: {rotation_speed:.2f}", (10, 60),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                        filename = f"tracking_{step:06d}.png"
                        cv2.imwrite(filename, vis_img)
                        print(f"  Saved: {filename}")

                    last_detection = detection

                else:
                    # No detection - stop rotating
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)

                    if step % 50 == 0 and last_detection:
                        print(f"[Step {step}] Lost tracking - stopping")
                        last_detection = None

if __name__ == "__main__":
    tracker = AprilTagTracker()
    tracker.run()