#!/usr/bin/env python3
"""
AprilTag Collector - Robot approaches and collects AprilTag
When close enough, the AprilTag attaches to the top of the robot
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

class AprilTagCollector:
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

        # Get nodes for position tracking
        self.robot_node = self.robot.getSelf()
        self.apriltag_node = self.robot.getFromDef("APRILTAG_CUBE")

        if self.apriltag_node:
            print("Got AprilTag cube node")

        # Collection parameters
        self.collection_distance = 0.05  # Distance in meters to collect (very close - essentially touching)
        self.has_collected = False
        self.collection_time = None

        # Movement parameters
        self.max_forward_speed = 4.0
        self.max_rotation_speed = 4.0
        self.target_tag_size = 500  # Extremely large - we want to touch it
        self.kp_rotation = 0.02
        self.kp_distance = 0.2

        # Robot dimensions (e-puck)
        self.robot_height = 0.047  # e-puck height in meters
        self.attach_height_offset = 0.073  # Height above robot center to attach cube (6cm + 1.3cm)

        print("\n=== AprilTag Collector Initialized ===")
        print("Robot will approach and collect AprilTag")
        print(f"Collection distance: {self.collection_distance}m (essentially touching)")

    def get_robot_position(self):
        """Get robot's current position"""
        if self.robot_node:
            position = self.robot_node.getPosition()
            return np.array(position[:2])  # Return only x, y
        return np.array([0, 0])

    def get_apriltag_position(self):
        """Get AprilTag's current position"""
        if self.apriltag_node:
            translation_field = self.apriltag_node.getField("translation")
            if translation_field:
                position = translation_field.getSFVec3f()
                return np.array(position[:2])  # Return only x, y
        return None

    def detect_apriltag(self, gray_image):
        """Detect AprilTag in image"""
        if self.detector is None:
            return None

        try:
            detections = self.detector.detect(gray_image)
            if len(detections) > 0:
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
        edge1 = np.linalg.norm(corners[1] - corners[0])
        edge2 = np.linalg.norm(corners[2] - corners[1])
        edge3 = np.linalg.norm(corners[3] - corners[2])
        edge4 = np.linalg.norm(corners[0] - corners[3])

        avg_size = (edge1 + edge2 + edge3 + edge4) / 4
        return avg_size

    def check_collection(self):
        """Check if robot is close enough to collect the AprilTag"""
        if self.has_collected:
            return False

        robot_pos = self.get_robot_position()
        tag_pos = self.get_apriltag_position()

        if tag_pos is not None:
            distance = np.linalg.norm(robot_pos - tag_pos)

            # Check if we're touching (accounting for robot and cube radii)
            # E-puck radius ~3.5cm, cube half-width 2.75cm, so touching at ~6cm
            if distance < self.collection_distance:
                return True
        return False

    def attach_apriltag_to_robot(self):
        """Attach the AprilTag to the top of the robot"""
        if not self.apriltag_node or not self.robot_node:
            return False

        # Get robot's current position and rotation
        robot_position = self.robot_node.getPosition()
        robot_rotation = self.robot_node.getOrientation()

        # Calculate position on top of robot
        # E-puck center is at ground level, so we need to go up
        attach_x = robot_position[0]
        attach_y = robot_position[1]
        attach_z = self.attach_height_offset  # Place on top of robot

        # Move AprilTag to robot's top
        translation_field = self.apriltag_node.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([attach_x, attach_y, attach_z])

        # Make the AprilTag semi-transparent
        children_field = self.apriltag_node.getField("children")
        if children_field:
            shape = children_field.getMFNode(0)  # Get the Shape node
            if shape:
                appearance_field = shape.getField("appearance")
                if appearance_field:
                    appearance = appearance_field.getSFNode()
                    if appearance:
                        # Set transparency (0 = opaque, 1 = fully transparent)
                        transparency_field = appearance.getField("transparency")
                        if transparency_field:
                            transparency_field.setSFFloat(0.35)  # 65% opacity = 0.35 transparency
                        else:
                            print("Could not find transparency field")

        # Mark as collected
        self.has_collected = True
        self.collection_time = self.robot.getTime()

        print(f"\n*** APRILTAG COLLECTED! ***")
        print(f"Collection time: {self.collection_time:.2f}s")
        print(f"AprilTag now attached to robot at height {attach_z:.3f}m")

        return True

    def update_attached_apriltag(self):
        """Keep the AprilTag attached to the robot as it moves"""
        if not self.has_collected or not self.apriltag_node or not self.robot_node:
            return

        # Get robot's current position
        robot_position = self.robot_node.getPosition()

        # Update AprilTag position to stay on top of robot
        attach_x = robot_position[0]
        attach_y = robot_position[1]
        attach_z = self.attach_height_offset

        translation_field = self.apriltag_node.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([attach_x, attach_y, attach_z])

    def calculate_motor_speeds(self, detection):
        """Calculate motor speeds to approach the AprilTag"""
        if detection is None:
            return 0, 0

        # Get tag position in image
        tag_center_x = detection['center'][0]
        tag_size = self.calculate_tag_size(detection)

        if tag_size is None:
            return 0, 0

        # Rotation control (center the tag)
        rotation_error = tag_center_x - self.center_x
        rotation_speed = self.kp_rotation * rotation_error
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        # Distance control (approach the tag)
        # Keep moving forward at full speed until we collect it
        # Don't slow down - we'll collect based on distance
        forward_speed = self.max_forward_speed

        return rotation_speed, forward_speed

    def apply_motor_speeds(self, rotation_speed, forward_speed):
        """Apply motor speeds"""
        left_speed = forward_speed + rotation_speed
        right_speed = forward_speed - rotation_speed

        # Clip to motor limits
        left_speed = np.clip(left_speed, -6.28, 6.28)
        right_speed = np.clip(right_speed, -6.28, 6.28)

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def run(self):
        """Main control loop"""
        step = 0
        post_collection_steps = 0
        last_known_tag_position = None
        lost_tag_counter = 0

        while self.robot.step(self.timestep) != -1:
            step += 1

            # Update attached AprilTag position if collected
            if self.has_collected:
                self.update_attached_apriltag()
                post_collection_steps += 1

                # After collection, do a victory spin!
                if post_collection_steps < 100:
                    # Spin in place to celebrate
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(-2.0)
                elif post_collection_steps == 100:
                    # Stop after spinning
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    print("\nRobot has successfully collected the AprilTag!")
                    print("AprilTag is now attached to the robot.")

                continue

            # ALWAYS check for collection based on distance
            if self.check_collection():
                self.attach_apriltag_to_robot()
                continue

            # Process camera for AprilTag detection
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
                    # Calculate motor speeds to approach
                    rotation_speed, forward_speed = self.calculate_motor_speeds(detection)

                    # Apply speeds
                    self.apply_motor_speeds(rotation_speed, forward_speed)

                    # Debug output
                    if step % 20 == 0:
                        tag_size = self.calculate_tag_size(detection)
                        robot_pos = self.get_robot_position()
                        tag_pos = self.get_apriltag_position()

                        if tag_pos is not None:
                            distance = np.linalg.norm(robot_pos - tag_pos)
                            print(f"[Step {step}] Approaching AprilTag")
                            print(f"  Distance: {distance:.3f}m, Tag size: {tag_size:.0f}px")
                            print(f"  Forward: {forward_speed:.2f}, Rotation: {rotation_speed:.2f}")

                    # Save visualization periodically
                    if step % 100 == 0:
                        vis_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

                        # Draw detection
                        corners = np.array(detection['corners'], dtype=np.int32)
                        cv2.polylines(vis_img, [corners], True, (0, 255, 0), 2)

                        # Draw center
                        center = tuple(map(int, detection['center']))
                        cv2.circle(vis_img, center, 5, (0, 0, 255), -1)

                        # Add status text
                        cv2.putText(vis_img, "Approaching...", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                        filename = f"collection_{step:06d}.png"
                        cv2.imwrite(filename, vis_img)
                        print(f"  Saved: {filename}")
                    # Remember last known position
                    last_known_tag_position = self.get_apriltag_position()
                    lost_tag_counter = 0
                else:
                    # No detection
                    lost_tag_counter += 1

                    # If we recently lost the tag (likely because it's too close)
                    if lost_tag_counter < 30 and last_known_tag_position is not None:
                        # Keep moving forward at full speed to reach the tag
                        self.left_motor.setVelocity(4.0)
                        self.right_motor.setVelocity(4.0)

                        if step % 10 == 0:
                            robot_pos = self.get_robot_position()
                            tag_pos = self.get_apriltag_position()
                            if tag_pos is not None:
                                distance = np.linalg.norm(robot_pos - tag_pos)
                                print(f"[Step {step}] Lost visual, moving forward. Distance: {distance:.3f}m")
                    else:
                        # Really lost - search by rotating
                        self.left_motor.setVelocity(1.0)
                        self.right_motor.setVelocity(-1.0)

                        if step % 50 == 0:
                            print(f"[Step {step}] Searching for AprilTag...")

if __name__ == "__main__":
    collector = AprilTagCollector()
    collector.run()