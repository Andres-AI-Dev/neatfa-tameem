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

# Add the AprilTag library to path (harmless if unused)
sys.path.insert(0, '../../../apriltag/build')

# ---- MINIMAL CHANGE: import pupil_apriltags Detector ----
try:
    from pupil_apriltags import Detector as _Detector
    APRILTAG_AVAILABLE = True
    print("AprilTag library (pupil_apriltags) loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import pupil_apriltags library: {e}")
    APRILTAG_AVAILABLE = False
# ---------------------------------------------------------

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
                # MINIMAL CHANGE: construct pupil_apriltags detector for tag36h11
                self.detector = _Detector(families="tag36h11")
                print("AprilTag detector initialized with tag36h11 family (pupil_apriltags)")
            except Exception as e:
                print(f"Could not initialize pupil_apriltags detector: {e}")

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
            # MINIMAL CHANGE: use pupil_apriltags Detector and map to your dict format
            detections = self.detector.detect(gray_image)
            if len(detections) > 0:
                det = detections[0]
                return {
                    'id': getattr(det, 'tag_id', 0),
                    'center': det.center,       # (x, y)
                    'corners': det.corners,     # 4x2 array
                    'margin': 0                 # placeholder to keep existing code happy
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
            if distance < self.collection_distance:
                return True
        return False

    def attach_apriltag_to_robot(self):
        """Attach the AprilTag to the top of the robot"""
        if not self.apriltag_node or not self.robot_node:
            return False

        robot_position = self.robot_node.getPosition()

        attach_x = robot_position[0]
        attach_y = robot_position[1]
        attach_z = self.attach_height_offset  # Place on top of robot

        translation_field = self.apriltag_node.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([attach_x, attach_y, attach_z])

        children_field = self.apriltag_node.getField("children")
        if children_field:
            shape = children_field.getMFNode(0)
            if shape:
                appearance_field = shape.getField("appearance")
                if appearance_field:
                    appearance = appearance_field.getSFNode()
                    if appearance:
                        transparency_field = appearance.getField("transparency")
                        if transparency_field:
                            transparency_field.setSFFloat(0.35)

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

        robot_position = self.robot_node.getPosition()
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

        tag_center_x = detection['center'][0]
        tag_size = self.calculate_tag_size(detection)

        if tag_size is None:
            return 0, 0

        rotation_error = tag_center_x - self.center_x
        rotation_speed = self.kp_rotation * rotation_error
        rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

        forward_speed = self.max_forward_speed
        return rotation_speed, forward_speed

    def apply_motor_speeds(self, rotation_speed, forward_speed):
        """Apply motor speeds"""
        left_speed = forward_speed + rotation_speed
        right_speed = forward_speed - rotation_speed

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

            if self.has_collected:
                self.update_attached_apriltag()
                post_collection_steps += 1

                if post_collection_steps < 100:
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(-2.0)
                elif post_collection_steps == 100:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    print("\nRobot has successfully collected the AprilTag!")
                    print("AprilTag is now attached to the robot.")
                continue

            if self.check_collection():
                self.attach_apriltag_to_robot()
                continue

            image_data = self.camera.getImage()
            if image_data:
                image = np.frombuffer(image_data, np.uint8).reshape(
                    (self.height, self.width, 4))
                gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

                detection = self.detect_apriltag(gray)

                if detection:
                    rotation_speed, forward_speed = self.calculate_motor_speeds(detection)
                    self.apply_motor_speeds(rotation_speed, forward_speed)

                    if step % 20 == 0:
                        tag_size = self.calculate_tag_size(detection)
                        robot_pos = self.get_robot_position()
                        tag_pos = self.get_apriltag_position()

                        if tag_pos is not None:
                            distance = np.linalg.norm(robot_pos - tag_pos)
                            print(f"[Step {step}] Approaching AprilTag")
                            print(f"  Distance: {distance:.3f}m, Tag size: {tag_size:.0f}px")
                            print(f"  Forward: {forward_speed:.2f}, Rotation: {rotation_speed:.2f}")

                    if step % 100 == 0:
                        vis_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                        corners = np.array(detection['corners'], dtype=np.int32)
                        cv2.polylines(vis_img, [corners], True, (0, 255, 0), 2)
                        center = tuple(map(int, detection['center']))
                        cv2.circle(vis_img, center, 5, (0, 0, 255), -1)
                        cv2.putText(vis_img, "Approaching...", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        filename = f"collection_{step:06d}.png"
                        cv2.imwrite(filename, vis_img)
                        print(f"  Saved: {filename}")

                    last_known_tag_position = self.get_apriltag_position()
                    lost_tag_counter = 0
                else:
                    lost_tag_counter += 1

                    if lost_tag_counter < 30 and last_known_tag_position is not None:
                        self.left_motor.setVelocity(4.0)
                        self.right_motor.setVelocity(4.0)

                        if step % 10 == 0:
                            robot_pos = self.get_robot_position()
                            tag_pos = self.get_apriltag_position()
                            if tag_pos is not None:
                                distance = np.linalg.norm(robot_pos - tag_pos)
                                print(f"[Step {step}] Lost visual, moving forward. Distance: {distance:.3f}m")
                    else:
                        self.left_motor.setVelocity(1.0)
                        self.right_motor.setVelocity(-1.0)

                        if step % 50 == 0:
                            print(f"[Step {step}] Searching for AprilTag...")

if __name__ == "__main__":
    collector = AprilTagCollector()
    collector.run()
