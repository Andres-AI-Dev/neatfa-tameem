#!/usr/bin/env python3
"""
AprilTag Collector and Depositor - Robot collects AprilTags and deposits them at base
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

class AprilTagCollectorDepositor:
    def __init__(self):
        # Initialize as Supervisor
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

        # Get nodes
        self.robot_node = self.robot.getSelf()
        self.base_node = self.robot.getFromDef("DEPOSIT_BASE")

        # Storage for collected AprilTags (we find them visually)
        self.collected_tags = []  # List of collected tag nodes
        self.current_carried_tag = None

        # State machine
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, TURNING_TO_BASE, RETURNING, DEPOSITING
        self.lost_visual_counter = 0
        self.turn_to_base_counter = 0

        # Deposit counter
        self.deposit_count = 0

        # Movement parameters
        self.max_forward_speed = 4.0
        self.max_rotation_speed = 4.0
        self.collection_distance = 0.05
        self.deposit_distance = 0.1  # Distance to center for depositing (smaller since we phase through)
        self.attach_height = 0.073

        # Base position (always at origin)
        self.base_position = np.array([0, 0])

        print("\n=== AprilTag Collector & Depositor Initialized ===")
        print("Robot will search for AprilTags visually")
        print("Robot will collect and deposit them at the base")
        print("="*50)

    def get_robot_position(self):
        """Get robot's current position"""
        if self.robot_node:
            position = self.robot_node.getPosition()
            return np.array(position[:2])
        return np.array([0, 0])

    def calculate_tag_size(self, detection):
        """Calculate apparent size of tag"""
        if detection is None:
            return None
        try:
            corners = np.array(detection['corners'])
            edges = [np.linalg.norm(corners[(i+1)%4] - corners[i]) for i in range(4)]
            return sum(edges) / 4
        except:
            return None

    def find_apriltag_visually(self, gray_image):
        """Search for AprilTag in camera view"""
        if self.detector is None:
            return None

        try:
            detections = self.detector.detect(gray_image)
            if len(detections) > 0:
                return detections[0]  # Return first detection
        except:
            pass
        return None

    def find_apriltag_visually(self, gray_image):
        """Search for AprilTag in camera view"""
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
        except:
            pass
        return None

    def collect_apriltag_at_position(self, x, y):
        """Try to collect AprilTag near given position"""
        # Find AprilTag nodes near this position
        for i in range(1, 25):  # Check up to 25 possible tags (increased from 10)
            node = self.robot.getFromDef(f"APRILTAG_{i}")
            if node:
                translation_field = node.getField("translation")
                if translation_field:
                    pos = translation_field.getSFVec3f()
                    dist = math.sqrt((pos[0]-x)**2 + (pos[1]-y)**2)
                    if dist < 0.1:  # Within 10cm
                        return self.attach_apriltag(node)
        return False

    def attach_apriltag(self, node):
        """Attach AprilTag node to robot"""
        # Get robot position
        robot_position = self.robot_node.getPosition()

        # Move AprilTag to robot top
        translation_field = node.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([
                robot_position[0],
                robot_position[1],
                self.attach_height
            ])

        # Make semi-transparent
        children_field = node.getField("children")
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

        self.current_carried_tag = node
        self.collected_tags.append(node)

        print(f"\n*** COLLECTED APRILTAG! ***")
        return True

    def update_carried_apriltag(self):
        """Update position of carried AprilTag"""
        if self.current_carried_tag:
            robot_position = self.robot_node.getPosition()

            translation_field = self.current_carried_tag.getField("translation")
            if translation_field:
                translation_field.setSFVec3f([
                    robot_position[0],
                    robot_position[1],
                    self.attach_height
                ])

    def deposit_apriltag(self):
        """Deposit AprilTag at base and make it disappear"""
        if not self.current_carried_tag:
            return False

        # Hide the AprilTag (move it far away)
        translation_field = self.current_carried_tag.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([100, 100, -10])  # Move far away

        # Update counters
        self.deposit_count += 1

        print("\n" + "="*50)
        print(f"*** APRILTAG DEPOSITED AT BASE! ***")
        print(f"*** {self.deposit_count} AprilTag(s) collected ***")
        print("="*50 + "\n")

        self.current_carried_tag = None
        return True

    def get_robot_orientation(self):
        """Get the robot's current orientation (yaw angle)"""
        if self.robot_node:
            rotation = self.robot_node.getOrientation()
            # Rotation matrix to yaw angle
            # For Webots, the orientation matrix is 3x3 flattened
            # Calculate yaw from rotation matrix
            yaw = math.atan2(rotation[3], rotation[0])  # rotation[1][0], rotation[0][0]
            return yaw
        return 0

    def calculate_angle_to_base(self):
        """Calculate the angle the robot needs to turn to face the base"""
        robot_pos = self.get_robot_position()

        # Angle from robot to base (0,0)
        target_angle = math.atan2(-robot_pos[1], -robot_pos[0])

        # Current robot orientation
        current_angle = self.get_robot_orientation()

        # Calculate angle difference
        angle_diff = target_angle - current_angle

        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff


    def run(self):
        """Main control loop"""
        step = 0

        while self.robot.step(self.timestep) != -1:
            step += 1

            # Update carried AprilTag position
            if self.current_carried_tag:
                self.update_carried_apriltag()

            # Get camera image
            image_data = self.camera.getImage()
            if not image_data:
                continue

            image = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

            # State machine
            if self.state == "SEARCHING":
                # Look for AprilTag visually
                detection = self.find_apriltag_visually(gray)

                if detection:
                    self.state = "APPROACHING"
                    self.lost_visual_counter = 0
                    print(f"\n[SEARCHING] Found AprilTag visually!")
                else:
                    # Rotate to search
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(-2.0)
                    if step % 50 == 0:
                        print(f"[SEARCHING] Looking for AprilTags... (Deposited: {self.deposit_count})")

            elif self.state == "APPROACHING":
                # Look for AprilTag
                detection = self.find_apriltag_visually(gray)

                if detection:
                    # Visual tracking - center the tag
                    tag_center_x = detection['center'][0]
                    rotation_error = tag_center_x - self.center_x
                    rotation_speed = rotation_error * 0.02
                    rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)

                    # ALWAYS move forward at FULL SPEED - no slowing down
                    forward_speed = self.max_forward_speed

                    # Apply motor speeds
                    left_speed = forward_speed + rotation_speed
                    right_speed = forward_speed - rotation_speed
                    left_speed = np.clip(left_speed, -6.28, 6.28)
                    right_speed = np.clip(right_speed, -6.28, 6.28)
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)

                    self.lost_visual_counter = 0

                    # Check tag size to determine if close enough
                    tag_size = self.calculate_tag_size(detection)
                    if step % 20 == 0:
                        print(f"[APPROACHING] Tag size: {tag_size:.0f}px, Full speed ahead!")

                    # Don't slow down or stop - just note we're getting close
                    if tag_size and tag_size > 400:
                        print(f"[APPROACHING] Very close! Maintaining speed for collection...")
                else:
                    # Lost visual - KEEP CHARGING FORWARD at full speed
                    self.lost_visual_counter += 1
                    if self.lost_visual_counter < 40:  # Increased time
                        # Full speed ahead!
                        self.left_motor.setVelocity(4.0)
                        self.right_motor.setVelocity(4.0)

                        # Try to collect after brief forward movement
                        if self.lost_visual_counter % 10 == 0:
                            robot_pos = self.get_robot_position()
                            if self.collect_apriltag_at_position(robot_pos[0], robot_pos[1]):
                                self.state = "TURNING_TO_BASE"
                                self.turn_to_base_counter = 0
                                print(f"[COLLECTED] Got AprilTag! Turning to face base...")

                        if self.lost_visual_counter == 10:
                            print(f"[APPROACHING] Lost visual - charging forward to collect!")
                    else:
                        # Really lost it - go back to searching
                        self.state = "SEARCHING"
                        print("[LOST] No collection after charge, searching again...")

            elif self.state == "TURNING_TO_BASE":
                # Calculate how much to turn to face the base
                angle_error = self.calculate_angle_to_base()

                # If we're roughly facing the base, start moving
                if abs(angle_error) < 0.15:  # Within ~8 degrees
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state = "RETURNING"
                    print(f"[TURNING] Facing base! Angle error: {math.degrees(angle_error):.1f}°")
                else:
                    # Turn proportionally to the error
                    turn_speed = np.clip(angle_error * 3.0, -3.0, 3.0)

                    # Turn in place
                    self.left_motor.setVelocity(-turn_speed)
                    self.right_motor.setVelocity(turn_speed)

                    if step % 20 == 0:
                        robot_pos = self.get_robot_position()
                        print(f"[TURNING] Angle to base: {math.degrees(angle_error):.1f}°")
                        print(f"  Position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")

            elif self.state == "RETURNING":
                # Drive toward base with course correction
                robot_pos = self.get_robot_position()
                distance = np.linalg.norm(robot_pos - self.base_position)

                if distance > self.deposit_distance:
                    # Check if we're still facing the base
                    angle_error = self.calculate_angle_to_base()

                    if abs(angle_error) > 0.3:  # Drifted off course
                        # Need to correct heading
                        self.state = "TURNING_TO_BASE"
                        print(f"[RETURNING] Course correction needed, angle error: {math.degrees(angle_error):.1f}°")
                    else:
                        # Move forward with slight steering correction
                        forward_speed = min(4.0, 1.0 + distance * 3)

                        # Small proportional steering while moving
                        steering = angle_error * 2.0

                        left_speed = forward_speed - steering
                        right_speed = forward_speed + steering

                        # Apply speeds
                        self.left_motor.setVelocity(np.clip(left_speed, -6.28, 6.28))
                        self.right_motor.setVelocity(np.clip(right_speed, -6.28, 6.28))

                        if step % 20 == 0:
                            print(f"[RETURNING] Distance: {distance:.3f}m, Position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
                            print(f"  Heading error: {math.degrees(angle_error):.1f}°")
                else:
                    # Close enough to base - keep moving to phase through
                    self.state = "DEPOSITING"
                    print("[RETURNING] Reached base - depositing while phasing through!")

            elif self.state == "DEPOSITING":
                # Deposit while phasing through base
                if self.deposit_apriltag():
                    # Continue moving forward to exit the base
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(2.0)

                    # After a short delay, start searching again
                    for _ in range(20):
                        self.robot.step(self.timestep)
                        self.update_carried_apriltag()  # Keep tag hidden

                    self.state = "SEARCHING"
                    print(f"[DEPOSITING] Complete! Searching for more AprilTags...")

if __name__ == "__main__":
    collector = AprilTagCollectorDepositor()
    collector.run()