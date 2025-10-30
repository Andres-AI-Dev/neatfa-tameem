#!/usr/bin/env python3
"""
AprilTag Collector and Depositor V2 - Robot locks onto single AprilTag to prevent jittering
"""

from controller import Robot, Supervisor
import numpy as np
import cv2
import sys
import math
import os  # <-- added

# ---- shared policy utils (logging / model) ----
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "policy"))
from common_policy import init_logger, log_row, features_from_detection, load_policy, policy_predict_left_right
# ------------------------------------------------

# Add the AprilTag library to path
sys.path.insert(0, '../../../apriltag/build')

# CHANGED: prefer pupil_apriltags (prebuilt wheel) instead of apriltag-from-source
try:
    from pupil_apriltags import Detector as AprilDetector
    APRILTAG_AVAILABLE = True
    print("AprilTag library (pupil_apriltags) loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import pupil_apriltags: {e}")
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
                # CHANGED: use pupil_apriltags Detector with tag36h11 family
                self.detector = AprilDetector(families="tag36h11")
                print("AprilTag detector initialized with tag36h11 family (pupil_apriltags)")
            except Exception as e:
                print(f"Could not initialize pupil_apriltags detector: {e}")

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

        # Locking mechanism - lock onto single AprilTag to prevent jittering
        self.locked_tag_corners = None  # Corners of the locked AprilTag (unique identifier)
        self.locked_tag_center = None  # Last known center position of locked tag
        self.lock_timeout = 0  # Counter for how long we've lost the locked tag
        self.lock_tolerance = 150  # Increased pixel tolerance for matching locked tag (more forgiving)
        self.lock_duration = 0  # How long we've been locked on current tag
        self.min_lock_duration = 10  # Very short minimum lock (about 0.3 seconds) to prevent rapid switching

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

        # ----- policy integration (minimal) -----
        self.policy_mode = os.environ.get("APRILTAG_POLICY", "").lower()
        self.policy_model = None
        self.epuck_max_speed = 6.28  # for clipping policy outputs

        # attributes expected by features_from_detection()
        self.smoothed_center_x = None
        self.smoothed_size = None
        self.smoothing_factor = 0.7
        self.last_rotation_error = 0
        self.last_distance_error = 0
        self.target_tag_size = 150  # reuse follower target for distance feature

        if self.policy_mode in ("log", "1", "true"):
            init_logger()  # paths resolved via env in common_policy
            print("Policy logging: ON (APRILTAG_POLICY=log)")
        elif self.policy_mode == "use":
            self.policy_model = load_policy()
            if self.policy_model is None:
                print("⚠️  APRILTAG_POLICY=use set but no model found; continuing without policy.")
                self.policy_mode = ""
            else:
                print("Policy inference: ON (APRILTAG_POLICY=use)")
        # ---------------------------------------

        print("\n=== AprilTag Collector & Depositor V2 Initialized ===")
        print("Robot will search for AprilTags visually")
        print("Robot will LOCK onto single AprilTag to prevent jittering")
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

        # If it's stale data, return a default size
        if detection.get('stale', False):
            return 100  # Return medium size to maintain approach

        try:
            corners = np.array(detection['corners'])
            edges = [np.linalg.norm(corners[(i+1)%4] - corners[i]) for i in range(4)]
            return sum(edges) / 4
        except:
            return None

    # (kept) original simple version (will be overridden by the enhanced one below)
    def find_apriltag_visually(self, gray_image):
        """Search for AprilTag in camera view"""
        if self.detector is None:
            return None
        try:
            detections = self.detector.detect(gray_image)
            if len(detections) > 0:
                # CHANGED: convert pupil_apriltags result to your expected dict
                det = detections[0]
                return {
                    'id': int(getattr(det, "tag_id", 0)),
                    'center': tuple(map(float, det.center)),
                    'corners': np.asarray(det.corners, dtype=float),
                    'lb-rb-rt-lt': np.asarray(det.corners, dtype=float),  # maintain key your code uses
                    'margin': float(getattr(det, "decision_margin", 0.0))
                }
        except:
            pass
        return None

    # Enhanced locking version (overrides the simple one above)
    def find_apriltag_visually(self, gray_image):
        """Search for AprilTag in camera view with closest-tag locking mechanism"""
        if self.detector is None:
            return None

        try:
            raw = self.detector.detect(gray_image)

            # Convert all detections to the dict format your code expects
            detections = []
            for det in raw:
                detections.append({
                    'id': int(getattr(det, "tag_id", 0)),
                    'center': tuple(map(float, det.center)),
                    'corners': np.asarray(det.corners, dtype=float),
                    'lb-rb-rt-lt': np.asarray(det.corners, dtype=float),
                    'margin': float(getattr(det, "decision_margin", 0.0))
                })

            # If we have a locked tag, try to find it again
            if self.locked_tag_corners is not None and self.locked_tag_center is not None:
                self.lock_duration += 1  # Increment lock duration

                if self.lock_duration < self.min_lock_duration:
                    # Try to find our locked tag in current detections
                    for det in detections:
                        center = det['center']
                        dist = np.sqrt((center[0] - self.locked_tag_center[0])**2 +
                                       (center[1] - self.locked_tag_center[1])**2)
                        if dist < self.lock_tolerance:
                            # Update position but stay locked
                            self.lock_timeout = 0
                            self.locked_tag_center = det['center']
                            self.locked_tag_corners = det['lb-rb-rt-lt']
                            return det

                    # Even if we don't see it, return last known position during min lock period
                    self.lock_timeout += 1
                    return {
                        'id': 0,
                        'center': self.locked_tag_center,
                        'corners': self.locked_tag_corners,
                        'lb-rb-rt-lt': self.locked_tag_corners,
                        'margin': 0.0,
                        'stale': True
                    }

                # After minimum lock duration, try to find our locked tag
                best_match = None
                best_distance = float('inf')

                for det in detections:
                    center = det['center']
                    dist = np.sqrt((center[0] - self.locked_tag_center[0])**2 +
                                   (center[1] - self.locked_tag_center[1])**2)

                    if dist < self.lock_tolerance and dist < best_distance:
                        best_match = det
                        best_distance = dist

                if best_match:
                    # Found our locked tag
                    self.lock_timeout = 0
                    self.locked_tag_center = best_match['center']
                    self.locked_tag_corners = best_match['lb-rb-rt-lt']
                    return best_match

                # Didn't find locked tag - increment timeout
                self.lock_timeout += 1

                if self.lock_timeout > 100:
                    print(f"[LOCK] Lost locked tag for extended time, unlocking...")
                    self.locked_tag_corners = None
                    self.locked_tag_center = None
                    self.lock_timeout = 0
                    self.lock_duration = 0
                else:
                    if self.lock_timeout < 50:
                        return {
                            'id': 0,
                            'center': self.locked_tag_center,
                            'corners': self.locked_tag_corners,
                            'lb-rb-rt-lt': self.locked_tag_corners,
                            'margin': 0.0,
                            'stale': True
                        }
                    return None

            # No locked tag or lost it - find the CLOSEST tag (centered and large)
            if len(detections) > 0:
                best_det = None
                best_score = -float('inf')

                for det in detections:
                    corners = np.array(det['lb-rb-rt-lt'])
                    # Calculate tag size (perimeter)
                    edges = [np.linalg.norm(corners[(i+1)%4] - corners[i]) for i in range(4)]
                    size = sum(edges)

                    # Centeredness
                    center_x = det['center'][0]
                    center_offset = abs(center_x - self.center_x)
                    center_penalty = (center_offset / self.center_x) ** 2

                    score = size * (1.0 - center_penalty)

                    if score > best_score:
                        best_score = score
                        best_size = size
                        best_det = det

                if best_det:
                    # Lock onto the closest tag
                    self.locked_tag_corners = best_det['lb-rb-rt-lt']
                    self.locked_tag_center = best_det['center']
                    self.lock_timeout = 0
                    self.lock_duration = 0
                    center_offset = abs(best_det['center'][0] - self.center_x)
                    print(f"[LOCK] Locked onto CLOSEST AprilTag at position ({best_det['center'][0]:.0f}, {best_det['center'][1]:.0f}), size: {best_size:.0f}px, offset: {center_offset:.0f}px")
                    return best_det

        except Exception as e:
            print(f"Detection error: {e}")
        return None

    def collect_apriltag_at_position(self, x, y):
        """Try to collect AprilTag near given position"""
        for i in range(1, 55):
            node = self.robot.getFromDef(f"APRILTAG_{i}")
            if node:
                translation_field = node.getField("translation")
                if translation_field:
                    pos = translation_field.getSFVec3f()
                    dist = math.sqrt((pos[0]-x)**2 + (pos[1]-y)**2)
                    if dist < 0.1:
                        return self.attach_apriltag(node)
        return False

    def attach_apriltag(self, node):
        """Attach AprilTag node to robot"""
        robot_position = self.robot_node.getPosition()

        translation_field = node.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([
                robot_position[0],
                robot_position[1],
                self.attach_height
            ])

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

        translation_field = self.current_carried_tag.getField("translation")
        if translation_field:
            translation_field.setSFVec3f([100, 100, -10])

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
            yaw = math.atan2(rotation[3], rotation[0])
            return yaw
        return 0

    def calculate_angle_to_base(self):
        """Calculate the angle the robot needs to turn to face the base"""
        robot_pos = self.get_robot_position()
        target_angle = math.atan2(-robot_pos[1], -robot_pos[0])
        current_angle = self.get_robot_orientation()
        angle_diff = target_angle - current_angle
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

            if self.current_carried_tag:
                self.update_carried_apriltag()

            image_data = self.camera.getImage()
            if not image_data:
                continue

            image = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

            if self.state == "SEARCHING":
                detection = self.find_apriltag_visually(gray)

                if detection:
                    self.state = "APPROACHING"
                    self.lost_visual_counter = 0
                    print(f"\n[SEARCHING] Found AprilTag visually!")
                else:
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(-2.0)
                    if step % 50 == 0:
                        print(f"[SEARCHING] Looking for AprilTags... (Deposited: {self.deposit_count})")

            elif self.state == "APPROACHING":
                robot_pos = self.get_robot_position()
                if self.collect_apriltag_at_position(robot_pos[0], robot_pos[1]):
                    print(f"[LOCK] Collected a tag, clearing lock")
                    self.locked_tag_corners = None
                    self.locked_tag_center = None
                    self.lock_timeout = 0
                    self.lock_duration = 0

                    self.state = "TURNING_TO_BASE"
                    self.turn_to_base_counter = 0
                    print(f"[COLLECTED] Got AprilTag! Turning to face base...")
                    continue

                detection = self.find_apriltag_visually(gray)

                if detection:
                    # rotation + forward (this is your original logic)
                    tag_center_x = detection['center'][0]
                    rotation_error = tag_center_x - self.center_x
                    rotation_speed = rotation_error * 0.02
                    rotation_speed = np.clip(rotation_speed, -self.max_rotation_speed, self.max_rotation_speed)
                    forward_speed = self.max_forward_speed

                    # ---- policy hook ----
                    if self.policy_mode in ("log", "1", "true") or (self.policy_mode == "use" and self.policy_model is not None):
                        feat = features_from_detection(self, detection)

                    if self.policy_mode == "use" and self.policy_model is not None:
                        # Use learned policy outputs directly
                        left_speed, right_speed = policy_predict_left_right(self.policy_model, feat, vmax=self.epuck_max_speed)
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                        # skip PD-style action when using policy
                    else:
                        # Your original actuation
                        left_speed = forward_speed + rotation_speed
                        right_speed = forward_speed - rotation_speed
                        left_speed = np.clip(left_speed, -6.28, 6.28)
                        right_speed = np.clip(right_speed, -6.28, 6.28)
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)

                        # If logging, record the features + PD action we just applied
                        if self.policy_mode in ("log", "1", "true"):
                            t = self.robot.getTime()
                            err_x, err_x_d, size_err, size_err_d = feat
                            log_row(t, err_x, err_x_d, size_err, size_err_d,
                                    rotation_speed, forward_speed, float(left_speed), float(right_speed))
                    # ---- end policy hook ----

                    self.lost_visual_counter = 0

                    tag_size = self.calculate_tag_size(detection)
                    if step % 20 == 0:
                        tag_id = detection.get('id', 'Unknown')
                        print(f"[APPROACHING] Locked tag {tag_id}, size: {tag_size:.0f}px, Full speed ahead!")

                    if tag_size and tag_size > 400:
                        print(f"[APPROACHING] Very close! Maintaining speed for collection...")
                else:
                    self.lost_visual_counter += 1
                    if self.lost_visual_counter < 40:
                        self.left_motor.setVelocity(4.0)
                        self.right_motor.setVelocity(4.0)

                        if self.lost_visual_counter % 10 == 0:
                            robot_pos = self.get_robot_position()
                            if self.collect_apriltag_at_position(robot_pos[0], robot_pos[1]):
                                print(f"[LOCK] Collected locked tag, clearing lock")
                                self.locked_tag_corners = None
                                self.locked_tag_center = None
                                self.lock_timeout = 0
                                self.lock_duration = 0

                                self.state = "TURNING_TO_BASE"
                                self.turn_to_base_counter = 0
                                print(f"[COLLECTED] Got AprilTag! Turning to face base...")

                        if self.lost_visual_counter == 10:
                            print(f"[APPROACHING] Lost visual - charging forward to collect!")
                    else:
                        self.state = "SEARCHING"
                        print("[LOST] No collection after charge, searching again...")

            elif self.state == "TURNING_TO_BASE":
                angle_error = self.calculate_angle_to_base()

                if abs(angle_error) < 0.15:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    self.state = "RETURNING"
                    print(f"[TURNING] Facing base! Angle error: {math.degrees(angle_error):.1f}°")
                else:
                    turn_speed = np.clip(angle_error * 3.0, -3.0, 3.0)
                    self.left_motor.setVelocity(-turn_speed)
                    self.right_motor.setVelocity(turn_speed)

                    if step % 20 == 0:
                        robot_pos = self.get_robot_position()
                        print(f"[TURNING] Angle to base: {math.degrees(angle_error):.1f}°")
                        print(f"  Position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")

            elif self.state == "RETURNING":
                robot_pos = self.get_robot_position()
                distance = np.linalg.norm(robot_pos - self.base_position)

                if distance > self.deposit_distance:
                    angle_error = self.calculate_angle_to_base()

                    if abs(angle_error) > 0.3:
                        self.state = "TURNING_TO_BASE"
                        print(f"[RETURNING] Course correction needed, angle error: {math.degrees(angle_error):.1f}°")
                    else:
                        forward_speed = 4.0
                        steering = angle_error * 2.0

                        left_speed = forward_speed - steering
                        right_speed = forward_speed + steering

                        self.left_motor.setVelocity(np.clip(left_speed, -6.28, 6.28))
                        self.right_motor.setVelocity(np.clip(right_speed, -6.28, 6.28))

                        if step % 20 == 0:
                            print(f"[RETURNING] Distance: {distance:.3f}m, Position: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
                            print(f"  Heading error: {math.degrees(angle_error):.1f}°")
                else:
                    self.state = "DEPOSITING"
                    print("[RETURNING] Reached base - depositing while phasing through!")

            elif self.state == "DEPOSITING":
                if self.deposit_apriltag():
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(2.0)

                    for _ in range(20):
                        self.robot.step(self.timestep)
                        self.update_carried_apriltag()

                    self.state = "SEARCHING"
                    print(f"[DEPOSITING] Complete! Searching for more AprilTags...")

if __name__ == "__main__":
    collector = AprilTagCollectorDepositor()
    collector.run()
