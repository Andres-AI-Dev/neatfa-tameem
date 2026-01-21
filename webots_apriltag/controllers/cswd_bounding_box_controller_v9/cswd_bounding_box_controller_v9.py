#!/usr/bin/env python3
"""
CSWD Controller with Bounding Box Visualization - Collect, Stop, Wait, Deposit with LED, sound signaling, and visual feedback
"""

from controller import Robot, Supervisor
import numpy as np
import cv2
import sys
import math
import os
import ctypes

# Add the AprilTag library to path
apriltag_path = os.path.abspath('../../../apriltag/build')
sys.path.insert(0, apriltag_path)

# Preload the shared library using ctypes
try:
    lib_path = os.path.join(apriltag_path, 'libapriltag.so.3')
    ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
    print(f"Loaded AprilTag shared library from: {lib_path}")
except Exception as e:
    print(f"Warning: Could not preload libapriltag.so.3: {e}")

try:
    import apriltag
    APRILTAG_AVAILABLE = True
    print("AprilTag library loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import apriltag library: {e}")
    print(f"  AprilTag path: {apriltag_path}")
    APRILTAG_AVAILABLE = False

class CSWDController:
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

        # Display for bounding box visualization (using turretSlot)
        self.display = self.robot.getDevice("bounding_box_display")
        if self.display:
            # Attach the camera to the display for overlay
            self.display.attachCamera(self.camera)
            print("Display device initialized and attached to camera")
        else:
            print("Warning: Display device not found")
            self.display = None

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Initialize LEDs
        self.leds = []
        for i in range(8):
            led = self.robot.getDevice(f"led{i}")
            if led:
                self.leds.append(led)

        # Body LED (green)
        self.body_led = self.robot.getDevice("led8")

        # Front LED (red)
        self.front_led = self.robot.getDevice("led9")

        # Speaker for beeps
        self.speaker = self.robot.getDevice("speaker")

        print(f"Initialized {len(self.leds)} ring LEDs, body LED, and speaker")

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
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, SIGNALING, COLLECTING, TURNING_TO_BASE, RETURNING, DEPOSITING
        self.lost_visual_counter = 0
        self.turn_to_base_counter = 0
        self.signal_phase = 0  # For tracking signal sequence
        self.signal_timer = 0  # Timer for signal phases
        self.collection_timer = 0  # Timer for collection phase

        # Locking mechanism - lock onto single AprilTag to prevent jittering
        self.locked_tag_corners = None  # Corners of the locked AprilTag (unique identifier)
        self.locked_tag_center = None  # Last known center position of locked tag
        self.lock_timeout = 0  # Counter for how long we've lost the locked tag
        self.lock_tolerance = 150  # Increased pixel tolerance for matching locked tag (more forgiving)
        self.lock_duration = 0  # How long we've been locked on current tag
        self.min_lock_duration = 10  # Very short minimum lock (about 0.3 seconds) to prevent rapid switching

        # Store all detections for visualization
        self.all_detections = []

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

        print("\n=== CSWD Controller Initialized ===")
        print("Collect: Approach AprilTag")
        print("Stop: Stop at optimal distance")
        print("Wait: Signal with LEDs and beeps")
        print("Deposit: Return to base")
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
        """Search for AprilTag in camera view with closest-tag locking mechanism"""
        if self.detector is None:
            return None

        try:
            detections = self.detector.detect(gray_image)

            # If we have a locked tag, try to find it again
            if self.locked_tag_corners is not None and self.locked_tag_center is not None:
                self.lock_duration += 1  # Increment lock duration

                # If we haven't been locked long enough, ALWAYS return the locked tag
                # This prevents switching even if we see a closer tag
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
                            return {
                                'id': det['id'],
                                'center': det['center'],
                                'corners': det['lb-rb-rt-lt'],
                                'margin': det['margin']
                            }

                    # Even if we don't see it, return last known position during min lock period
                    self.lock_timeout += 1
                    return {
                        'id': 0,
                        'center': self.locked_tag_center,
                        'corners': self.locked_tag_corners,
                        'margin': 0,
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
                    return {
                        'id': best_match['id'],
                        'center': best_match['center'],
                        'corners': best_match['lb-rb-rt-lt'],
                        'margin': best_match['margin']
                    }

                # Didn't find locked tag - increment timeout
                self.lock_timeout += 1

                # If we've lost it for too long, unlock and find a new one
                if self.lock_timeout > 100:  # Much longer timeout - about 3 seconds
                    print(f"[LOCK] Lost locked tag for extended time, unlocking...")
                    self.locked_tag_corners = None
                    self.locked_tag_center = None
                    self.lock_timeout = 0
                    self.lock_duration = 0
                    # Fall through to find new closest tag
                else:
                    # Keep returning last known position for longer
                    if self.lock_timeout < 50:  # Return stale position for longer
                        return {
                            'id': 0,
                            'center': self.locked_tag_center,
                            'corners': self.locked_tag_corners,
                            'margin': 0,
                            'stale': True
                        }
                    return None

            # No locked tag or lost it - find the LARGEST tag (closest physically)
            if len(detections) > 0:
                # Find the largest tag by size only (biggest = closest)
                best_det = None
                best_size = 0

                for det in detections:
                    corners = np.array(det['lb-rb-rt-lt'])
                    # Calculate tag size (perimeter)
                    edges = [np.linalg.norm(corners[(i+1)%4] - corners[i]) for i in range(4)]
                    size = sum(edges)

                    # Simply pick the largest tag
                    if size > best_size:
                        best_size = size
                        best_det = det

                if best_det:
                    # Lock onto the largest (closest) tag
                    self.locked_tag_corners = best_det['lb-rb-rt-lt']
                    self.locked_tag_center = best_det['center']
                    self.lock_timeout = 0
                    self.lock_duration = 0  # Reset lock duration for new lock
                    center_offset = abs(best_det['center'][0] - self.center_x)
                    print(f"[LOCK] Locked onto LARGEST AprilTag at position ({best_det['center'][0]:.0f}, {best_det['center'][1]:.0f}), size: {best_size:.0f}px, offset: {center_offset:.0f}px")
                    return {
                        'id': best_det['id'],
                        'center': best_det['center'],
                        'corners': best_det['lb-rb-rt-lt'],
                        'margin': best_det['margin']
                    }
        except Exception as e:
            print(f"Detection error: {e}")
        return None

    def signal_for_pickup(self):
        """Execute LED and sound signaling sequence"""
        # Flash ring LEDs 3 times with beeps
        if self.signal_phase < 6:  # 3 flashes (on-off-on-off-on-off)
            if self.signal_phase % 2 == 0:  # Even phase = LEDs on
                # Turn on all ring LEDs
                for led in self.leds:
                    led.set(1)
                # Play short beep
                if self.speaker and self.signal_phase == 0:
                    # Note: In real implementation, would play WAV file
                    # self.speaker.playSound(self.speaker, "short_beep.wav", 1.0, 1.0, 0, False)
                    pass
            else:  # Odd phase = LEDs off
                # Turn off all ring LEDs
                for led in self.leds:
                    led.set(0)

            self.signal_timer += 1
            if self.signal_timer > 10:  # Each phase lasts ~0.3 seconds
                self.signal_phase += 1
                self.signal_timer = 0

        elif self.signal_phase == 6:  # Turn on green body LED and long beep
            # Turn off ring LEDs
            for led in self.leds:
                led.set(0)
            # Turn on green body LED
            if self.body_led:
                self.body_led.set(1)
            # Play long beep
            if self.speaker and self.signal_timer == 0:
                # Note: In real implementation, would play WAV file
                # self.speaker.playSound(self.speaker, "long_beep.wav", 1.0, 1.0, 0, False)
                pass

            self.signal_timer += 1
            if self.signal_timer > 20:  # Long beep phase
                self.signal_phase += 1
                self.signal_timer = 0
                return True  # Signal sequence complete

        return False  # Still signaling

    def reset_leds(self):
        """Turn off all LEDs"""
        for led in self.leds:
            led.set(0)
        if self.body_led:
            self.body_led.set(0)
        if self.front_led:
            self.front_led.set(0)

    def clear_display(self):
        """Clear the display for new bounding boxes"""
        if self.display:
            # The display automatically shows the camera feed as background
            # We just need to clear any previous overlays
            pass

    def draw_bounding_box(self, detection, color=0xFF0000, label=""):
        """Draw a bounding box around detected AprilTag

        Args:
            detection: AprilTag detection with 'corners' field
            color: RGB color as integer (default red 0xFF0000)
            label: Optional text label
        """
        if not self.display or not detection:
            return

        try:
            # Get corners from detection
            corners = detection.get('corners', None)
            if corners is None:
                return

            # Convert corners to integer coordinates
            corners = np.array(corners).astype(int)

            # Calculate bounding box from corners
            x_coords = corners[:, 0]
            y_coords = corners[:, 1]
            x_min, x_max = int(np.min(x_coords)), int(np.max(x_coords))
            y_min, y_max = int(np.min(y_coords)), int(np.max(y_coords))

            # Ensure coordinates are within display bounds
            x_min = max(0, min(x_min, self.width - 1))
            x_max = max(0, min(x_max, self.width - 1))
            y_min = max(0, min(y_min, self.height - 1))
            y_max = max(0, min(y_max, self.height - 1))

            # Calculate width and height
            bbox_width = x_max - x_min
            bbox_height = y_max - y_min

            if bbox_width > 0 and bbox_height > 0:
                # Set color and alpha for drawing
                self.display.setColor(color)
                self.display.setAlpha(1.0)  # Fully opaque for the bounding box
                self.display.drawRectangle(x_min, y_min, bbox_width, bbox_height)

                # Draw center cross
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                cross_size = 5
                self.display.drawLine(center_x - cross_size, center_y, center_x + cross_size, center_y)
                self.display.drawLine(center_x, center_y - cross_size, center_x, center_y + cross_size)

                # Add label if provided
                if label:
                    self.display.drawText(label, x_min, y_min - 15)

        except Exception as e:
            print(f"Error drawing bounding box: {e}")

    def get_bounding_box_color(self):
        """Get color for bounding box based on current state"""
        if self.state == "SEARCHING":
            return 0xFF0000  # Red - detecting
        elif self.state == "APPROACHING":
            return 0x00FF00  # Green - locked and approaching
        elif self.state in ["SIGNALING", "COLLECTING"]:
            return 0x0000FF  # Blue - collecting
        elif self.state in ["TURNING_TO_BASE", "RETURNING", "DEPOSITING"]:
            return 0xFFFF00  # Yellow - returning
        else:
            return 0xFF00FF  # Magenta - other states

    def collect_apriltag_at_position(self, x, y):
        """Try to collect AprilTag near given position"""
        # Find AprilTag nodes near this position
        print(f"[DEBUG] Checking for AprilTags near robot position: ({x:.2f}, {y:.2f})")
        closest_dist = float('inf')
        closest_node = None

        for i in range(1, 55):  # Check up to 55 possible tags (50 in clusters + margin)
            node = self.robot.getFromDef(f"APRILTAG_{i}")
            if node:
                translation_field = node.getField("translation")
                if translation_field:
                    pos = translation_field.getSFVec3f()
                    dist = math.sqrt((pos[0]-x)**2 + (pos[1]-y)**2)
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_node = node
                    if dist < 0.15:  # Within 15cm (more generous)
                        print(f"[DEBUG] Found AprilTag {i} at distance {dist:.3f}m - Collecting!")
                        return self.attach_apriltag(node)

        if closest_node:
            print(f"[DEBUG] Closest AprilTag is {closest_dist:.3f}m away - too far to collect")
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

            # Get ALL detections for visualization (fresh each frame)
            if self.detector is not None:
                try:
                    self.all_detections = self.detector.detect(gray)
                except:
                    self.all_detections = []
            else:
                self.all_detections = []

            # Look for AprilTags and prepare visualization
            current_detection = None

            # State machine
            if self.state == "SEARCHING":
                # Look for AprilTag visually
                detection = self.find_apriltag_visually(gray)
                current_detection = detection

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
                current_detection = detection

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
                        tag_id = detection.get('id', 'Unknown')
                        print(f"[APPROACHING] Locked tag {tag_id}, size: {tag_size:.0f}px, Full speed ahead!")

                    # Check if we should stop for signaling
                    if tag_size and tag_size > 250:  # Stop earlier to avoid overshooting
                        # Stop the robot
                        self.left_motor.setVelocity(0)
                        self.right_motor.setVelocity(0)
                        self.state = "SIGNALING"
                        self.signal_phase = 0
                        self.signal_timer = 0
                        print(f"[CSWD] Stopping at optimal distance (size: {tag_size:.0f}px) - Starting signal sequence...")
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
                                # Clear the lock since we collected the tag
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
                        # Really lost it - go back to searching
                        self.state = "SEARCHING"
                        print("[LOST] No collection after charge, searching again...")

            elif self.state == "SIGNALING":
                # Execute signal sequence
                signal_complete = self.signal_for_pickup()

                if signal_complete:
                    print(f"[CSWD] Signal complete - Moving forward to collect...")
                    # Reset LEDs
                    self.reset_leds()

                    # Move forward briefly to get closer for collection
                    self.left_motor.setVelocity(2.0)
                    self.right_motor.setVelocity(2.0)

                    # Change to a collection state
                    self.state = "COLLECTING"
                    self.collection_timer = 0

            elif self.state == "COLLECTING":
                # Move forward for a brief time then try to collect
                self.collection_timer += 1

                if self.collection_timer > 10:  # After moving forward briefly
                    # Stop and try to collect
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)

                    robot_pos = self.get_robot_position()
                    if self.collect_apriltag_at_position(robot_pos[0], robot_pos[1]):
                        # Clear the lock
                        self.locked_tag_corners = None
                        self.locked_tag_center = None
                        self.lock_timeout = 0
                        self.lock_duration = 0

                        self.state = "TURNING_TO_BASE"
                        self.turn_to_base_counter = 0
                        print(f"[COLLECTED] AprilTag attached! Returning to base...")
                    else:
                        # If collection failed, go back to approaching
                        print(f"[CSWD] Collection failed, continuing approach...")
                        self.state = "APPROACHING"

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
                        # Move forward at FULL SPEED with slight steering correction
                        forward_speed = 4.0  # Always max speed when returning

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

            # Clear display and draw bounding box visualization
            if self.display:
                # Clear the display by filling with transparent color (this resets the overlay)
                self.display.setColor(0x000000)  # Black
                self.display.setAlpha(0.0)      # Fully transparent
                self.display.fillRectangle(0, 0, self.width, self.height)

                # Draw all detected tags in green
                if hasattr(self, 'all_detections') and self.all_detections:
                    for det in self.all_detections:
                        # Convert detection to our format
                        det_dict = {
                            'id': det['id'],
                            'center': det['center'],
                            'corners': det['lb-rb-rt-lt'],
                            'margin': det['margin']
                        }
                        # Draw in green (all detections)
                        self.draw_bounding_box(det_dict, 0x00FF00, "")

                # Draw locked tag in red on top
                if current_detection and self.locked_tag_center is not None:
                    state_label = f"{self.state} (LOCKED)"
                    self.draw_bounding_box(current_detection, 0xFF0000, state_label)

if __name__ == "__main__":
    controller = CSWDController()
    controller.run()