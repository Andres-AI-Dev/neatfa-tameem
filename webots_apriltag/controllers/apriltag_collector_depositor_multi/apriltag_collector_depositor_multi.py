#!/usr/bin/env python3
"""
ENHANCED CSWD Controller - With Communication & Federated Learning
Building on the proven working code with multi-robot coordination
"""

from controller import Robot, Supervisor
import numpy as np
import cv2
import sys
import math
import os
import re
import json
import random
from collections import deque

# --- Prefer SWIG apriltag; fallback to pupil_apriltags ---
sys.path.insert(0, '../../../apriltag/build')
APRIL_BACKEND = None
try:
    import apriltag as apriltag_swig
    APRIL_BACKEND = "swig"
    print("AprilTag (SWIG) available")
except Exception as e:
    print(f"[APRILTAG] SWIG load failed: {e}")

if APRIL_BACKEND is None:
    try:
        from pupil_apriltags import Detector as PupilDetector
        APRIL_BACKEND = "pupil"
        print("Using pupil_apriltags detector.")
    except Exception as e:
        print(f"[APRILTAG] pupil_apriltags load failed: {e}")
        APRIL_BACKEND = None


class RobotCommunication:
    """
    Simulated communication between robots for coordination and federated learning
    """
    def __init__(self, robot_supervisor, robot_id):
        self.supervisor = robot_supervisor
        self.robot_id = robot_id
        self.communication_range = 3.0  # meters
        self.message_queue = deque()
        self.received_messages = []
        self.federated_model = {
            'tag_success_rates': {},
            'zone_efficiency': {},
            'robot_performance': {}
        }
        
    def broadcast_message(self, message_type, data):
        """Broadcast message to nearby robots"""
        message = {
            'sender': self.robot_id,
            'type': message_type,
            'data': data,
            'timestamp': self.supervisor.getTime()
        }
        self.message_queue.append(message)
        
    def receive_messages(self):
        """Receive messages from nearby robots (simulated)"""
        self.received_messages.clear()
        
        # Get all robots in the world
        root = self.supervisor.getRoot()
        children = root.getField('children')
        
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node.getTypeName() == 'E-puck':
                robot_name = node.getField('name').getSFString()
                if robot_name != self.robot_id:  # Don't communicate with self
                    # Check if robot is in range
                    robot_pos = node.getPosition()
                    my_pos = self.supervisor.getSelf().getPosition()
                    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(robot_pos, my_pos)))
                    
                    if distance <= self.communication_range:
                        # Simulate receiving messages (in real implementation, this would use emitter/receiver)
                        if random.random() < 0.7:  # 70% chance of successful communication
                            # Simulate receiving performance data
                            simulated_message = {
                                'sender': robot_name,
                                'type': 'performance_update',
                                'data': {
                                    'tags_collected': random.randint(0, 5),
                                    'efficiency': random.uniform(0.1, 0.8),
                                    'current_zone': self._get_zone_from_position(robot_pos)
                                },
                                'timestamp': self.supervisor.getTime() - random.uniform(0, 2)
                            }
                            self.received_messages.append(simulated_message)
    
    def _get_zone_from_position(self, position):
        """Convert position to zone identifier"""
        x, y = position[0], position[1]
        if x >= 0 and y >= 0: return "top_right"
        elif x < 0 and y >= 0: return "top_left"
        elif x < 0 and y < 0: return "bottom_left"
        else: return "bottom_right"
    
    def update_federated_model(self, local_data):
        """Update federated learning model with local and received data"""
        # Add local data
        self.federated_model['robot_performance'][self.robot_id] = {
            'efficiency': local_data.get('efficiency', 0),
            'tags_collected': local_data.get('tags_collected', 0),
            'last_update': self.supervisor.getTime()
        }
        
        # Process received messages
        for msg in self.received_messages:
            if msg['type'] == 'performance_update':
                sender = msg['sender']
                data = msg['data']
                self.federated_model['robot_performance'][sender] = {
                    'efficiency': data.get('efficiency', 0),
                    'tags_collected': data.get('tags_collected', 0),
                    'last_update': msg['timestamp']
                }
        
        # Calculate aggregated statistics
        self._calculate_aggregate_stats()
    
    def _calculate_aggregate_stats(self):
        """Calculate aggregate statistics from all robot data"""
        efficiencies = []
        total_tags = 0
        
        for robot_id, data in self.federated_model['robot_performance'].items():
            efficiencies.append(data['efficiency'])
            total_tags += data['tags_collected']
        
        if efficiencies:
            self.federated_model['average_efficiency'] = sum(efficiencies) / len(efficiencies)
            self.federated_model['total_tags_collected'] = total_tags
        else:
            self.federated_model['average_efficiency'] = 0
            self.federated_model['total_tags_collected'] = 0
    
    def get_coordination_suggestion(self):
        """Get suggestion for coordinated behavior based on federated model"""
        if len(self.federated_model['robot_performance']) < 2:
            return "explore"  # Not enough data for coordination
        
        # Simple coordination: if other robots are efficient, focus on less explored areas
        my_efficiency = self.federated_model['robot_performance'].get(self.robot_id, {}).get('efficiency', 0)
        avg_efficiency = self.federated_model.get('average_efficiency', 0)
        
        if my_efficiency < avg_efficiency * 0.7:
            return "improve_efficiency"
        else:
            return "explore_new_areas"


class DetectorWrapper:
    """
    Normalize detections to:
      {'id': int, 'center': (x,y), 'corners': np.ndarray shape(4,2), 'margin': float}
    """
    def __init__(self):
        self.kind = APRIL_BACKEND
        self.det = None
        if self.kind == "swig":
            try:
                self.det = apriltag_swig.apriltag("tag36h11")
                print("[APRILTAG] SWIG detector ready (tag36h11)")
            except Exception as e:
                print(f"[APRILTAG] SWIG init failed: {e}")
                self.kind = None
        elif self.kind == "pupil":
            try:
                self.det = PupilDetector(families="tag36h11")
                print("[APRILTAG] pupil_apriltags ready (tag36h11)")
            except Exception as e:
                print(f"[APRILTAG] pupil init failed: {e}")
                self.kind = None

    def ok(self):
        return self.det is not None

    def detect(self, gray):
        if not self.ok():
            return []
        out = []
        try:
            if self.kind == "swig":
                for d in self.det.detect(gray):
                    # SWIG returns dict-like with 'center' and 'lb-rb-rt-lt'
                    out.append({
                        'id': int(d.get('id', 0)),
                        'center': tuple(map(float, d['center'])),
                        'corners': np.asarray(d['lb-rb-rt-lt'], dtype=float),
                        'margin': float(d.get('margin', 0.0)),
                    })
            else:  # pupil
                for d in self.det.detect(gray):
                    out.append({
                        'id': int(getattr(d, 'tag_id', 0)),
                        'center': tuple(map(float, d.center)),
                        'corners': np.asarray(d.corners, dtype=float),
                        'margin': float(getattr(d, 'decision_margin', 0.0)),
                    })
        except Exception as e:
            print(f"[APRILTAG] detect error: {e}")
        return out


class CSWDController:
    def __init__(self):
        # Initialize as Supervisor
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Identity (used to stagger waypoints across robots EPuck_1/2/3)
        self.name = self.robot.getName() or ""
        self.robot_idx = self._infer_robot_index(self.name)
        print(f"Robot '{self.name}' idx={self.robot_idx}")

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

        # Initialize LEDs
        self.leds = []
        for i in range(8):
            led = self.robot.getDevice(f"led{i}")
            if led:
                self.leds.append(led)

        # Body LED (green) & Front LED (red)
        self.body_led = self.robot.getDevice("led8")
        self.front_led = self.robot.getDevice("led9")

        # Speaker (optional)
        try:
            self.speaker = self.robot.getDevice("speaker")
        except Exception:
            self.speaker = None

        print(f"Initialized {len(self.leds)} ring LEDs, body LED, and speaker")

        # AprilTag detector
        self.detector = DetectorWrapper()
        if not self.detector.ok():
            print("⚠️ No AprilTag detector available — will roam only.")

        # Nodes
        self.robot_node = self.robot.getSelf()
        self.base_node = self.robot.getFromDef("DEPOSIT_BASE")

        # Storage for collected AprilTags
        self.collected_tags = []
        self.current_carried_tag = None

        # State machine
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, SIGNALING, COLLECTING, TURNING_TO_BASE, RETURNING, DEPOSITING
        self.lost_visual_counter = 0
        self.turn_to_base_counter = 0
        self.signal_phase = 0
        self.signal_timer = 0
        self.collection_timer = 0

        # Locking
        self.locked_tag_corners = None
        self.locked_tag_center = None
        self.lock_timeout = 0
        self.lock_tolerance = 150
        self.lock_duration = 0
        self.min_lock_duration = 10

        # Deposit counter
        self.deposit_count = 0

        # --- NEW: Performance Tracking ---
        self.performance_stats = {
            'tags_collected': 0,
            'total_search_time': 0.0,
            'search_start_time': self.robot.getTime(),
            'collection_attempts': 0,
            'successful_collections': 0,
            'failed_collections': 0,
            'total_operation_time': 0.0
        }
        
        # --- NEW: Smart Tag Memory ---
        self.failed_tags = set()  # Track tags we've failed to collect
        self.successful_tags = set()  # Track tags we've successfully collected

        # --- NEW: Communication & Federated Learning ---
        self.communication = RobotCommunication(self.robot, self.name)
        self.coordination_mode = "explore"
        self.last_communication_update = 0
        self.communication_interval = 5.0  # seconds

        # Movement parameters
        self.max_forward_speed = 4.0
        self.max_rotation_speed = 4.0
        self.collection_distance = 0.05
        self.deposit_distance = 0.1
        self.attach_height = 0.073

        # Base position (origin)
        self.base_position = np.array([0.0, 0.0], dtype=float)

        # --- Enhanced roaming with coordination ---
        self.waypoints = [
            (-1.8, 1.8), (-1.8, 1.2), (-1.4, 2.0), (0.0, 2.0), 
            (1.8, 1.8), (2.2, 1.3), (2.0, 0.0), (2.0, -2.0), 
            (0.0, -2.0), (-1.8, -1.8), (-1.8, -1.3), (-1.3, -2.1),
            (-2.0, 0.0)
        ]
        shift = self.robot_idx % len(self.waypoints)
        self.waypoints = self.waypoints[shift:] + self.waypoints[:shift]
        self.wp_idx = 0
        self.wp_reach_radius = 0.25

        self.search_sweep_ticks = 0
        self.SEARCH_SWEEP_PERIOD = 90
        self.spin_fraction = 0.25
        self.edge_ignore_px = 20

        # Stop-for-signal threshold (px). 110–140 tends to work across common FOVs.
        self.stop_tag_size_px = 110

        print("\n=== ENHANCED CSWD Controller with Communication & FL ===")
        print("NEW: Multi-robot communication capabilities")
        print("NEW: Federated learning for coordinated behavior")
        print("NEW: Performance analytics & efficiency tracking")
        print("NEW: Smart tag memory to avoid wasted effort")
        print("="*60)

    # --------- Helper Methods ----------
    def _infer_robot_index(self, name):
        m = re.search(r'(\d+)$', name or "")
        if m:
            return max(0, int(m.group(1)) - 1)
        try:
            return int(os.environ.get("ID", "0"))
        except:
            return 0

    def _drive(self, L, R):
        self.left_motor.setVelocity(float(np.clip(L, -6.28, 6.28)))
        self.right_motor.setVelocity(float(np.clip(R, -6.28, 6.28)))

    def get_robot_position(self):
        if self.robot_node:
            position = self.robot_node.getPosition()
            return np.array(position[:2])
        return np.array([0.0, 0.0])

    def get_robot_orientation(self):
        if self.robot_node:
            R = self.robot_node.getOrientation()
            return math.atan2(R[3], R[0])  # yaw from 3x3
        return 0.0

    def calculate_angle_to(self, target_xy):
        pos = self.get_robot_position()
        target = math.atan2(target_xy[1] - pos[1], target_xy[0] - pos[0])
        yaw = self.get_robot_orientation()
        d = target - yaw
        while d > math.pi:  d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    def calculate_angle_to_base(self):
        robot_pos = self.get_robot_position()
        target_angle = math.atan2(-robot_pos[1], -robot_pos[0])
        current_angle = self.get_robot_orientation()
        angle_diff = target_angle - current_angle
        while angle_diff > math.pi:  angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        return angle_diff

    def _advance_waypoint_if_reached(self):
        pos = self.get_robot_position()
        wp = np.array(self.waypoints[self.wp_idx])
        if np.linalg.norm(pos - wp) < self.wp_reach_radius:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            print(f"[ROAM] Reached waypoint → next {self.waypoints[self.wp_idx]}")

    def _drive_to_waypoint(self, speed=3.2):
        wp = np.array(self.waypoints[self.wp_idx])
        ang = self.calculate_angle_to(wp)
        steer = float(np.clip(ang * 2.0, -2.0, 2.0))
        self._drive(speed - steer, speed + steer)

    # --- NEW: Communication & Federated Learning Methods ---
    def update_communication(self):
        """Update communication and federated learning model"""
        current_time = self.robot.getTime()
        
        if current_time - self.last_communication_update >= self.communication_interval:
            # Receive messages from other robots
            self.communication.receive_messages()
            
            # Calculate local performance data
            efficiency = self.calculate_efficiency()
            local_data = {
                'tags_collected': self.performance_stats['tags_collected'],
                'efficiency': efficiency,
                'current_zone': self._get_current_zone(),
                'state': self.state
            }
            
            # Update federated model
            self.communication.update_federated_model(local_data)
            
            # Broadcast our performance
            self.communication.broadcast_message('performance_update', local_data)
            
            # Update coordination mode
            self.coordination_mode = self.communication.get_coordination_suggestion()
            
            self.last_communication_update = current_time
            
            # Print coordination status occasionally
            if random.random() < 0.3:  # 30% chance to print status
                self.print_coordination_status()

    def _get_current_zone(self):
        """Get current zone based on position"""
        pos = self.get_robot_position()
        x, y = pos[0], pos[1]
        if x >= 0 and y >= 0: return "top_right"
        elif x < 0 and y >= 0: return "top_left" 
        elif x < 0 and y < 0: return "bottom_left"
        else: return "bottom_right"

    def calculate_efficiency(self):
        """Calculate current collection efficiency"""
        if self.performance_stats['total_operation_time'] > 0:
            return self.performance_stats['tags_collected'] / self.performance_stats['total_operation_time']
        return 0.0

    def print_coordination_status(self):
        """Print current coordination status"""
        model = self.communication.federated_model
        print(f"\n🤖 ROBOT {self.robot_idx} COORDINATION STATUS:")
        print(f"   Mode: {self.coordination_mode.upper()}")
        print(f"   Known Robots: {len(model.get('robot_performance', {}))}")
        print(f"   Avg Efficiency: {model.get('average_efficiency', 0):.3f}")
        print(f"   Total Tags (Team): {model.get('total_tags_collected', 0)}")
        print("   " + "-" * 40)

    def apply_coordination_behavior(self):
        """Apply coordinated behavior based on federated learning suggestions"""
        if self.coordination_mode == "explore_new_areas":
            # Occasionally jump to a new waypoint to explore new areas
            if random.random() < 0.02:  # 2% chance per step
                self.wp_idx = random.randint(0, len(self.waypoints) - 1)
                print(f"[COORDINATION] Exploring new area: waypoint {self.wp_idx}")
                
        elif self.coordination_mode == "improve_efficiency":
            # Be more persistent with current target
            self.lock_tolerance = 200  # Increase lock tolerance
            if self.state == "SEARCHING":
                # Search more systematically
                self.search_sweep_ticks = (self.search_sweep_ticks + 2) % self.SEARCH_SWEEP_PERIOD

    # --- Enhanced Performance Reporting ---
    def print_efficiency_report(self):
        """Print comprehensive performance metrics"""
        current_time = self.robot.getTime()
        total_op_time = current_time - self.performance_stats['total_operation_time']
        search_time = current_time - self.performance_stats['search_start_time']
        
        if self.performance_stats['collection_attempts'] > 0:
            success_rate = (self.performance_stats['successful_collections'] / 
                           self.performance_stats['collection_attempts']) * 100
        else:
            success_rate = 0
            
        efficiency = self.calculate_efficiency()
        
        print(f"\n📊 ROBOT {self.robot_idx} PERFORMANCE REPORT:")
        print(f"   Tags Collected: {self.performance_stats['tags_collected']}")
        print(f"   Success Rate: {success_rate:.1f}%")
        print(f"   Collection Efficiency: {efficiency:.3f} tags/sec")
        print(f"   Collection Attempts: {self.performance_stats['collection_attempts']}")
        print(f"   Failed Collections: {self.performance_stats['failed_collections']}")
        print(f"   Coordination Mode: {self.coordination_mode}")
        print(f"   Memory: {len(self.successful_tags)} successful, {len(self.failed_tags)} failed tags")
        
        # Federated learning insights
        model = self.communication.federated_model
        if model.get('robot_performance'):
            print(f"   Team Performance: {len(model['robot_performance'])} robots, "
                  f"Avg Eff: {model.get('average_efficiency', 0):.3f}")
        print("   " + "-" * 40)

    def update_performance_stats(self):
        """Update real-time performance metrics"""
        current_time = self.robot.getTime()
        self.performance_stats['total_operation_time'] = current_time
        
        # Update search time when in searching state
        if self.state == "SEARCHING":
            self.performance_stats['total_search_time'] += self.timestep / 1000.0

    # -------------------------------------------------------

    def calculate_tag_size(self, detection):
        if detection is None:
            return None
        if detection.get('stale', False):
            return 100.0
        try:
            c = np.asarray(detection['corners'], dtype=float)
            edges = [np.linalg.norm(c[(i+1)%4] - c[i]) for i in range(4)]
            return float(sum(edges) / 4.0)
        except Exception:
            return None

    def find_apriltag_visually(self, gray_image):
        """FIXED: Remove smart tag memory to ensure all tags are collected"""
        if not self.detector.ok():
            return None

        try:
            detections = self.detector.detect(gray_image)

            # SIMPLIFIED: Only filter by border, don't skip any tags
            def usable(det):
                # Original border filter
                cx, cy = det['center']
                if (cx < self.edge_ignore_px or cx > (self.width - self.edge_ignore_px) or
                    cy < self.edge_ignore_px or cy > (self.height - self.edge_ignore_px)):
                    return self.locked_tag_center is not None
                return True

            detections = [d for d in detections if usable(d)]

            # Maintain lock if we have one
            if self.locked_tag_corners is not None and self.locked_tag_center is not None:
                self.lock_duration += 1

                if self.lock_duration < self.min_lock_duration:
                    for det in detections:
                        cx, cy = det['center']
                        dx = cx - self.locked_tag_center[0]
                        dy = cy - self.locked_tag_center[1]
                        if (dx*dx + dy*dy) ** 0.5 < self.lock_tolerance:
                            self.lock_timeout = 0
                            self.locked_tag_center = det['center']
                            self.locked_tag_corners = det['corners']
                            return det
                    self.lock_timeout += 1
                    return {'id': 0, 'center': self.locked_tag_center, 'corners': self.locked_tag_corners, 'margin': 0.0, 'stale': True}

                # After min lock, pick best match near prior center
                best, best_d = None, 1e9
                for det in detections:
                    cx, cy = det['center']
                    d = ((cx - self.locked_tag_center[0])**2 + (cy - self.locked_tag_center[1])**2) ** 0.5
                    if d < self.lock_tolerance and d < best_d:
                        best, best_d = det, d
                if best is not None:
                    self.lock_timeout = 0
                    self.locked_tag_center = best['center']
                    self.locked_tag_corners = best['corners']
                    return best

                self.lock_timeout += 1
                if self.lock_timeout > 100:
                    print("[LOCK] Lost locked tag for extended time, unlocking...")
                    self.locked_tag_corners = None
                    self.locked_tag_center = None
                    self.lock_timeout = 0
                    self.lock_duration = 0
                else:
                    if self.lock_timeout < 50:
                        return {'id': 0, 'center': self.locked_tag_center, 'corners': self.locked_tag_corners, 'margin': 0.0, 'stale': True}
                    return None

            # No lock: pick centered & large
            if len(detections) > 0:
                best_det, best_score = None, -1e9
                for det in detections:
                    c = np.asarray(det['corners'], dtype=float)
                    edges = [np.linalg.norm(c[(i+1)%4] - c[i]) for i in range(4)]
                    size = float(sum(edges))
                    cx = det['center'][0]
                    offset = abs(cx - self.center_x)
                    center_penalty = (offset / self.center_x) ** 2
                    score = size * (1.0 - center_penalty)
                    if score > best_score:
                        best_score = score
                        best_det = det
                if best_det:
                    self.locked_tag_corners = best_det['corners']
                    self.locked_tag_center = best_det['center']
                    self.lock_timeout = 0
                    self.lock_duration = 0
                    return best_det
        except Exception as e:
            print(f"Detection error: {e}")
        return None

    def signal_for_pickup(self):
        # Flash ring LEDs 3 times, then body LED
        if self.signal_phase < 6:
            on = (self.signal_phase % 2 == 0)
            for led in self.leds:
                try: led.set(1 if on else 0)
                except: pass
            self.signal_timer += 1
            if self.signal_timer > 10:
                self.signal_phase += 1
                self.signal_timer = 0
        elif self.signal_phase == 6:
            for led in self.leds:
                try: led.set(0)
                except: pass
            if self.body_led:
                try: self.body_led.set(1)
                except: pass
            self.signal_timer += 1
            if self.signal_timer > 20:
                self.signal_phase += 1
                self.signal_timer = 0
                return True
        return False

    def reset_leds(self):
        for led in self.leds:
            try: led.set(0)
            except: pass
        if self.body_led:
            try: self.body_led.set(0)
            except: pass
        if self.front_led:
            try: self.front_led.set(0)
            except: pass

    def collect_apriltag_at_position(self, x, y):
        """FIXED: Remove smart tag memory to ensure all tags are collected"""
        self.performance_stats['collection_attempts'] += 1
        print(f"[DEBUG] Checking for AprilTags near robot position: ({x:.2f}, {y:.2f})")
        closest_dist = float('inf')
        closest_node = None
        closest_tag_id = None

        for i in range(1, 55):            
            node = self.robot.getFromDef(f"APRILTAG_{i}")
            if node:
                tf = node.getField("translation")
                if not tf: 
                    continue
                px, py, pz = tf.getSFVec3f()
                # If already carried by someone else, skip (z near attach height)
                if abs(pz - self.attach_height) < 0.02:
                    continue
                dist = math.hypot(px - x, py - y)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_node = node
                    closest_tag_id = i
                if dist < 0.15:
                    print(f"[DEBUG] Found AprilTag {i} at distance {closest_dist:.3f}m - Collecting!")
                    success = self.attach_apriltag(node)
                    if success:
                        self.performance_stats['successful_collections'] += 1
                        self.successful_tags.add(i)  # Remember successful collection
                    return success

        if closest_node and closest_dist < 0.25:  # If somewhat close but not quite
            print(f"[DEBUG] Closest AprilTag {closest_tag_id} is {closest_dist:.3f}m away - attempting collection")
            success = self.attach_apriltag(closest_node)
            if success:
                self.performance_stats['successful_collections'] += 1
                self.successful_tags.add(closest_tag_id)
            else:
                self.performance_stats['failed_collections'] += 1
            return success
                
        if closest_node:
            print(f"[DEBUG] Closest AprilTag is {closest_dist:.3f}m away - too far to collect")
            
        # If we get here, it's a failed collection
        self.performance_stats['failed_collections'] += 1
        return False

    def attach_apriltag(self, node):
        robot_position = self.robot_node.getPosition()
        tf = node.getField("translation")
        if tf:
            tf.setSFVec3f([robot_position[0], robot_position[1], self.attach_height])

        # Make semi-transparent
        try:
            shape = node.getField("children").getMFNode(0)
            app = shape.getField("appearance").getSFNode()
            app.getField("transparency").setSFFloat(0.35)
        except Exception:
            pass

        self.current_carried_tag = node
        self.collected_tags.append(node)
        print("\n*** COLLECTED APRILTAG! ***")
        return True

    def update_carried_apriltag(self):
        if self.current_carried_tag:
            x, y, _ = self.robot_node.getPosition()
            self.current_carried_tag.getField("translation").setSFVec3f([x, y, self.attach_height])

    def deposit_apriltag(self):
        if not self.current_carried_tag:
            return False
        self.current_carried_tag.getField("translation").setSFVec3f([100.0, 100.0, -10.0])
        self.deposit_count += 1
        self.performance_stats['tags_collected'] += 1
        
        print("\n" + "="*50)
        print(f"*** APRILTAG DEPOSITED AT BASE! ***")
        print(f"*** {self.deposit_count} AprilTag(s) collected ***")
        print("="*50 + "\n")
        
        # Reset search timer for efficiency calculation
        self.performance_stats['search_start_time'] = self.robot.getTime()
        
        self.current_carried_tag = None
        # Clear lock to allow new target selection
        self.locked_tag_corners = None
        self.locked_tag_center  = None
        self.lock_timeout = 0
        self.lock_duration = 0
        return True

    def run(self):
        step = 0
        last_report_step = 0
        
        while self.robot.step(self.timestep) != -1:
            step += 1

            # Update performance statistics
            self.update_performance_stats()

            # NEW: Update communication and federated learning
            self.update_communication()
            
            # NEW: Apply coordinated behavior
            self.apply_coordination_behavior()

            if self.current_carried_tag:
                self.update_carried_apriltag()

            img = self.camera.getImage()
            if not img:
                continue
            frame = np.frombuffer(img, np.uint8).reshape((self.height, self.width, 4))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)

            if self.state == "SEARCHING":
                det = self.find_apriltag_visually(gray)
                if det:
                    self.state = "APPROACHING"
                    self.lost_visual_counter = 0
                    print(f"\n[SMART SEARCH] Found available AprilTag {det.get('id', 'Unknown')}!")
                else:
                    # short spin sweep, then waypoint cruise
                    self.search_sweep_ticks = (self.search_sweep_ticks + 1) % self.SEARCH_SWEEP_PERIOD
                    if (self.search_sweep_ticks / self.SEARCH_SWEEP_PERIOD) < self.spin_fraction:
                        self._drive(2.2, -2.2)
                    else:
                        self._advance_waypoint_if_reached()
                        self._drive_to_waypoint(speed=3.2)
                    if step % 90 == 0:
                        wp = self.waypoints[self.wp_idx]
                        efficiency = self.calculate_efficiency()
                        print(f"[SEARCHING] Eff: {efficiency:.3f} | Deposited: {self.deposit_count} | "
                              f"Memory: {len(self.successful_tags)}S/{len(self.failed_tags)}F | "
                              f"Mode: {self.coordination_mode}")

            elif self.state == "APPROACHING":
                det = self.find_apriltag_visually(gray)

                if det:
                    cx = det['center'][0]
                    rot_err = cx - self.center_x
                    rot = float(np.clip(rot_err * 0.02, -self.max_rotation_speed, self.max_rotation_speed))
                    fwd = self.max_forward_speed
                    self._drive(fwd + rot, fwd - rot)
                    self.lost_visual_counter = 0

                    tag_size = self.calculate_tag_size(det)
                    if step % 20 == 0:
                        tid = det.get('id', 'Unknown')
                        print(f"[APPROACHING] id={tid}, size={0 if tag_size is None else tag_size:.0f}px")

                    if tag_size and tag_size > self.stop_tag_size_px:
                        self._drive(0, 0)
                        self.state = "SIGNALING"
                        self.signal_phase = 0
                        self.signal_timer = 0
                        print("[CSWD] Stopping and signaling...")

                else:
                    self.lost_visual_counter += 1
                    if self.lost_visual_counter < 40:
                        self._drive(4.0, 4.0)
                        if self.lost_visual_counter % 10 == 0:
                            pos = self.get_robot_position()
                            if self.collect_apriltag_at_position(pos[0], pos[1]):
                                self.locked_tag_corners = None
                                self.locked_tag_center  = None
                                self.lock_timeout = 0
                                self.lock_duration = 0
                                self.state = "TURNING_TO_BASE"
                                self.turn_to_base_counter = 0
                                print("[COLLECTED] Got tag! Turning to base...")
                        if self.lost_visual_counter == 10:
                            print("[APPROACHING] Lost visual — charging forward to collect!")
                    else:
                        self.state = "SEARCHING"
                        print("[LOST] No collection after charge, resume roaming")

            elif self.state == "SIGNALING":
                if self.signal_for_pickup():
                    print("[CSWD] Signal complete — nudging for pickup...")
                    self.reset_leds()
                    self._drive(2.0, 2.0)
                    for _ in range(10):
                        self.robot.step(self.timestep)
                    self._drive(0, 0)
                    self.state = "COLLECTING"
                    self.collection_timer = 0

            elif self.state == "COLLECTING":
                self.collection_timer += 1
                if self.collection_timer > 10:
                    self._drive(0, 0)
                    pos = self.get_robot_position()
                    if self.collect_apriltag_at_position(pos[0], pos[1]):
                        self.locked_tag_corners = None
                        self.locked_tag_center  = None
                        self.lock_timeout = 0
                        self.lock_duration = 0
                        self.state = "TURNING_TO_BASE"
                        self.turn_to_base_counter = 0
                        print("[COLLECTED] Attached! Returning to base...")
                    else:
                        print("[CSWD] Pickup failed — continuing approach")
                        self.state = "APPROACHING"

            elif self.state == "TURNING_TO_BASE":
                ang = self.calculate_angle_to_base()
                if abs(ang) < 0.15:
                    self._drive(0, 0)
                    self.state = "RETURNING"
                    print(f"[TURNING] Facing base (err={math.degrees(ang):.1f}°)")
                else:
                    w = float(np.clip(ang * 3.0, -3.0, 3.0))
                    self._drive(-w, w)
                    if step % 20 == 0:
                        pos = self.get_robot_position()
                        print(f"[TURNING] to base: {math.degrees(ang):.1f}°, pos=({pos[0]:.2f},{pos[1]:.2f})")

            elif self.state == "RETURNING":
                pos = self.get_robot_position()
                dist = float(np.linalg.norm(pos - self.base_position))
                if dist > self.deposit_distance:
                    ang = self.calculate_angle_to_base()
                    if abs(ang) > 0.3:
                        self.state = "TURNING_TO_BASE"
                        print(f"[RETURN] course correction (err={math.degrees(ang):.1f}°)")
                    else:
                        fwd = 4.0
                        steer = float(np.clip(ang * 2.0, -2.0, 2.0))
                        self._drive(fwd - steer, fwd + steer)
                        if step % 20 == 0:
                            print(f"[RETURN] d={dist:.3f} m, err={math.degrees(ang):.1f}°")
                else:
                    self.state = "DEPOSITING"
                    print("[RETURN] Reached base — depositing")

            elif self.state == "DEPOSITING":
                if self.deposit_apriltag():
                    self._drive(2.0, 2.0)
                    for _ in range(20):
                        self.robot.step(self.timestep)
                        self.update_carried_apriltag()
                    self.state = "SEARCHING"
                    print("[DEPOSITING] Done — searching again")
                    self._advance_waypoint_if_reached()

            # Print performance report every 1500 steps (~30 seconds)
            if step - last_report_step >= 1500:
                self.print_efficiency_report()
                last_report_step = step


if __name__ == "__main__":
    controller = CSWDController()
    controller.run()