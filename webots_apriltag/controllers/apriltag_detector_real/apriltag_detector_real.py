#!/usr/bin/env python3
"""
AprilTag detector with debugging
"""

from controller import Robot
import numpy as np
import cv2
import sys
import os

# Add the built AprilTag library to path
sys.path.insert(0, '../../../apriltag/build')

try:
    import apriltag
    APRILTAG_AVAILABLE = True
    print("AprilTag library loaded successfully!")
except ImportError as e:
    print(f"Warning: Could not import apriltag library: {e}")
    APRILTAG_AVAILABLE = False

class AprilTagDetectorReal:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        print(f"Camera initialized: {self.width}x{self.height}")

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        # Initialize AprilTag detector
        self.detector = None
        if APRILTAG_AVAILABLE:
            try:
                self.detector = apriltag.apriltag("tag36h11")
                print("AprilTag detector initialized with tag36h11 family")
            except Exception as e:
                print(f"Could not initialize apriltag detector: {e}")

        self.max_speed = 2.0
        self.detection_count = 0
        self.save_next_image = True

    def detect_apriltags(self, gray_image):
        """Detect AprilTags"""
        detections = []

        # Try real AprilTag detection
        if self.detector is not None:
            try:
                raw_detections = self.detector.detect(gray_image)

                # Debug: print what we got
                if len(raw_detections) > 0:
                    print(f"Raw detections: {raw_detections}")

                # Convert to our format
                for det in raw_detections:
                    detections.append({
                        'id': det['id'] if 'id' in det else -1,
                        'center': det.get('center', [0, 0]),
                        'corners': det.get('corners', [[0,0]]*4)
                    })
            except Exception as e:
                print(f"Detection error: {e}")

        # Fallback: Simple black square detection
        if len(detections) == 0:
            # Look for black squares
            _, binary = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if 500 < area < 50000:  # Reasonable size
                    x, y, w, h = cv2.boundingRect(contour)
                    if 0.7 < w/h < 1.3:  # Square-ish
                        detections.append({
                            'id': -99,  # Fallback detection
                            'center': [x + w/2, y + h/2],
                            'corners': [[x, y], [x+w, y], [x+w, y+h], [x, y+h]]
                        })

        return detections

    def run(self):
        """Main control loop"""
        step = 0

        print("\n=== Starting Detection ===")
        print("Robot will look forward first, then rotate if needed\n")

        # Start stationary to check what's in front
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        no_detection_count = 0

        while self.robot.step(self.timestep) != -1:
            step += 1

            # Process every 10 steps
            if step % 10 == 0:
                image_data = self.camera.getImage()
                if image_data:
                    # Convert to numpy array
                    image = np.frombuffer(image_data, np.uint8).reshape(
                        (self.height, self.width, 4))

                    # Convert to grayscale
                    gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)

                    # Save first few images to debug
                    if step <= 50 and step % 10 == 0:
                        filename = f"camera_view_{step:04d}.png"
                        cv2.imwrite(filename, gray)
                        print(f"Saved {filename} - Mean brightness: {gray.mean():.1f}")

                    # Detect
                    detections = self.detect_apriltags(gray)

                    if detections:
                        print(f"\n[Step {step}] DETECTED {len(detections)} tag(s)!")
                        for i, tag in enumerate(detections):
                            self.detection_count += 1
                            tag_type = "AprilTag" if tag['id'] >= 0 else "Black Square"
                            print(f"  {tag_type} {i}: ID={tag['id']}, Center={tag['center'][0]:.0f},{tag['center'][1]:.0f}")

                            # Save detection image
                            vis_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                            corners = np.array(tag['corners'], dtype=np.int32)
                            cv2.polylines(vis_img, [corners], True, (0, 255, 0), 2)

                            center = tuple(map(int, tag['center']))
                            cv2.circle(vis_img, center, 5, (0, 0, 255), -1)

                            detection_file = f"detection_{step:04d}.png"
                            cv2.imwrite(detection_file, vis_img)
                            print(f"  Saved: {detection_file}")

                        # Stop and look at the tag
                        self.left_motor.setVelocity(0)
                        self.right_motor.setVelocity(0)
                        no_detection_count = 0
                    else:
                        no_detection_count += 1
                        # Only start rotating after no detections for a while
                        if no_detection_count > 5:
                            self.left_motor.setVelocity(self.max_speed * 0.3)
                            self.right_motor.setVelocity(-self.max_speed * 0.3)

            # Status update
            if step % 100 == 0:
                print(f"Step {step}: Total detections = {self.detection_count}")

if __name__ == "__main__":
    detector = AprilTagDetectorReal()
    detector.run()