#!/usr/bin/env python3
"""
AprilTag Distance Detection Test Controller
Tests AprilTag detection at various distances
"""

from controller import Robot, Camera, Supervisor
import numpy as np
import cv2
import os
import sys

# Add AprilTag library path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import apriltag

class AprilTagDistanceTest:
    def __init__(self):
        # Initialize supervisor robot
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Get robot name to identify which lane
        self.robot_name = self.robot.getName()
        self.lane_number = int(self.robot_name.split('_')[-1])
        self.test_distance = 1.0 + self.lane_number * 0.1  # Distance in meters (1.0m start + 0.1m increments)

        # Initialize camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        print(f"[Lane {self.lane_number}] Camera initialized: {self.camera.getWidth()}x{self.camera.getHeight()}")

        # Initialize AprilTag detector
        self.detector = apriltag.apriltag("tag36h11")

        # Test parameters
        self.test_frames = 30  # Number of frames to test
        self.frame_count = 0
        self.detection_count = 0
        self.sizes = []
        self.tested = False

    def process_image(self):
        """Process camera image for AprilTag detection"""
        # Get image from camera
        image_data = self.camera.getImageArray()
        if not image_data:
            return None

        # Convert to numpy array and correct format
        height = self.camera.getHeight()
        width = self.camera.getWidth()
        image = np.array(image_data, dtype=np.uint8)

        # Check if image has alpha channel
        if image.size == height * width * 4:
            image = image.reshape((height, width, 4))[:, :, :3]  # Remove alpha channel
        else:
            image = image.reshape((height, width, 3))  # Already RGB

        # Convert RGB to BGR for OpenCV
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Convert to grayscale for AprilTag detection
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        results = self.detector.detect(gray)

        # Debug: save first frame for debugging
        if self.frame_count == 0:
            import os
            debug_dir = "/tmp/apriltag_debug"
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(f"{debug_dir}/lane_{self.lane_number}_frame.jpg", image_bgr)
            print(f"[Lane {self.lane_number}] Saved debug image to {debug_dir}/lane_{self.lane_number}_frame.jpg")

        return results

    def run(self):
        """Main control loop"""
        print(f"[Lane {self.lane_number}] Testing detection at {self.test_distance:.1f}m")

        while self.robot.step(self.timestep) != -1:
            if self.tested:
                continue  # Keep running but don't test again

            # Process image and detect AprilTags
            results = self.process_image()

            if self.frame_count < self.test_frames:
                if results:
                    # Record detection
                    for detection in results:
                        # Calculate size (average of width and height)
                        # Detections are returned as dictionaries
                        try:
                            corners = detection['lb-rb-rt-lt']  # corners in dictionary format
                            width = np.linalg.norm(corners[1] - corners[0])
                            height = np.linalg.norm(corners[3] - corners[0])
                            size = (width + height) / 2
                            self.sizes.append(size)
                            self.detection_count += 1
                        except (KeyError, TypeError):
                            # Skip if corners not accessible
                            continue

                self.frame_count += 1

                # Report progress every 10 frames
                if self.frame_count % 10 == 0:
                    if self.sizes:
                        avg_size = np.mean(self.sizes)
                        print(f"[Lane {self.lane_number}] Frame {self.frame_count}/{self.test_frames}: "
                              f"Detected {len(self.sizes)} times, avg size: {avg_size:.1f} pixels")
                    else:
                        print(f"[Lane {self.lane_number}] Frame {self.frame_count}/{self.test_frames}: "
                              f"No detection")

            # Test complete
            if self.frame_count >= self.test_frames and not self.tested:
                self.tested = True

                # Calculate results
                detection_rate = (self.detection_count / self.test_frames) * 100
                avg_size = np.mean(self.sizes) if self.sizes else 0

                # Print results
                print(f"\n{'='*50}")
                print(f"[Lane {self.lane_number}] RESULTS at {self.test_distance:.1f}m:")
                print(f"  Detection rate: {detection_rate:.1f}% ({self.detection_count}/{self.test_frames} frames)")
                if self.sizes:
                    print(f"  Average tag size: {avg_size:.1f} pixels")
                    print(f"  Min size: {min(self.sizes):.1f} pixels")
                    print(f"  Max size: {max(self.sizes):.1f} pixels")
                else:
                    print(f"  No detections - tag too far or too small")
                print(f"{'='*50}\n")

if __name__ == "__main__":
    controller = AprilTagDistanceTest()
    controller.run()