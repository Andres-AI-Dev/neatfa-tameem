#!/usr/bin/env python3
"""
Simple AprilTag detector for e-puck robot
Detects black and white patterns resembling AprilTags
"""

from controller import Robot
import numpy as np
import cv2

class AprilTagDetector:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        print(f"AprilTag Detector initialized")
        print(f"Camera: {self.width}x{self.height}")

    def detect_tags(self, image):
        """Simple tag detection based on contours"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tags = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if 500 < area < 50000:
                x, y, w, h = cv2.boundingRect(contour)
                aspect = w / h if h > 0 else 0
                if 0.7 < aspect < 1.3:  # Roughly square
                    tags.append({
                        'center': (x + w//2, y + h//2),
                        'size': (w, h)
                    })
        return tags

    def run(self):
        """Main loop"""
        step = 0

        # Simple rotation to scan environment
        self.left_motor.setVelocity(2.0)
        self.right_motor.setVelocity(-2.0)

        while self.robot.step(self.timestep) != -1:
            step += 1

            if step % 10 == 0:  # Process every 10 steps
                image_data = self.camera.getImage()
                if image_data:
                    image = np.frombuffer(image_data, np.uint8).reshape(
                        (self.height, self.width, 4))
                    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

                    tags = self.detect_tags(image_bgr)

                    if tags:
                        print(f"[Step {step}] Detected {len(tags)} tag(s)")
                        for i, tag in enumerate(tags):
                            print(f"  Tag {i}: center={tag['center']}, size={tag['size']}")

if __name__ == "__main__":
    detector = AprilTagDetector()
    detector.run()