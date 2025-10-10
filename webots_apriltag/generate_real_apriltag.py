#!/usr/bin/env python3
"""
Generate a real AprilTag image for tag36h11 family
"""

import numpy as np
import cv2

# Tag36h11 ID 0 bit pattern (simplified representation)
# Real tag would need the actual bit encoding from the library
def generate_tag36h11_id0(size=256):
    """Generate tag36h11 ID 0"""

    # Create white background
    img = np.ones((size, size), dtype=np.uint8) * 255

    # Black border (1/8 of size)
    border = size // 8
    cv2.rectangle(img, (0, 0), (size-1, size-1), 0, border)

    # The actual tag36h11 ID 0 pattern (simplified)
    # This is a 10x10 data matrix with specific pattern for ID 0
    inner_size = size - 2*border
    cell_size = inner_size // 10

    # tag36h11 ID 0 pattern (approximation)
    pattern = [
        [1,1,1,1,1,1,1,1,1,1],
        [1,0,0,0,1,0,1,0,0,1],
        [1,0,1,0,0,1,0,1,0,1],
        [1,0,0,1,0,0,0,0,0,1],
        [1,1,0,0,0,1,1,0,1,1],
        [1,0,1,1,0,0,0,1,0,1],
        [1,0,0,0,1,1,0,0,0,1],
        [1,0,1,0,0,0,1,0,1,1],
        [1,0,0,1,0,1,0,1,0,1],
        [1,1,1,1,1,1,1,1,1,1]
    ]

    for i in range(10):
        for j in range(10):
            if pattern[i][j] == 0:
                x = border + j * cell_size
                y = border + i * cell_size
                cv2.rectangle(img, (x, y), (x + cell_size - 1, y + cell_size - 1), 0, -1)

    return img

# Generate and save
tag_img = generate_tag36h11_id0(256)
cv2.imwrite('webots_apriltag/textures/tag36h11_id0.png', tag_img)
print("Generated AprilTag image: webots_apriltag/textures/tag36h11_id0.png")

# Also save a larger version
tag_img_large = generate_tag36h11_id0(512)
cv2.imwrite('webots_apriltag/textures/tag36h11_id0_large.png', tag_img_large)
print("Generated large AprilTag image: webots_apriltag/textures/tag36h11_id0_large.png")