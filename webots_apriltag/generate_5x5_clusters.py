#!/usr/bin/env python3
"""Generate 5x5 AprilTag clusters for Webots world file"""

def generate_apriltag(tag_id, x, y, z=0.0275):
    """Generate a single AprilTag definition"""
    return f"""DEF APRILTAG_{tag_id} Solid {{
  translation {x} {y} {z}
  name "apriltag_{tag_id}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{
          url ["../textures/real_tag36h11_id0.png"]
        }}
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size 0.055 0.055 0.055
      }}
    }}
  ]
}}
"""

def generate_5x5_cluster(start_id, center_x, center_y, spacing=0.12):
    """Generate a 5x5 cluster of AprilTags"""
    result = ""
    tag_id = start_id

    # 5x5 grid: from -2 to +2 relative to center
    for row in range(-2, 3):  # -2, -1, 0, 1, 2
        for col in range(-2, 3):  # -2, -1, 0, 1, 2
            x = center_x + col * spacing
            y = center_y + row * spacing
            result += generate_apriltag(tag_id, x, y)
            result += "\n"
            tag_id += 1

    return result, tag_id

# Generate first cluster (top-left area)
print("# First 5x5 cluster of AprilTags (top-left area)")
print("# Cluster center at (-1.3, 1.3), spacing 0.12m")
cluster1, next_id = generate_5x5_cluster(1, -1.3, 1.3, 0.12)
print(cluster1)

# Generate second cluster (bottom-right area)
print("# Second 5x5 cluster of AprilTags (bottom-right area)")
print("# Cluster center at (1.3, -1.3), spacing 0.12m")
cluster2, final_id = generate_5x5_cluster(next_id, 1.3, -1.3, 0.12)
print(cluster2)

print(f"# Total AprilTags generated: {final_id - 1}")