#!/usr/bin/env python3
"""Generate distance test world for AprilTag detection range testing"""

def generate_header():
    """Generate world file header"""
    return """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {
  title "AprilTag Distance Detection Test"
  basicTimeStep 32
}

Viewpoint {
  orientation -0.5 0.5 0.7 1.2
  position 0 -2 4
}

TexturedBackground {
}

TexturedBackgroundLight {
}

# Floor
Solid {
  name "floor"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.9 0.2 0.2
        roughness 1
        metalness 0
      }
      geometry Box {
        size 12 8 0.001
      }
    }
  ]
  boundingObject Box {
    size 12 8 0.001
  }
}

"""

def generate_wall(name, x, y, width, height):
    """Generate a wall segment"""
    return f"""Solid {{
  translation {x} {y} 0.15
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.2 0.2
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {width} {height} 0.3
      }}
    }}
  ]
  boundingObject Box {{
    size {width} {height} 0.3
  }}
}}

"""

def generate_apriltag(tag_id, x, y, z=0.0275):
    """Generate a single AprilTag"""
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

def generate_robot(name, x, y, controller="apriltag_distance_test"):
    """Generate an E-puck robot facing forward (toward positive Y)"""
    return f"""E-puck {{
  translation {x} {y} 0
  rotation 0 0 1 1.5708
  name "{name}"
  controller "{controller}"
  supervisor TRUE
  camera_width 640
  camera_height 480
  camera_fieldOfView 1.0
}}

"""

def generate_distance_label(distance, x, y):
    """Generate a distance label (as a flat colored box on the floor)"""
    return f"""# Distance marker: {distance:.1f}m
Solid {{
  translation {x} {y} 0.002
  name "label_{int(distance*100)}cm"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.2 0.8
        roughness 1
        metalness 0
        emissiveColor 0.1 0.1 0.4
        emissiveIntensity 0.2
      }}
      geometry Box {{
        size 0.15 0.05 0.001
      }}
    }}
  ]
}}

"""

# Main generation
world_content = generate_header()

# Test configuration
num_lanes = 20  # Number of test lanes
lane_width = 0.5  # Width of each lane (reduced for more lanes)
total_width = 12.0  # Total width of the world (increased for 20 lanes)
start_distance = 1.0  # Starting distance in meters (testing further range)
distance_increment = 0.1  # Distance increment per lane (testing up to 2.9m)
wall_thickness = 0.03  # Thinner walls

# Calculate lane positions
lane_spacing = total_width / (num_lanes + 1)
start_x = -total_width/2 + lane_spacing

# Generate lanes
for lane_idx in range(num_lanes):
    x_pos = start_x + lane_idx * lane_spacing
    distance = start_distance + lane_idx * distance_increment

    # Add distance label on the floor
    world_content += generate_distance_label(distance, x_pos, 1.5)

    # Add walls on both sides of the lane (except for the outer edges)
    if lane_idx > 0:  # Left wall
        wall_x = x_pos - lane_spacing/2
        world_content += generate_wall(
            f"wall_left_{lane_idx}",
            wall_x, 0, wall_thickness, 7.0  # Extra long walls for longer test range
        )

    if lane_idx < num_lanes - 1:  # Right wall
        wall_x = x_pos + lane_spacing/2
        world_content += generate_wall(
            f"wall_right_{lane_idx}",
            wall_x, 0, wall_thickness, 7.0  # Extra long walls for longer test range
        )

    # Add robot at one end
    robot_y = -3.5  # Move robots back even further for longer distance range (up to 2.9m)
    world_content += generate_robot(f"robot_lane_{lane_idx}", x_pos, robot_y)

    # Add AprilTag at specified distance
    tag_y = robot_y + distance
    world_content += generate_apriltag(lane_idx + 1, x_pos, tag_y)

# Add outer walls (extra long walls to contain the lanes)
world_content += generate_wall("wall_outer_left", -6, 0, wall_thickness, 7.0)
world_content += generate_wall("wall_outer_right", 6, 0, wall_thickness, 7.0)

# Add front and back walls (adjusted for longer test range)
world_content += generate_wall("wall_back", 0, -3.8, 12, wall_thickness)
world_content += generate_wall("wall_front", 0, 2.4, 12, wall_thickness)

# Write the world file
output_path = '/home/andres2020/Dev/RL-FL-Research/webots_apriltag/worlds/apriltag_distance_detection_test.wbt'
with open(output_path, 'w') as f:
    f.write(world_content)

print(f"World file created: {output_path}")
print(f"Generated {num_lanes} test lanes:")
for i in range(num_lanes):
    distance = start_distance + i * distance_increment
    print(f"  Lane {i}: {distance:.1f}m")