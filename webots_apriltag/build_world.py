#!/usr/bin/env python3
"""Build the new world file with 5x5 clusters"""

# World file header and base objects
header = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {
  title "AprilTag Collect and Deposit V2"
  basicTimeStep 32
}

Viewpoint {
  orientation -0.35 0.35 0.87 1.4
  position 0.7 -0.7 1.2
  follow "collector_depositor_robot"
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
        baseColor 0.85 0.85 0.85
        roughness 1
        metalness 0
      }
      geometry Box {
        size 5 5 0.001
      }
    }
  ]
  boundingObject Box {
    size 5 5 0.001
  }
}

# Deposit Base (cylinder in center - no physics for phasing through)
DEF DEPOSIT_BASE Solid {
  translation 0 0 0.05
  name "deposit_base"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.2 0.6 0.9
        roughness 0.5
        metalness 0.3
        emissiveColor 0.1 0.3 0.5
        emissiveIntensity 0.3
        transparency 0.35  # 65% opaque
      }
      geometry Cylinder {
        radius 0.2
        height 0.1
      }
    }
    # Visual ring around base
    Transform {
      translation 0 0 -0.049
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 1 0.8 0.2
            roughness 0.3
            metalness 0.5
            transparency 0.35  # 65% opaque
          }
          geometry Cylinder {
            radius 0.25
            height 0.002
          }
        }
      ]
    }
  ]
  # No boundingObject - robot can phase through
}

"""

# Read the generated clusters
with open('/home/andres2020/Dev/RL-FL-Research/webots_apriltag/clusters_5x5.txt', 'r') as f:
    clusters = f.read()

# Robot definition
robot = """
# E-puck robot with supervisor privileges
E-puck {
  translation -0.5 0 0
  name "collector_depositor_robot"
  controller "apriltag_collector_depositor_v2"
  supervisor TRUE
  camera_width 640
  camera_height 480
  camera_fieldOfView 1.0
}
"""

# Write the complete world file
with open('/home/andres2020/Dev/RL-FL-Research/webots_apriltag/worlds/apriltag_collect_and_deposit_v2.wbt', 'w') as f:
    f.write(header)
    f.write(clusters)
    f.write(robot)

print("World file created: apriltag_collect_and_deposit_v2.wbt")