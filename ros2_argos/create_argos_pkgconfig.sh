#!/bin/bash

# Create pkg-config file for ARGoS3
echo "Creating ARGoS3 pkg-config file..."

sudo mkdir -p /usr/local/lib/pkgconfig

sudo tee /usr/local/lib/pkgconfig/argos3_simulator.pc > /dev/null <<EOF
prefix=/usr/local
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib/argos3
includedir=\${prefix}/include

Name: argos3_simulator
Description: ARGoS3 multi-robot simulator
Version: 3.0.0-beta59
Libs: -L\${libdir} -largos3core_simulator -largos3plugin_simulator_dynamics2d -largos3plugin_simulator_entities -largos3plugin_simulator_footbot -largos3plugin_simulator_genericrobot -largos3plugin_simulator_media -largos3plugin_simulator_qtopengl -Wl,-rpath,\${libdir}
Cflags: -I\${includedir}
EOF

echo "ARGoS3 pkg-config file created successfully!"
echo "Testing pkg-config..."
pkg-config --modversion argos3_simulator
pkg-config --cflags argos3_simulator
pkg-config --libs argos3_simulator