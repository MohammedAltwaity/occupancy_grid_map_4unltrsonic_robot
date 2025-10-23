Occupancy Grid Mapper
Builds 2D occupancy grid maps from robot poses + 4 ultrasonic ToF sensors using log-odds Bayesian updates
Quick Start
Compile (C++17):
g++ -std=c++17 main.cpp src/*.cpp -Iinclude -o occmap
Run - builds map from CSV to PGM image:
./occmap assets/robot.csv map.pgm
Input Format
CSV columns: timestamp,x,y,theta,tof0,tof1,tof2,tof3





















FieldDescriptionx,yRobot position (meters)thetaRobot heading (radians)tof[0-3]Time-of-flight (seconds) for 4 sensors
Sensor Layout:
us1 (front-left)    us0 (front-right)
↖️               ↗️
rear-left (us2)  ↔  rear-right (us3)
Key Features





























FeatureDescriptionLog-OddsBayesian occupancy updates (P_occ=70%, P_free=30%)Ray TracingDDA algorithm marks free space + occupied blobsSensor Model30° FOV cones, configurable ray countUncertaintySmears hits over 5cm radius, clips at 4mEfficiencySTRIDE=2 skips rows for fast processing
Tunable Parameters
const double RES        = 0.05;  // 5cm grid cells
const double MAP_SIZE_M = 20.0;  // 20×20m map
const int    N_RAYS     = 1;     // Rays per sensor cone
const double MAX_RANGE  = 4.0;   // Sensor max distance
const int    STRIDE     = 2;     // Process every 2nd pose
Files
main.cpp          # Core mapping algorithm
include/
├── grid.hpp      # Occupancy grid + DDA ray tracing
└── csv_parser.hpp # CSV pose/measurement parser
src/
├── grid.cpp      # Grid implementation
└── csv_parser.cpp # CSV parsing
Output
PGM Image (grayscale):

Black (0): Unknown
Gray (~127): Uncertain
White (255): Free space
Dark (~25): Occupied

View with: eog map.pgm or ImageMagick
Algorithm

Parse poses + ToF measurements
For each pose:

Convert ToF to distance (speed of sound)
Transform sensor positions to world frame
Ray trace free space to hit point
Mark occupied blob at detection


Log-odds update: L += log(P/(1-P))
Export PGM with prob to 0-255 mapping

Example Usage
Full resolution (slow)
STRIDE=1 ./occmap data/full.csv detailed_map.pgm
Fast preview
STRIDE=4 ./occmap data/full.csv quick_map.pgm
Custom resolution
RES=0.1 ./occmap data/full.csv coarse_map.pgm
Perfect for: ROS integration, SLAM validation, ultrasonic sensor characterization!
Built with love for robotics research • License: MIT