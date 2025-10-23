Occupancy Grid Mapper
A lightweight C++ tool for generating 2D occupancy grid maps from robot poses and four ultrasonic Time-of-Flight (ToF) sensors, using log-odds Bayesian updates. Perfect for robotics, SLAM, and sensor-based mapping in research and development.
Quick Start
Compile (C++17)

g++ -std=c++17 main.cpp src/*.cpp -Iinclude -o occmap

Run
Create a map from CSV to PGM:
./occmap assets/robot.csv map.pgm


Input Format
CSV: timestamp,x,y,theta,tof0,tof1,tof2,tof3

x, y: Robot position (meters)
theta: Heading (radians)
tof[0-3]: ToF measurements (seconds) for 4 sensors

Sensor Layout:

us0: Front-right (45°)
us1: Front-left (135°)
us2: Rear-left (-135°)
us3: Rear-right (-45°)

Key Features

Log-Odds Updates: Bayesian model (P_occ=0.7, P_free=0.3)
Ray Tracing: DDA algorithm for free/occupied cells
Sensor Model: 30° FOV cones, configurable rays
Uncertainty: 5cm hit smearing, 4m range cap
Efficiency: STRIDE skips rows for speed


Tunable Parameters

const double RES        = 0.05;  // 5cm grid cells
const double MAP_SIZE_M = 20.0;  // 20×20m map
const int    N_RAYS     = 1;     // Rays per cone
const double MAX_RANGE  = 4.0;   // Max range (m)
const int    STRIDE     = 2;     // Process every 2nd pose


Project Structure

main.cpp: Core algorithm
include/grid.hpp: Grid and ray tracing
include/csv_parser.hpp: CSV parser
src/grid.cpp, src/csv_parser.cpp: Implementations



Output
PGM Image (grayscale):

Black (0): Unknown
Gray (~127): Uncertain
White (255): Free
Dark (~25): Occupied

View with: eog map.pgm or ImageMagick



Algorithm

Parse CSV for poses and ToF data
For each pose:

Convert ToF to distance (343 m/s)
Transform sensor positions to world frame
Trace free space to hit point
Mark occupied blob (5cm radius)


Update log-odds: L += log(P/(1-P))
Export PGM (probability → 0-255)



Applications

ROS integration
SLAM validation
Ultrasonic sensor analysis




License: MIT

