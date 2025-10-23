// build an occupancy grid map from robot poses + 4 ultrasonic tof sensors.


//   compile(we compile the main file which is in the root dir along with src files, the flag -Iinclude to tell the compiler about the header folder  ):
//    g++ -std=c++17 main.cpp src/*.cpp -Iinclude -o occmap
//     run( we pass the csv file path and the  name of the ouput iamge in pgm format):     ./occmap assets/robot.csv map.pgm


#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <utility>

#include "grid.hpp"
#include "csv_parser.hpp"

// --- main tunable parameters..the can be adjusted accordingly ---


// you can adjust these to match your robot setup or sensor behavior
static const double V_SOUND    = 343.0;   // speed of sound in m/s
static const double MAX_RANGE  = 4.0;     // cap measurements at 4m
static const double MIN_RANGE  = 0.12;    // ignore very close echoes (robot's own body)
static const double RES        = 0.05;    // grid resolution: 5 cm per cell
static const double MAP_SIZE_M = 20.0;    // map covers 20 x 20 meters
static const double FOV_DEG    = 30.0;    // each sensor sees a cone of ~30°
static const int    N_RAYS     = 1;       // how many discrete rays per cone
static const double W_CHASSIS  = 0.10;    // robot width (x direction)
static const double L_CHASSIS  = 0.20;    // robot length (y direction)
static const double HIT_RADIUS = 0.05;    // radius to smear occupied hits
static const int    STRIDE     = 2;       // skip rows (process every 2nd row from csv)

// log-odds update values for free and occupied cells
static const double P_OCC  = 0.70;
static const double P_FREE = 0.30;
static const double L_OCC  = std::log(P_OCC /(1.0 - P_OCC));
static const double L_FREE = std::log(P_FREE/(1.0 - P_FREE));

int main(int argc, char** argv){
    // check args
    if(argc < 3){
        std::cerr << "usage: " << argv[0] << " input.csv output.pgm\n";
        return 1;
    }
    const std::string in_csv  = argv[1];
    const std::string out_pgm = argv[2];

    // read in the csv with robot poses and tof measurements
    CSVParser parser;
    std::vector<PoseRow> rows;
    if(!parser.parse(in_csv, rows)){
        std::cerr << "failed to read csv: " << in_csv << "\n";
        return 2;
    }

    // set up an empty grid
    Grid grid(MAP_SIZE_M, RES, -5.0, +5.0);

    // sensor bearings relative to robot heading
    // us0 = front-right (45°), us1 = front-left (135°),
    // us2 = rear-left (-135°), us3 = rear-right (-45°)
    const double PI = std::acos(-1.0);
    const std::array<double,4> bearing = {
        +PI/4.0,
        +3.0*PI/4.0,
        -3.0*PI/4.0,
        -PI/4.0
    };

    // physical offsets of each sensor on the chassis
    const std::array<std::pair<double,double>,4> offset = {
        std::pair<double,double>(+W_CHASSIS/2.0, +L_CHASSIS/2.0),
        std::pair<double,double>(-W_CHASSIS/2.0, +L_CHASSIS/2.0),
        std::pair<double,double>(-W_CHASSIS/2.0, -L_CHASSIS/2.0),
        std::pair<double,double>(+W_CHASSIS/2.0, -L_CHASSIS/2.0)
    };

    const double half_fov = (FOV_DEG * PI / 180.0) * 0.5;

    // walk through the dataset
    for(size_t rix=0; rix<rows.size(); rix += std::max(1, STRIDE)){
        const PoseRow &r = rows[rix];

        // handle each of the 4 sensors
        for(int s=0; s<4; ++s){
            // 1) convert time-of-flight to distance (meters)
            double tof = r.tof[s];
            if(!(tof==tof)) continue;        // skip nan
            if(tof<=0.0 || tof>1.0) continue;
            double raw_range = 0.5 * V_SOUND * tof;
            if(raw_range <= MIN_RANGE) continue;

            bool clipped    = (raw_range > MAX_RANGE + 1e-6);
            double range    = std::min(raw_range, MAX_RANGE);

            // 2) compute sensor position in world coordinates
            double sx_r = offset[s].first, sy_r = offset[s].second;
            double sx, sy; Grid::rot(r.theta, sx_r, sy_r, sx, sy);
            sx += r.x; sy += r.y;

            // 3) sensor facing direction in world
            double phi0 = r.theta + bearing[s];

            // 4) cast one or more rays through the cone
            for(int k=0; k<N_RAYS; ++k){
                double a = (N_RAYS==1)? 0.0 : (-half_fov + (2.0*half_fov)*(double(k)/(N_RAYS-1)));
                double phi = phi0 + a;
                double vx = std::cos(phi), vy = std::sin(phi);

                // mark free cells up to just before the hit
                double free_stop = clipped ? range : std::max(0.0, range - HIT_RADIUS);
                double fx = sx + free_stop*vx;
                double fy = sy + free_stop*vy;
                grid.traceRayDDA(sx, sy, fx, fy, true,
                                  [&](int gi,int gj){ grid.addLogOdds(gi,gj, L_FREE); });

                // mark the occupied blob at the hit point
                if(!clipped){
                    double hx = sx + range*vx;
                    double hy = sy + range*vy;
                    int ci,cj;
                    if(grid.worldToCell(hx,hy,ci,cj)){
                        int r_cells = std::max(1, (int)std::round(HIT_RADIUS / grid.resolution()));
                        for(int di=-r_cells; di<=r_cells; ++di){
                            for(int dj=-r_cells; dj<=r_cells; ++dj){
                                if(di*di + dj*dj <= r_cells*r_cells)
                                    grid.addLogOdds(ci+di, cj+dj, L_OCC);
                            }
                        }
                    }
                }
            } // rays
        } // sensors
    } // rows

    //  save the map as a pgm image
    if(!grid.savePGM(out_pgm)){
        std::cerr << "failed to write pgm: " << out_pgm << "\n";
        return 3;
    }
    std::cout << "wrote map: " << out_pgm << "\n";
    return 0;
}
