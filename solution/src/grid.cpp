#include "grid.hpp"
#include <fstream>
#include <limits>

/* constructor: set up an empty square grid with given size and resolution.
 the map is centered at (0,0) in world coordinates, so we store an origin offset.
 all log-odds are initialized to 0 → probability 0.5 (unknown) */


Grid::Grid(double size_m, double res_, double lmin, double lmax)
: res(res_), L_MIN(lmin), L_MAX(lmax)
{
    W = H = std::max(1, (int)std::round(size_m / res));
    origin_x = size_m/2.0;
    origin_y = size_m/2.0;
    logodds.assign((size_t)W*H, 0.0f);
}



// check if a cell index (i,j) is inside the map bounds.
bool Grid::inBounds(int i, int j) const {
    return (i>=0 && i<H && j>=0 && j<W);
}



// convert world coordinates (x,y in meters) to grid cell indices (i,j).
// note: i is row index (downward), j is column index (to the right).
bool Grid::worldToCell(double x, double y, int &i, int &j) const {
    i = (int)std::floor((origin_y - y) / res);   // rows increase downward
    j = (int)std::floor((x + origin_x) / res);   // cols increase rightward
    return inBounds(i, j);
}



// add a log-odds increment to a cell and clamp it within [L_MIN, L_MAX].
// this lets us "push" the probability toward occupied or free gradually.
void Grid::addLogOdds(int i, int j, double dL){
    if(!inBounds(i,j)) return;
    float &L = logodds[(size_t)i*W + j];
    L = (float)clampd(L + dL, L_MIN, L_MAX);
}



// export the current grid as a grayscale pgm image.
// black = occupied, white = free, mid-gray = unknown.
bool Grid::savePGM(const std::string &path) const {
    std::vector<unsigned char> img((size_t)W*H);
    for(int i=0;i<H;i++){
        for(int j=0;j<W;j++){
            double L = logodds[(size_t)i*W + j];
            double p = 1.0 / (1.0 + std::exp(-L));   // probability from log-odds
            int v = (int)std::round(255.0 - 255.0*p);
            if(v<0) v=0; if(v>255) v=255;
            img[(size_t)i*W + j] = (unsigned char)v;
        }
    }
    std::ofstream f(path, std::ios::binary);
    if(!f) return false;
    f << "P5\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), (std::streamsize)img.size());
    return true;
}

/* trace a ray from (sx,sy) to (ex,ey) in world coordinates.
we step through grid cells using dda (digital differential analyzer).
 for each traversed cell, we call the provided callback.
 if stop_before_end is true, we stop just before the endpoint cell (so it isn’t marked free). */


 
void Grid::traceRayDDA(double sx, double sy, double ex, double ey,
                       bool stop_before_end,
                       const std::function<void(int,int)> &cb) const
{
    // find start and end cells
    int i0,j0, i1,j1;
    if(!worldToCell(sx,sy,i0,j0)) return;
    if(!worldToCell(ex,ey,i1,j1)) return;

    int i = i0, j = j0;
    cb(i,j);  // mark the starting cell

    // helpers to compute world coordinates of cell boundaries
    auto cellMinX = [&](int jj){ return -origin_x + jj    * res; };
    auto cellMaxX = [&](int jj){ return -origin_x + (jj+1)* res; };
    auto cellMaxY = [&](int ii){ return  origin_y - ii     * res; };
    auto cellMinY = [&](int ii){ return  origin_y - (ii+1) * res; };

    // set up dda stepping
    double dx = ex - sx, dy = ey - sy;
    int stepJ = (dx>0) ?  1 : (dx<0 ? -1 : 0);
    int stepI = (dy>0) ? -1 : (dy<0 ?  1 : 0); // note: rows go downward

    double txDelta = (dx==0) ? std::numeric_limits<double>::infinity() : std::abs(res/dx);
    double tyDelta = (dy==0) ? std::numeric_limits<double>::infinity() : std::abs(res/dy);

    double nextX = (stepJ>0) ? cellMaxX(j) : cellMinX(j);
    double nextY = (stepI>0) ? cellMinY(i) : cellMaxY(i);
    double txMax = (dx==0) ? std::numeric_limits<double>::infinity() : (nextX - sx)/dx;
    double tyMax = (dy==0) ? std::numeric_limits<double>::infinity() : (nextY - sy)/dy;

    // walk cell by cell until we reach the target
    while(true){
        if(stop_before_end && i==i1 && j==j1) break;
        if(!stop_before_end && i==i1 && j==j1){ cb(i,j); break; }

        if(txMax < tyMax){ j += stepJ; txMax += txDelta; }
        else             { i += stepI; tyMax += tyDelta; }

        if(!inBounds(i,j)) break;
        cb(i,j);
    }
}
