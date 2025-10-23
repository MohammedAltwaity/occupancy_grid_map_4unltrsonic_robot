#pragma once
#include <vector>
#include <string>
#include <functional>
#include <cmath>
#include <algorithm>


//we define interface for all the funcion we will use in the grid cell

class Grid {
public:
    explicit Grid(double size_m, double res,
                  double lmin = -5.0, double lmax = 5.0);

    int    width()  const { return W; }
    int    height() const { return H; }
    double resolution() const { return res; }
    double originX() const { return origin_x; }
    double originY() const { return origin_y; }

    bool worldToCell(double x, double y, int &i, int &j) const;
    bool inBounds(int i, int j) const;

    void addLogOdds(int i, int j, double dL);
    bool savePGM(const std::string &path) const;

    static inline double clampd(double v, double a, double b){
        return v<a?a:(v>b?b:v);
    }
    static inline void rot(double theta, double vx, double vy, double &ox, double &oy){
        double c = std::cos(theta), s = std::sin(theta);
        ox = c*vx - s*vy;
        oy = s*vx + c*vy;
    }

    void traceRayDDA(double sx, double sy, double ex, double ey,
                     bool stop_before_end,
                     const std::function<void(int,int)> &cb) const;

private:
    int W=0, H=0;
    double res=0.02;
    double origin_x=0.0, origin_y=0.0;
    double L_MIN=-5.0, L_MAX=5.0;
    std::vector<float> logodds;
};
