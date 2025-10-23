#pragma once
#include <array>
#include <string>
#include <vector>

struct PoseRow {
    double t_ms=0, x=0, y=0, theta=0;
    std::array<double,4> tof{}; // for each sonar
};

class CSVParser {
public:
    bool parse(const std::string &path, std::vector<PoseRow> &rows) const;
};
