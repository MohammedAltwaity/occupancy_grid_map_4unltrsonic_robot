#include "csv_parser.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cctype>
#include <limits>
#include <iostream>

static inline void trim_inplace(std::string &s){
    auto b=s.begin(), e=s.end();
    while(b!=e && std::isspace((unsigned char)*b)) ++b;
    while(e!=b && std::isspace((unsigned char)*(e-1))) --e;
    s.assign(b,e);
}

bool CSVParser::parse(const std::string &path, std::vector<PoseRow> &rows) const {
    std::ifstream f(path);
    if(!f) return false;
    std::string line; size_t lineno=0;

    while(std::getline(f,line)){
        ++lineno;
        if(!line.empty() && line.back()=='\r') line.pop_back(); // CRLF
        std::string orig=line; trim_inplace(line);
        if(line.empty()) continue;

        auto it = std::find_if(line.begin(), line.end(),
                               [](unsigned char c){ return !std::isspace(c); });
        if(it==line.end()) continue;
        if(std::isalpha((unsigned char)*it) || *it=='#') continue; // header/comment

        std::array<double,8> v{};
        std::string token; std::stringstream ss(line);
        int k=0; bool bad=false;

        while(std::getline(ss, token, ',') && k<8){
            trim_inplace(token);
            if(!token.empty() && (token.front()=='"' || token.front()=='\'')) token.erase(token.begin());
            if(!token.empty() && (token.back()=='"'  || token.back()=='\'' )) token.pop_back();
            std::string tl=token; std::transform(tl.begin(), tl.end(), tl.begin(), ::tolower);
            if(tl.empty() || tl=="nan"){ v[k++]=std::numeric_limits<double>::quiet_NaN(); continue; }
            try{
                size_t pos=0; double d=std::stod(token,&pos);
                if(pos!=token.size()){ bad=true; break; }
                v[k++]=d;
            }catch(...){ bad=true; break; }
        }
        if(bad || k<8){
            std::cerr<<"[CSVParser] Skipping bad line "<<lineno<<": "<<orig<<"\n";
            continue;
        }

        PoseRow r;
        r.t_ms=v[0]; r.x=v[1]; r.y=v[2]; r.theta=v[3];
        for(int i=0;i<4;i++) r.tof[i]=v[4+i];
        rows.push_back(r);
    }
    return true;
}
