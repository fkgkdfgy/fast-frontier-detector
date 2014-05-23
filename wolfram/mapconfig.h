#ifndef CARMEN_CPP_MAPCONFIG_H
#define CARMEN_CPP_MAPCONFIG_H

#include "point.h"

namespace mre {

  class MapConfig {
  public:
    MapConfig();
    virtual ~MapConfig() {};
    MapConfig(int sizeX, int sizeY,  double res, Point offset);
    MapConfig(const MapConfig& src);
    bool isValid() const;

    bool operator==(const MapConfig& other);
    bool operator!=(const MapConfig& other);
  
    int m_sizeX;
    int m_sizeY;
    double m_res;
    Point m_offset;
  };

} // namespace

#endif
