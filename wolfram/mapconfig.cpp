#include "mapconfig.h"


namespace mre {


  MapConfig::MapConfig() { 
    m_sizeX  = -1;
    m_sizeY  = -1;
    m_res    = 0.0;
    m_offset = Point(0.0,0.0);
  }

  MapConfig::MapConfig(int sizeX, int sizeY, double res, Point offset) { 
    m_sizeX  = sizeX;
    m_sizeY  = sizeY;
    m_res    = res;
    m_offset = offset;
  }

  MapConfig::MapConfig(const MapConfig& src) {
    m_sizeX  = src.m_sizeX;
    m_sizeY  = src.m_sizeY;
    m_res    = src.m_res;
    m_offset = src.m_offset;
  }

  bool MapConfig::isValid() const {
    if (m_sizeX <= 0 || m_sizeY <= 0 ||  m_res <=0.0)
      return false;
    return true;
  }

  bool operator==(const MapConfig& cfg1, const MapConfig& cfg2)  {
    if (cfg1.m_sizeX  == cfg2.m_sizeX && 
        cfg1.m_sizeY  == cfg2.m_sizeY && 
        cfg1.m_res    == cfg2.m_res && 
        cfg1.m_offset == cfg2.m_offset)
      return true;
    else
      return false;
  }

  bool operator!=(const MapConfig& cfg1, const MapConfig& cfg2)  {
    return !(cfg1 == cfg2);
  }

} // namespace
