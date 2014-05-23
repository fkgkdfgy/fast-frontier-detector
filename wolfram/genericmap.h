#ifndef CARMEN_CPP_GENERIC_MAP_H
#define CARMEN_CPP_GENERIC_MAP_H

#include "point.h"
#include "abstractmap.h"

namespace mre {

  template<class CELL>
    class GenericMap : public AbstractMap<CELL> {
  public:
    GenericMap();
    GenericMap(const MapConfig& cfg);
    GenericMap(const GenericMap<CELL>& src);
    virtual ~GenericMap();  

    virtual bool init(const MapConfig& cfg);  
    virtual const CELL& defaultCell() const;
  
    virtual CELL& getCellWorld(double x, double y);
    virtual CELL& getCellWorld(double x, double y) const;
    virtual CELL& getCell(int x, int y);
    virtual CELL& getCell(int x, int y) const;

  protected:
    CELL*  m_maplinear;
    CELL** m_map;
    CELL   m_defaultCell;
  };

} // namespace

#include "genericmap.hxx"

#endif

