#ifndef CARMEN_CPP_ABSTRACT_MAP_H
#define CARMEN_CPP_ABSTRACT_MAP_H

#include "point.h"
#include "mapconfig.h"

namespace mre {

  template <class CELL>
    class AbstractMap {
  public:
    AbstractMap();
    AbstractMap(const AbstractMap& x);
    AbstractMap(const MapConfig& cfg);
    virtual ~AbstractMap();  


    virtual CELL& getCell(int x, int y) = 0;
    virtual CELL& getCell(int x, int y) const = 0;
  

    // Vitual function to be defined in subclass
    virtual bool init(const MapConfig& cfg) = 0;
    virtual const CELL& defaultCell() const = 0;

    // non-virtual function for the map handling
    bool init(int sizeX, int sizeY, double res=1.0, Point offset = Point(0.0, 0.0) ); 
    bool init(double xfrom, double xto, double yfrom, double yto, double res);
    bool initIfSmaller(int sizeX, int sizeY, double res=1.0, Point offset = Point(0.0, 0.0) ); 

    const MapConfig& getConfig() const;

    IntPoint getMin() const;
    IntPoint getMax() const;
    int getMapSizeX() const;
    int getMapSizeY() const;

    void setOffset(const Point& p);
    Point getOffset() const;

    void setResolution(double res);
    double getResolution() const;
  
    Point map2world_double(const Point& p) const;
    Point world2map_double(const Point& p) const;
    Point map2world(const Point& p) const;
    IntPoint world2map(const Point& p) const;
    Point map2world(const IntPoint& p) const;
    Point map2world(int x, int y) const;
    IntPoint world2map(double x, double y) const;
  
    bool isInside(const Point& p) const;
    bool isInside(const IntPoint& p) const;
    bool isInside(int x, int y) const;
    bool isInside(double x, double y) const;
  
    IntPoint minInside(const IntPoint& p) const;
    IntPoint maxInside(const IntPoint& p) const;
    IntPoint putInside(const IntPoint& p) const;

    CELL& cell(const IntPoint& p);  
    CELL& cell(const IntPoint& p) const;

    CELL& cell(int x, int y);
    CELL& cell(int x, int y) const;

    CELL& cell(double x, double y);
    CELL& cell(double x, double y) const;
  
    CELL& cell(const Point& p) const ;
    CELL& cell(const Point& p);

    void copy(const AbstractMap<CELL>& src);
    void copy(const AbstractMap<CELL>& src, const IntPoint& relative_offset);
    void moveMap(int dx, int dy);

    void resetCells();
    void resetCells(const CELL& val);
    void resetCells(const CELL& val, const IntPoint& from, const IntPoint& to);
  public:
    MapConfig m_cfg;
  };

} // namespace

#include "abstractmap.hxx"

#endif

