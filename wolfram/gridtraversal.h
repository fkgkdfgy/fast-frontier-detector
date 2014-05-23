#ifndef GRIDTRAVERSAL_H
#define GRIDTRAVERSAL_H

#include <vector>
#include "point.h"


namespace mre {

  class GridTraversal {

  public: 

  typedef std::vector<IntPoint> GridLine;

  public: 
    GridTraversal() {};
    static void  gridLine( const IntPoint& start, const IntPoint& end, GridLine& line ) ;

  protected:
    static void gridLineCore( const IntPoint& start, const IntPoint& end, GridLine& line ) ;
  };

} // namespace

#endif
