#include "gridtraversal.h"
#include <stdlib.h>


namespace mre {

  void GridTraversal::gridLineCore( const IntPoint& start, const IntPoint& end, GridLine& line )
  {
    int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

    dx = abs(end.x-start.x); dy = abs(end.y-start.y);
  
    if (dy <= dx) {
      d = 2*dy - dx; incr1 = 2 * dy; incr2 = 2 * (dy - dx);
      if (start.x > end.x) {
        x = end.x; y = end.y;
        ydirflag = (-1);
        xend = start.x;
      } else {
        x = start.x; y = start.y;
        ydirflag = 1;
        xend = end.x;
      }
      line.push_back(IntPoint(x,y));
      if (((end.y - start.y) * ydirflag) > 0) {
        while (x < xend) {
          x++;
          if (d <0) {
            d+=incr1;
          } else {
            y++; d+=incr2;
          }
          line.push_back(IntPoint(x,y));
        }
      } else {
        while (x < xend) {
          x++;
          if (d <0) {
            d+=incr1;
          } else {
            y--; d+=incr2;
          }
          line.push_back(IntPoint(x,y));
        }
      }		
    } else {
      d = 2*dx - dy;
      incr1 = 2*dx; incr2 = 2 * (dx - dy);
      if (start.y > end.y) {
        y = end.y; x = end.x;
        yend = start.y;
        xdirflag = (-1);
      } else {
        y = start.y; x = start.x;
        yend = end.y;
        xdirflag = 1;
      }
      line.push_back(IntPoint(x,y));
      if (((end.x - start.x) * xdirflag) > 0) {
        while (y < yend) {
          y++;
          if (d <0) {
            d+=incr1;
          } else {
            x++; d+=incr2;
          }
          line.push_back(IntPoint(x,y));
        }
      } else {
        while (y < yend) {
          y++;
          if (d <0) {
            d+=incr1;
          } else {
            x--; d+=incr2;
          }
          line.push_back(IntPoint(x,y));
        }
      }
    }
  }

  void GridTraversal::gridLine( const IntPoint& start, const IntPoint& end, GridLine& line ) {
    int i,j;
    int half;
    IntPoint v;
    line.clear();
    gridLineCore( start, end, line );
    int linesize = (int) line.size();

    if ( start.x!=line[0].x || start.y!=line[0].y ) {
      half = linesize/2;
      for (i=0,j=linesize - 1;i<half; i++,j--) {
        v = line[i];
        line[i] = line[j];
        line[j] = v;
      }
    }
  }

} // namespace
