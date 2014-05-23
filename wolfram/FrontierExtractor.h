#ifndef FRONTIERS_H
#define FRONTIERS_H

#include <list>
#include <vector>
#include "mapdefinitions.h"
#include <scanmatcher/smmap.h>
//#include <exploration/Exploration.h>
#include "ExplorationTools.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <ostream>
#include <fstream>

using namespace GMapping;

namespace mre {

    class Frontier {

    public:

    Frontier() : id(0) {}
    Frontier(uint _id) : id(_id) {}
      ~Frontier() { points.clear(); }

      uint size() { return points.size(); }
      void setId(uint _id) { id = _id; }
      uint getId() { return id; }

      uint id;
      std::list<IntPoint> points;
      IntPoint target;
    };

    typedef std::list<Frontier*> FrontierList;
    typedef std::list<Frontier*>::iterator FrontierListIterator;
    typedef std::list<IntPoint> PointList;
    typedef std::list<IntPoint>::iterator PointListIterator;

  /*!
    Based on code by Cyrill Stachniss
    Restructured by Kai M. Wurm <wurm@informatik.uni-freiburg.de>

    Class to extract frontiers from gridmaps.
    Maps have to implement carmenpp::AbstractMap.
  */
    class FrontierExtractor {

  public:

    FrontierExtractor(unsigned int _min_frontier_length);
    ~FrontierExtractor();

    // set cell types explicitely.
    // they default to (FREE, UNKNOWN)
//    void setFrontierTypes(std::vector<TypeCell::CellTypes>& explored, TypeCell::CellTypes unexplored);

    //! compute frontiers and return list of representative points
    //  WARNING: will create frontiers with new, delete them yourself
    void computeFrontierCells(ScanMatcherMap* gridmap, FrontierList& frontiers);

  protected:

    //! Is p a frontier cell in gridmap?
    bool IsFrontierCell(ScanMatcherMap* gridmap, IntPoint p);

    void updateFrontierList(ScanMatcherMap* map, FrontierList& frontiers);
    PointList computeFrontierFromFrontierCell(ScanMatcherMap* map, IntPoint front, int** visited);
    IntPoint  getEndOfFrontier(ScanMatcherMap* map, IntPoint start, int **visited , int spur );
    PointList computeFrontier(ScanMatcherMap* map, PointList& alreadyComputed, int **visited);

    void getTargetsFromFrontiers(FrontierList& frontiers);

    int m_frontiers_gridmapsize_x;
    int m_frontiers_gridmapsize_y;

    std::vector<TypeCell::CellTypes> explored_types;
    TypeCell::CellTypes unexplored_type;

    int m_spur;
    int **m_visited;

    int id_counter;

    unsigned int min_frontier_length;
  };

} // namespace


#endif
