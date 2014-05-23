#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <values.h>
#include <iostream>

#include "FrontierExtractor.h"

namespace mre {


  FrontierExtractor::FrontierExtractor(unsigned int _min_frontier_length) :
    m_frontiers_gridmapsize_x(0), m_frontiers_gridmapsize_y(0), m_spur(1), m_visited(NULL),
    id_counter(0), min_frontier_length(_min_frontier_length) {

    explored_types.push_back(TypeCell::FREE);
    unexplored_type = TypeCell::UNKNOWN;
  }



  FrontierExtractor::~FrontierExtractor() {

    if (m_frontiers_gridmapsize_x > 0 &&
        m_frontiers_gridmapsize_y > 0 &&
        m_visited != NULL) {
      for (int x = 0; x < m_frontiers_gridmapsize_x; x++)
        delete [] m_visited[x];
      delete [] m_visited;
      m_visited = NULL;
    }
    m_frontiers_gridmapsize_x = 0;
    m_frontiers_gridmapsize_y = 0;
  }


//  void FrontierExtractor::setFrontierTypes(std::vector<TypeCell::CellTypeTypeMaps>& explored, TypeCell::CellTypes unexplored) {
//    unexplored_type = unexplored;
//    explored_types = explored;
//  }



  bool FrontierExtractor::IsFrontierCell(ScanMatcherMap* gridmap, IntPoint p) {

	  GMapping::IntPoint gIntPoint(p.x, p.y);
	  return isFrontierCell((*gridmap), gIntPoint);
  }

  /*bool FrontierExtractor::IsFrontierCell(ScanMatcherMap* gridmap, IntPoint p)  {

    char cell_type = gridmap->getCell(p.x, p.y).val;

    bool cell_is_explored = false;
    for (unsigned int i = 0; i < explored_types.size(); i++) {
      if (cell_type == explored_types[i]) cell_is_explored=true;
    }
    if (!cell_is_explored) return false;

    if (gridmap->isInside(p.x-1, p.y)) {
      if (gridmap->getCell(p.x-1, p.y).val == unexplored_type)
        return true;
    }
    else
      return true;

    if (gridmap->isInside(p.x+1, p.y)) {
      if (gridmap->getCell(p.x+1, p.y).val == unexplored_type)
        return true;
    }
    else
      return true;

    if (gridmap->isInside(p.x, p.y-1)) {
      if (gridmap->getCell(p.x, p.y-1).val == unexplored_type)
        return true;
    }
    else
      return true;

    if (gridmap->isInside(p.x, p.y+1)) {
      if (gridmap->getCell(p.x, p.y+1).val == unexplored_type)
        return true;
    }
    else
      return true;

    return false;
  }*/



  void FrontierExtractor::computeFrontierCells(ScanMatcherMap* gridmap, FrontierList& frontiers) {

    m_spur = 1;
    id_counter = 0;


    frontiers.clear();

    static ofstream f_out;
	f_out.open("exploration_execution_wolfram.txt", ios::app);

	struct timeval startTime;
	struct timeval endTime;
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	startTime = ru.ru_utime;
	cout << "DEBUG: INSIDE WOLFRAM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    // extract frontiers from map
    updateFrontierList(gridmap, frontiers);


    // kill small frontiers
    // also set ids in order

    id_counter = 0;
    FrontierListIterator fit = frontiers.begin();
    while (fit != frontiers.end()) {
      if ((*fit)->size() < min_frontier_length) {
        delete *fit;
        fit = frontiers.erase(fit);
      }
      else {
        (*fit)->setId(id_counter);
        fit++; id_counter++;
      }
    }

    getrusage(RUSAGE_SELF, &ru);
	endTime = ru.ru_utime;
//	double tS = startTime.tv_sec*1000000 + (startTime.tv_usec);
//	double tE = endTime.tv_sec*1000000  + (endTime.tv_usec);
	f_out << (endTime.tv_sec - startTime.tv_sec)*1000000.0 + (endTime.tv_usec - startTime.tv_usec) << endl;
	//f_out << (tE - tS) << endl;
	f_out.close();


    // calc target points
    getTargetsFromFrontiers(frontiers);

  }



  void FrontierExtractor::updateFrontierList(ScanMatcherMap* gridmap, FrontierList& frontiers) {

    // map size changed or not initialized yet -> init members
    if (m_frontiers_gridmapsize_x != gridmap->getMapSizeX() ||
        m_frontiers_gridmapsize_y != gridmap->getMapSizeY()) {


      // clean m_visited if prev. initialized
      if (m_frontiers_gridmapsize_x > 0 &&
          m_frontiers_gridmapsize_y > 0 &&
          m_visited != NULL) {
        for (int x = 0; x < m_frontiers_gridmapsize_x; x++)
          delete [] m_visited[x];
        delete [] m_visited;
        m_visited = NULL;
      }

      m_frontiers_gridmapsize_x = gridmap->getMapSizeX();
      m_frontiers_gridmapsize_y = gridmap->getMapSizeY();

      // init m_visited
      m_visited = new int*[gridmap->getMapSizeX()];
      for (int x = 0; x < gridmap->getMapSizeX(); x++)
        m_visited[x] = new int[gridmap->getMapSizeY()];

    } // end init members


    // reset visited matrix
    IntPoint pos;
    for (pos.x = 0; pos.x < (int) gridmap->getMapSizeX(); pos.x++)
      for (pos.y = 0; pos.y < (int) gridmap->getMapSizeY(); pos.y++)
        m_visited[pos.x][pos.y] = 0;


    // for all cells...
    IntPoint current_cell;
    for (current_cell.x = 0; current_cell.x < (int) gridmap->getMapSizeX(); current_cell.x++)  {
      for (current_cell.y = 0; current_cell.y < (int) gridmap->getMapSizeY(); current_cell.y++) {

        // ... that are frontier cells and have not been visited ...
        if (IsFrontierCell(gridmap, current_cell) && (m_visited[current_cell.x][current_cell.y]!=1 )) {

          // ... compute frontier
          PointList frontier_points = computeFrontierFromFrontierCell(gridmap, current_cell, m_visited);

          Frontier* new_frontier = new Frontier(id_counter);
          new_frontier->points = frontier_points;
          frontiers.push_back(new_frontier);
          id_counter++;
        }
      }
    } // end for all cells

  }



  PointList FrontierExtractor::computeFrontierFromFrontierCell(ScanMatcherMap* gridmap, IntPoint frontier_cell, int** visited) {

    PointList buffer;
    m_spur++;

    IntPoint end_of_frontier = getEndOfFrontier(gridmap, frontier_cell, visited, m_spur);

    buffer.push_back(end_of_frontier);
    visited[end_of_frontier.x][end_of_frontier.y]=1;

    return computeFrontier(gridmap, buffer, visited);
  }


  IntPoint FrontierExtractor::getEndOfFrontier(ScanMatcherMap* gridmap, IntPoint start, int **visited , int spur) {
    IntPoint pos;
    IntPoint next;

    int startX = std::max(0, start.x - 1);
    int endX = std::min(gridmap->getMapSizeX()-1, start.x + 1);
    int startY = std::max(0, start.y - 1);
    int endY = std::min(gridmap->getMapSizeY()-1, start.y + 1);

    int i; int j;
    visited[start.x][start.y] = spur;
    for( i = startX; i <= endX; i++) {
      for(j = startY; j <= endY; j++) {
        pos.x=i;
        pos.y=j;

        if(IsFrontierCell(gridmap, pos) && visited[i][j]!=1 && visited[i][j]!=spur){

          visited[i][j]=spur;

          next = getEndOfFrontier(gridmap, pos, visited, spur);
          return next;
        }
      }
    }
    return start;
  }



  PointList FrontierExtractor::computeFrontier(ScanMatcherMap* gridmap, PointList& alreadyComputed, int **visited) {

    IntPoint current_cell = alreadyComputed.back();

    // for all 4-neighbors of current cell...
    IntPoint neighbor;
    for(neighbor.x = (current_cell.x)-1;  neighbor.x <= (current_cell.x)+1; neighbor.x++) {
      for(neighbor.y = (current_cell.y)-1; neighbor.y <= (current_cell.y)+1; neighbor.y++) {

        // ... that are within the map...
        if (gridmap->isInside(neighbor.x, neighbor.y)) {

          // ... check if it is a frontier cell and has not been visited
          if ((IsFrontierCell(gridmap, neighbor)) && (visited[neighbor.x][neighbor.y]!=1)) {

            // if so, add it to the frontier
            alreadyComputed.push_back(neighbor);
            visited[neighbor.x][neighbor.y] = 1;

            return computeFrontier(gridmap, alreadyComputed, visited);
          }
        }

      }
    } // end for all neighbors


    return alreadyComputed;
  }




  void FrontierExtractor::getTargetsFromFrontiers(FrontierList& frontiers) {

    // for all frontiers
    for(FrontierListIterator it = frontiers.begin(); it != frontiers.end(); it++) {


      uint pos_counter = 0;
      for (PointListIterator pit = (*it)->points.begin(); pit != (*it)->points.end(); pit++,pos_counter++) {
        if (pos_counter == (*it)->size()/2) {
          (*it)->target = *pit;
          break;
        }
      }
    }
  }


} // namespace
