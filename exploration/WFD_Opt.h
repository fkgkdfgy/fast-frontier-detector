/*
 * Exploration.h
 *
 *  Created on: 22/09/2009
 *      Author: Matan
 */

#ifndef WFD_OPT_H_
#define WFD_OPT_H_

#include <utils/point.h>
#include "Hashing.h"
#include <scanmatcher/smmap.h>
#include <list>
#include <queue>
#include <vector>
#include <algorithm>
#include "BitMap.h"
#include "pathplanner/pathplan2.h"

//DEBUG
#include <utils/commandline.h>
#include <utils/stat.h>
#include <configfile/configfile.h>
#include <iomanip>

#include "math.h"
#include <limits>

#include "Grid.h"
#include <string.h>
//#include <scanmatcher/FrontierManager.h>

#define CELL_THRESHOLD 0.5
#define FRONTIER_THRESHOLD 5
#define FRONTIER_THRESHOLD_GRID 2
#define ROBOT_SIZE 0.4
#define EXPLORATION_OPEN_SPACE  0
#define EXPLORATION_OCCUPIED  1
#define EXPLORATION_UNKNOWN  -1

using namespace GMapping;
using namespace std;

typedef ScanMatcherMap MAP;
//typedef Grid MAP;


class WFD_Opt {
public:
	enum ScanState {
		None=0x0, CloseList=0x1, OpenList=0x2
	};
	WFD_Opt(int, int, char*);

	//Methods
	POINT calcNext(POINT &, const MAP &);
	POINT calcNext(POINT &, int**, int, int, int);
	static int getCellData(POINT &, const MAP &);
	static int getCellData(POINT &, int **, int, int);
	vector<POINT_LIST> getDetectedFrontiers();

	vector<POINT> planPath(POINT &, MAP &);
	vector<Point> planPath(POINT &, int** , int, int);
	//DEBUG
	static void printPoint(const POINT &);
	void setCurrentPos(POINT &);

private:
	//Data members
	FRONTIER_LIST _frontiers;
	POINT _currentPos;
	BitMap _scanList;
	vector<BitMap*> _scanLists; // for optimized version
	bool _flgFirstTime;
	int _mapSizeX;
	int _mapSizeY;
	pathplan2 _planner;

	char _logFile[80];

	//Methods
	double realDistance(POINT&, int**, int, int);
	POINT_LIST getFrontier(POINT&, const MAP &);
	POINT_LIST getFrontier(POINT&, int **, int, int);
	list<POINT> getNeighbors(int &, int &, int &, int &);
	list<POINT> getNeighbors(POINT &, const MAP &);
	list<POINT> getNeighbors(int, int, const MAP &);

	list<POINT_LIST > findFrontiers(POINT &, const MAP &);
	list<POINT_LIST > findFrontiers(POINT &, int **, int, int);
	bool isFrontierPoint(POINT &, const MAP &);
	bool isFrontierPoint(POINT &, int **, int, int);
	bool isInList(const POINT &, ScanState);
	bool isOpenSpace(POINT &, const MAP &);
	static bool isAllocated(POINT &, const MAP &);
	bool isInside(POINT &, const MAP &);
	static bool isInside(int, int, int, int);
	bool isNeighborsRelevant(POINT &, const MAP &);
	bool isNeighborsRelevant(POINT &, int **, int, int);
	void addToList(const POINT &, ScanState);
	void removeFromList(const POINT &, ScanState);
	void printMapValues(PointHashMap &);

	void unMarkFrontier(const POINT_LIST &);

	POINT findBestCentroid(FRONTIER_LIST&, int** map, int width, int height);
	POINT findBestCentroidGUI(FRONTIER_LIST&);
	static POINT calcCentroid(const POINT_LIST &);
};

//struct PointHashSetSorter {
//	bool operator() (POINT_LIST phs1, POINT_LIST phs2) {
//		return phs1.size() > phs2.size();
//	}
//} FrontierSorter;

#endif /* EXPLORATION_H_ */
