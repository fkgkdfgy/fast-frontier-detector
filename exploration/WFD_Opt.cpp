/*
 * Exploration.cpp
 *
 *  Created on: 22/09/2009
 *      Author: Matan
 */

#include "WFD_Opt.h"
#include "GridConverter.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <ostream>
#include <fstream>
using namespace std;

WFD_Opt::WFD_Opt(int  mapSizeX, int mapSizeY, char* logFile) : _scanList(mapSizeX, mapSizeY){
	_flgFirstTime = true;
	_mapSizeX = mapSizeX;
	_mapSizeY = mapSizeY;
	strcpy(_logFile, logFile);

	for (int i = 0; i < 30; ++i) {
		_scanLists.push_back(new BitMap(mapSizeX, mapSizeY));
	}
}

void WFD_Opt::unMarkFrontier(const POINT_LIST &frontier) {

	cout << "DEBUG: unmarkFrontier size = " << frontier.size() << endl;

	for (POINT_LIST::const_iterator itr = frontier.begin(); itr != frontier.end(); ++itr) {

		removeFromList(*itr, CloseList);
		removeFromList(*itr, OpenList);

	}
}

/**
 * Calculates the next destination on the grid
 */

POINT WFD_Opt::findBestCentroid(FRONTIER_LIST &frontiers, int** map, int width, int height) {

	vector<double> normalizedData;

	double totalSizes = 0;
	double totalDistance = 0;
	POINT currCentroid;
	FRONTIER_LIST::iterator itr;

	//sum the centroid distance and length of each frontier
	for (itr = frontiers.begin(); itr != frontiers.end(); ++itr) {
		currCentroid = calcCentroid(*itr);

		//totalDistance += euclidianDist(_currentPos, currCentroid);
		totalDistance += realDistance(currCentroid, map, width, height);

		totalSizes += itr->size();
	}

	static POINT lastCentroid;
	//cout << "Last centroid is: "; printPoint(lastCentroid); cout << endl;

	//normalize
	double currDistance = 0;
	for (itr = frontiers.begin(); itr != frontiers.end(); ++itr) {
		currCentroid = calcCentroid(*itr);

		currDistance = realDistance(currCentroid, map, width, height);
//		currDistance = euclidianDist(_currentPos, currCentroid);


		// TODO Eran: HACK HACK HACK
//		if (lastCentroid == currCentroid) {
//			cout << "CENTROID ALREADY SELECTED PREVIOUSLY, ASSIGNING IT REALLY LARGE DISTANCE" << endl;
//			cout << "Last centroid is: "; printPoint(lastCentroid); cout << endl;
//			cout << "currDistance = " << totalDistance << endl;
//			currDistance = totalDistance;
//		}

		//normalizedData.push_back( sqrt((itr->size() / totalSizes)) /
							     // ((currDistance * currDistance) / (totalDistance * totalDistance) )
		normalizedData.push_back((itr->size() / totalSizes) /
							      ((currDistance * currDistance) / (totalDistance * totalDistance) ));
	}

	//find max normalized frontier measurements
	int maxIndex = 0;

	for (unsigned int i = 0; i < normalizedData.size(); ++i) {
		if (normalizedData[i] > normalizedData[maxIndex])
			maxIndex = i;
	}

	//at this line, we've found the index of the nearest centroid to current robot position
	itr = frontiers.begin();
	for (int i = 0 ; i < maxIndex; ++itr, ++i) {}

	lastCentroid = currCentroid = calcCentroid(*itr);
	//cout << "Last centroid now equals: "; printPoint(lastCentroid); cout << endl;

	// Eran: Nearest point disabled since paths to unknown area were too short (and hence too many re-calc of frontier)
	//now we need to find the nearest frontier point to the centroid
	double minDistance = numeric_limits<double>::max();
	POINT minCentroidOnFrontier;
	POINT minPoint;

	for (POINT_LIST::iterator j = itr->begin(); j != itr->end(); ++j) {
		currDistance = euclidianDist(*j, currCentroid);

		if (currDistance < minDistance) {
			minDistance = currDistance;
			minCentroidOnFrontier = *j;
		}
	}
	return minCentroidOnFrontier;
//	return currCentroid;
}

POINT WFD_Opt::findBestCentroidGUI(FRONTIER_LIST &frontiers) {

	vector<double> normalizedData;

	double totalSizes = 0;
	double totalDistance = 0;
	POINT currCentroid;
	FRONTIER_LIST::iterator itr;

	//sum the centroid distance and length of each frontier
	for (itr = frontiers.begin(); itr != frontiers.end(); ++itr) {
		currCentroid = calcCentroid(*itr);

		totalDistance += euclidianDist(_currentPos, currCentroid);

		totalSizes += itr->size();
	}

	static POINT lastCentroid;
	//cout << "Last centroid is: "; printPoint(lastCentroid); cout << endl;

	//normalize
	double currDistance = 0;
	for (itr = frontiers.begin(); itr != frontiers.end(); ++itr) {
		currCentroid = calcCentroid(*itr);

		currDistance = euclidianDist(_currentPos, currCentroid);


		// TODO Eran: HACK HACK HACK
//		if (lastCentroid == currCentroid) {
//			cout << "CENTROID ALREADY SELECTED PREVIOUSLY, ASSIGNING IT REALLY LARGE DISTANCE" << endl;
//			cout << "Last centroid is: "; printPoint(lastCentroid); cout << endl;
//			cout << "currDistance = " << totalDistance << endl;
//			currDistance = totalDistance;
//		}

		//normalizedData.push_back( sqrt((itr->size() / totalSizes)) /
							     // ((currDistance * currDistance) / (totalDistance * totalDistance) )
		normalizedData.push_back((itr->size() / totalSizes) /
							      ((currDistance * currDistance) / (totalDistance * totalDistance) ));
	}

	//find max normalized frontier measurements
	int maxIndex = 0;

	for (unsigned int i = 0; i < normalizedData.size(); ++i) {
		if (normalizedData[i] > normalizedData[maxIndex])
			maxIndex = i;
	}

	//at this line, we've found the index of the nearest centroid to current robot position
	itr = frontiers.begin();
	for (int i = 0 ; i < maxIndex; ++itr, ++i) {}

	lastCentroid = currCentroid = calcCentroid(*itr);
	//cout << "Last centroid now equals: "; printPoint(lastCentroid); cout << endl;

	// Eran: Nearest point disabled since paths to unknown area were too short (and hence too many re-calc of frontier)
	//now we need to find the nearest frontier point to the centroid
	double minDistance = numeric_limits<double>::max();
	POINT minCentroidOnFrontier;
	POINT minPoint;

	for (POINT_LIST::iterator j = itr->begin(); j != itr->end(); ++j) {
		currDistance = euclidianDist(*j, currCentroid);

		if (currDistance < minDistance) {
			minDistance = currDistance;
			minCentroidOnFrontier = *j;
		}
	}
	return minCentroidOnFrontier;
//	return currCentroid;
}

void WFD_Opt::setCurrentPos(POINT &currPosition) {
	_currentPos = currPosition;
}

POINT WFD_Opt::calcNext(POINT &currPosition, int **map, int width, int height, int index) {

	cerr << "*********************** check calcGRID " << endl;
	_scanList = *(_scanLists[index]);

	//set current robot position
	_currentPos = currPosition;

	FRONTIER_LIST found;

	// NOTE: Optimization disabled:
	_frontiers.clear();

	static ofstream f_out;
	struct timeval startTime;
	struct timeval endTime;
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	startTime = ru.ru_utime;

	found = findFrontiers(_currentPos, map, width, height);

	getrusage(RUSAGE_SELF, &ru);
	endTime = ru.ru_utime;
	double tS = startTime.tv_sec*1000000.0 + (startTime.tv_usec);
	double tE = endTime.tv_sec*1000000.0  + (endTime.tv_usec);
	double result = tE - tS;

	if (result != 0) {
//		f_out.open("exploration_execution_grid.txt", ios::app);
		f_out.open(_logFile, ios::app);
		f_out << tE - tS << endl;
		f_out.close();
	}

	list<POINT_LIST >::iterator frontierItr;

	//insert all found frontiers
	for (frontierItr = found.begin(); frontierItr != found.end(); ++frontierItr)
		_frontiers.push_back(*frontierItr);

	_scanList.clear();

	//cout <<  "DEBUG: total frontiers detected: " << _frontiers.size() << endl;

	//check if there's nothing left to explore
	if (_frontiers.size() == 0)
		return _currentPos;

	//get the lowest cost centroid

	return findBestCentroid(_frontiers, map, width, height);

//	FRONTIER_LIST::iterator itr = _frontiers.begin();
//	POINT currCentroid, bestCentroid;
//
//	bestCentroid = currCentroid = calcCentroid((*itr));
//	maxScore = currScore = itr->size() / euclidianDist(currCentroid, _currentPos);
//	POINT_LIST::iterator j = itr->begin();
//	cout << "DEBUG: Frontier (" << (*j).x << "," << (*j).y << ") size = " << std::fixed << std::setw(4) << itr->size() << " dist = " << std::setw(5) << std::setprecision(0) << euclidianDist(currCentroid, _currentPos) << " score = " << std::setw(5) << std::setprecision(2) << currScore << endl;
//	++itr;
//
//
//	for (; itr != _frontiers.end(); ++itr) {
//		currCentroid = calcCentroid((*itr));
//		currScore = itr->size() / euclidianDist(currCentroid, _currentPos);
//		j = itr->begin();
//		cout << "DEBUG: Frontier (" << (*j).x << "," << (*j).y << ") size = " << std::fixed << std::setw(4) << itr->size() << " dist = " << std::setw(5) << std::setprecision(0) << euclidianDist(currCentroid, _currentPos) << " score = " << std::setw(5) << std::setprecision(2) << currScore << endl;
//		if (currScore > maxScore) {
//			maxScore = currScore;
//			bestCentroid = currCentroid;
//		}
//	}

//	//DEBUG
//	cout << "frontiers are: " << endl;
//	for (itr = _frontiers.begin(); itr != _frontiers.end(); ++itr) {
//		cout << "{ " ;
//
//		POINT_LIST::iterator j = itr->begin();
////		for (; j != itr->end(); ++j)
////			cout << "(" << (*j).x << "," << (*j).y << ") " ;
//
//		cout << "(" << (*j).x << "," << (*j).y << ") " ;
//
//		cout << "} size = " << itr->size() << endl;
//	}

	//return bestCentroid;
	*(_scanLists[index]) = _scanList;
}

POINT WFD_Opt::calcNext(POINT &currPosition, const MAP &map) {

	cerr << "*********************** check calcNextGUI " << endl;
	static ofstream f_out;
	f_out.open("exploration_execution_rafael.txt", ios::app);

	//set current robot position
	_currentPos = currPosition;

	FRONTIER_LIST found;

	if (_flgFirstTime) {

		struct timeval startTime;
		struct timeval endTime;
		struct rusage ru;
		getrusage(RUSAGE_SELF, &ru);
		startTime = ru.ru_utime;

		//for (int i = 0; i < 1; ++i) {

			// NOTE: Optimization disabled:
			_frontiers.clear();

			found = findFrontiers(_currentPos, map);


			list<POINT_LIST >::iterator frontierItr;

			//insert all found frontiers
			for (frontierItr = found.begin(); frontierItr != found.end(); ++frontierItr)
				_frontiers.push_back(*frontierItr);

	//		_flgFirstTime = false;
			_scanList.clear();
		//}

		getrusage(RUSAGE_SELF, &ru);
		endTime = ru.ru_utime;
		double tS = startTime.tv_sec*1000000 + (startTime.tv_usec);
		double tE = endTime.tv_sec*1000000  + (endTime.tv_usec);
		f_out << (tE - tS) << endl;
		f_out.close();

	} else {

		FRONTIER_LIST::iterator i;
		FRONTIER_LIST::iterator j;

		//DEBUG
		//int counter = 0;
		//for (i = _frontiers.begin(); i != _frontiers.end(); ++i) {
		//	cout << "frontier" << counter++ << " size =  " << i->size() << endl;
		//}

		//scan each detected frontier
		FRONTIER_LIST newFrontiers;
		for (i = _frontiers.begin(); i != _frontiers.end(); ++i) {

			unMarkFrontier(*i);

			//scan each point in current frontier
			for (POINT_LIST::iterator itr = i->begin(); itr != i->end(); ++itr) {
				POINT p = *itr;

				struct timeval startTime;
				struct timeval endTime;
				struct rusage ru;
				getrusage(RUSAGE_SELF, &ru);
				startTime = ru.ru_utime;

				//flood fill from current frontier
				found = findFrontiers(p, map);
				double tS = startTime.tv_sec*1000000 + (startTime.tv_usec);
				double tE = endTime.tv_sec*1000000  + (endTime.tv_usec);
				getrusage(RUSAGE_SELF, &ru);
				endTime = ru.ru_utime;
				//f_out << tE - tS << endl;
				//f_out.close();

				//insert all found frontiers
				for (j = found.begin(); j != found.end(); ++j)
					newFrontiers.push_back(*j);
			}
		}

		_frontiers = newFrontiers;

		//found = findFrontiers(_currentPos, map);
	}

	f_out.close();
	//cout <<  "DEBUG: total frontiers detected: " << _frontiers.size() << endl;


	//check if there's nothing left to explore
	if (_frontiers.size() == 0)
		return _currentPos;

	//get the lowest cost centroid
	// matan
	POINT dummy;
	return dummy;
	return findBestCentroidGUI(_frontiers);


//	double maxScore, currScore;
//
//	FRONTIER_LIST::iterator itr = _frontiers.begin();
//	POINT currCentroid, bestCentroid;
//
//	bestCentroid = currCentroid = calcCentroid((*itr));
//	maxScore = currScore = itr->size() / euclidianDist(currCentroid, _currentPos);
//	POINT_LIST::iterator j = itr->begin();
//	cout << "DEBUG: Frontier (" << (*j).x << "," << (*j).y << ") size = " << std::fixed << std::setw(4) << itr->size() << " dist = " << std::setw(5) << std::setprecision(0) << euclidianDist(currCentroid, _currentPos) << " score = " << std::setw(5) << std::setprecision(2) << currScore << endl;
//
//	++itr;
//
////	cout << "DEBUG: frontiers found: " << _frontiers.size() << endl;
//
//	for (; itr != _frontiers.end(); ++itr) {
//
//		currCentroid = calcCentroid((*itr));
//		currScore = itr->size() / euclidianDist(currCentroid, _currentPos);
//		j = itr->begin();
//		cout << "DEBUG: Frontier (" << (*j).x << "," << (*j).y << ") size = " << std::fixed << std::setw(4) << itr->size() << " dist = " << std::setw(5) << std::setprecision(0) << euclidianDist(currCentroid, _currentPos) << " score = " << std::setw(5) << std::setprecision(2) << currScore << endl;
//		if (currScore > maxScore) {
//			maxScore = currScore;
//			bestCentroid = currCentroid;
//		}
//	}
//
////	//DEBUG
////	cout << "frontiers are: " << endl;
////	for (itr = _frontiers.begin(); itr != _frontiers.end(); ++itr) {
////		cout << "{ " ;
////
////		POINT_LIST::iterator j = itr->begin();
//////		for (; j != itr->end(); ++j)
//////			cout << "(" << (*j).x << "," << (*j).y << ") " ;
////
////		cout << "(" << (*j).x << "," << (*j).y << ") " ;
////
////		cout << "} size = " << itr->size() << endl;
////	}
//
//	return bestCentroid;
}

/**
 * Calculates the centroid of the given collection
 */
POINT WFD_Opt::calcCentroid(const POINT_LIST &points) {

	POINT_LIST::const_iterator itr;
	double sumX = 0, sumY = 0;

	//calculate centroid of all points
	for (itr = points.begin(); itr != points.end(); ++itr) {
		//POINT p = *itr;
		sumX += itr->x;
		sumY += itr->y;
	}

	int size = points.size();

	return POINT(sumX / size, sumY / size); // return centroid
}

list<POINT_LIST > WFD_Opt::findFrontiers(POINT &root, int** map, int width, int height) {
	list<POINT_LIST > frontiers;	// list that contains all frontiers have been found
	queue<POINT> queue; 			// queue for BFS algorithm
	POINT p;						// keeps the head of queue
	bool frontierFound;

	//enqueue robot's current position
	addToList(root, OpenList);

	queue.push(root);

	//perform BFS
	while (!queue.empty()) {

		frontierFound = false;

		//pop head of queue
		p = queue.front();
		queue.pop();

		//DEBUG
		//printPoint(p); cout << " was popped from queue" << endl;

		//check if current point was already scanned
		if (isInList(p, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}
		//check if current point is a frontier point
		if (isFrontierPoint(p, map, width, height)) {

			//find the frontier that contains current point
			POINT_LIST frontier = getFrontier(p, map, width, height);

			//filter sensor noise
			if (frontier.size() > FRONTIER_THRESHOLD_GRID) {
				frontiers.push_back(frontier); //save frontier
				frontierFound = true;
			}
		}

		//insert all frontier points to close list
		if (frontierFound) {

			//cout << "DEBUG: frontier detected" << endl;

			POINT_LIST currFrontier = frontiers.back();
			POINT_LIST::iterator i;

			for (i = currFrontier.begin(); i != currFrontier.end(); ++i) {
				addToList(*i, CloseList);
			}

		}

		//get current point's neighbors
		list<POINT> neighbors = getNeighbors(p.x, p.y, width, height);

		//scan all neighbors
		for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

			//enqueue neighbor only if it is its first scan
			if (!isInList(*itr, CloseList) &&
				!isInList(*itr, OpenList) &&
				isNeighborsRelevant(*itr, map, width, height)) {
					queue.push(*itr);

					//openList.insert(*itr);
					addToList(*itr, OpenList);

					//cout << "\tDEBUG: neighbors insert to queue: (" << itr->x << "," << itr->y << ")" << endl;
			}
		}

//		openList.erase(p);
//		_closeList.insert(p);

		removeFromList(p, OpenList); //delete from open list
		addToList(p, CloseList); //insert to close list

		//printMapValues(_scanList);

		//DEBUG
		//cout << endl;
	}

	return frontiers;
}

list<POINT_LIST > WFD_Opt::findFrontiers(POINT &root, const MAP &map) {

	list<POINT_LIST > frontiers;	// list that contains all frontiers have been found
	//PointHashSet openList;			// contains points that are still in queue
	queue<POINT> queue; 			// queue for BFS algorithm
	POINT p;						// keeps the head of queue
	bool frontierFound;

	//enqueue robot's current position
	addToList(root, OpenList);

	queue.push(root);

	//perform BFS
	while (!queue.empty()) {

		frontierFound = false;

		//pop head of queue
		p = queue.front();
		queue.pop();

		//DEBUG
		//printPoint(p); cout << " was popped from queue" << endl;

		//check if current point was already scanned
		if (isInList(p, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}
		//check if current point is a frontier point
		if (isFrontierPoint(p, map)) {

			//find the frontier that contains current point
			POINT_LIST frontier = getFrontier(p, map);

			//filter sensor noise
			if (frontier.size() > FRONTIER_THRESHOLD) {
				frontiers.push_back(frontier); //save frontier
				frontierFound = true;
			}
		}

		//insert all frontier points to close list
		if (frontierFound) {

			//cout << "DEBUG: frontier detected" << endl;

			POINT_LIST currFrontier = frontiers.back();
			POINT_LIST::iterator i;

			for (i = currFrontier.begin(); i != currFrontier.end(); ++i) {
				addToList(*i, CloseList);
			}

		}

		//get current point's neighbors
		list<POINT> neighbors = getNeighbors(p, map);

		//scan all neighbors
		for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

			//enqueue neighbor only if it is its first scan
			if (!isInList(*itr, CloseList) &&
				!isInList(*itr, OpenList) &&
				isNeighborsRelevant(*itr, map)) {
					queue.push(*itr);

					//openList.insert(*itr);
					addToList(*itr, OpenList);

					//cout << "\tDEBUG: neighbors insert to queue: (" << itr->x << "," << itr->y << ")" << endl;
			}
		}

//		openList.erase(p);
//		_closeList.insert(p);

		removeFromList(p, OpenList); //delete from open list
		addToList(p, CloseList); //insert to close list

		//printMapValues(_scanList);

		//DEBUG
		//cout << endl;
	}

	return frontiers;
}

bool WFD_Opt::isNeighborsRelevant(POINT &p, const MAP &map) {

	list<POINT> neighbors = getNeighbors(p, map);

	for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		if (getCellData(*itr, map) == EXPLORATION_OPEN_SPACE)
			return true;
	}

	return false;
}

bool WFD_Opt::isNeighborsRelevant(POINT &p, int** map, int width, int height) {

	list<POINT> neighbors = getNeighbors(p.x, p.y, width, height);

	for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		if (getCellData(*itr, map, width, height) == EXPLORATION_OPEN_SPACE)
			return true;
	}

	return false;
}

POINT_LIST WFD_Opt::getFrontier(POINT &root, int **map, int width, int height) {
	POINT_LIST frontier;	// collection of frontier points
	BitMap frontierScanList(width, height);
	queue<POINT> queue;	// queue for BFS algorithm
	POINT p;				// keeps the head of the queue

	//enqueue given frontier point
	queue.push(root);

	frontierScanList.set(root.x, root.y, OpenList);

	//cout << "***************************************" << endl;
	//cout << "DEBUG: get frontier with " << "(" << root.x << "," << root.y << ")" << endl;

	// perform BFS search
	while (!queue.empty()) {
		//pop head
		p = queue.front();
		queue.pop();

		//check if current point was already scanned
		if(isInList(p, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}

		//check if current point was already scanned for frontier
		if (frontierScanList.isState(p.x, p.y, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}

		//check if current is a frontier point
		if (isFrontierPoint(p, map, width, height)) {

			//save current point as a frontier
			frontier.push_back(p);

			//get current point's neighbors
			list<POINT> neighbors = getNeighbors(p.x, p.y, width, height);

			//scan all neighbors
			for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

				//enqueue neighbor only if it is its first scan
				//if (!isInList(*itr, CloseList) && !isInList(*itr, OpenList)) {
				if (!isInList(*itr, CloseList)) {

					//if (closeList.find(p) == closeList.end()) {
					if (!frontierScanList.isState(itr->x,itr->y, CloseList) &&
						!frontierScanList.isState(itr->x, itr->y, OpenList)) {
					//if (closeList.find(*itr) == closeList.end() && openList.find(*itr) == openList.end()) {
						queue.push(*itr);

						frontierScanList.set(itr->x, itr->y, OpenList);
						//addToList(*itr, OpenList);
					}
					//openList.insert(*itr);
				}
			}
		}

		removeFromList(p, OpenList); //delete from open list

		//addToList(p, CloseList); //insert to close list
		frontierScanList.set(p.x, p.y, CloseList);
	}

	return frontier;
}

POINT_LIST WFD_Opt::getFrontier(POINT &root, const MAP &map) {
	POINT_LIST frontier;	// collection of frontier points
	BitMap frontierScanList(_mapSizeX, _mapSizeY);
	//POINT_HASHSET openList;	// contains points that are still in queue
	queue<POINT> queue;	// queue for BFS algorithm
	POINT p;				// keeps the head of the queue

	//enqueue given frontier point
	queue.push(root);
	//addToList(root, OpenList);

	frontierScanList.set(root.x, root.y, OpenList);

	//cout << "***************************************" << endl;
	//cout << "DEBUG: get frontier with " << "(" << root.x << "," << root.y << ")" << endl;

	// perform BFS search
	while (!queue.empty()) {
		//pop head
		p = queue.front();
		queue.pop();

		//check if current point was already scanned
		if(isInList(p, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}

		//check if current point was already scanned for frontier
		if (frontierScanList.isState(p.x, p.y, CloseList)) {
			removeFromList(p, OpenList);
			continue;
		}

		//check if current is a frontier point
		if (isFrontierPoint(p, map)) {

			//save current point as a frontier
			frontier.push_back(p);

			//get current point's neighbors
			list<POINT> neighbors = getNeighbors(p, map);

			//scan all neighbors
			for (list<POINT>::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

				//enqueue neighbor only if it is its first scan
				//if (!isInList(*itr, CloseList) && !isInList(*itr, OpenList)) {
				if (!isInList(*itr, CloseList)) {

					//if (closeList.find(p) == closeList.end()) {
					if (!frontierScanList.isState(itr->x,itr->y, CloseList) &&
						!frontierScanList.isState(itr->x, itr->y, OpenList)) {
					//if (closeList.find(*itr) == closeList.end() && openList.find(*itr) == openList.end()) {
						queue.push(*itr);

						frontierScanList.set(itr->x, itr->y, OpenList);
						//addToList(*itr, OpenList);
					}
					//openList.insert(*itr);
				}
			}
		}

		removeFromList(p, OpenList); //delete from open list

		//addToList(p, CloseList); //insert to close list
		frontierScanList.set(p.x, p.y, CloseList);
	}

	return frontier;
}

double WFD_Opt::realDistance(POINT& target, int** map, int width, int height) {
    vector<Point> path = planPath(target, map, width, height);

    if (path.empty()) {
    	return euclidianDist(_currentPos, target) * 2.0; // preferring targets with a path and considering obstacles
    }

    double sum = 0.0;
    vector<Point>::iterator itr = path.begin();
    Point prev = *itr;

    for (++itr; itr != path.end(); ++itr) {
        sum += euclidianDist(prev, *itr);
        prev = *itr;
    }

    return sum;
}

//int WFD_Opt::getCellData(POINT &p, const MAP &map) {
//	return map.getCell(p.x, p.y); //Matan
//
//}

int WFD_Opt::getCellData(POINT &p, const MAP &map) {

	if (!isAllocated(p, map))
		return EXPLORATION_UNKNOWN;

	double data = map.storage().cell(p);

	if (data == EXPLORATION_UNKNOWN)
		return EXPLORATION_UNKNOWN;

	return (data > CELL_THRESHOLD ? EXPLORATION_OCCUPIED : EXPLORATION_OPEN_SPACE);
}

int WFD_Opt::getCellData(POINT &p, int** map, int width, int height) {

	if (!isInside(p.x, p.y, width, height))
		return EXPLORATION_UNKNOWN;

	double data = map[p.x][p.y];

	if (data == EXPLORATION_UNKNOWN)
		return EXPLORATION_UNKNOWN;

	return (data > CELL_THRESHOLD ? EXPLORATION_OCCUPIED : EXPLORATION_OPEN_SPACE);
}

bool WFD_Opt::isFrontierPoint(POINT &p, const MAP &map) {

	//check if cell data is unknown
	//if (transformData(map.cell(p) != UNKNOWN)) //VERIFY!!!!

//	if (!isAllocated(p, map) || map.storage().cell(p) != UNKNOWN)
	//if (isAllocated(p, map) && getCellData(p, map) != UNKNOWN) //HACK

	if (getCellData(p, map) != EXPLORATION_UNKNOWN) //HACK
		return false;


	//if (transformData(map.storage().cell(p) != UNKNOWN)) //VERIFY!!!!
//		return false;


	//get the neighbors of current cell
	list<POINT> neighbors = getNeighbors(p, map);

	//ignore errors
	if (neighbors.size() == 0)
		return false;

	//collect the adjacent cells that are known and not limits
	list<POINT>::iterator itr;
	for (itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		//if (getCellData(map.storage().cell(*itr)) == OPEN_SPACE)
		//if ( isKnownCell(*itr, map))
		if (getCellData(*itr, map) == EXPLORATION_OPEN_SPACE)
			return true;
	}

	return false;
}

bool WFD_Opt::isFrontierPoint(POINT &p, int **map, int width, int height) {

	if (getCellData(p, map, width, height) != EXPLORATION_UNKNOWN) //HACK
		return false;


	//get the neighbors of current cell
	list<POINT> neighbors = getNeighbors(p.x, p.y, width, height);

	//ignore errors
	if (neighbors.size() == 0)
		return false;

	//collect the adjacent cells that are known and not limits
	list<POINT>::iterator itr;
	for (itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		//if (getCellData(map.storage().cell(*itr)) == OPEN_SPACE)
		//if ( isKnownCell(*itr, map))
		if (getCellData(*itr, map, width, height) == EXPLORATION_OPEN_SPACE)
			return true;
	}

	return false;
}

bool WFD_Opt::isInList(const POINT &p, ScanState state) {

	return _scanList.isState(p.x, p.y, state);
}

inline void WFD_Opt::addToList(const POINT &p, ScanState state) {


	int value = _scanList.get(p.x, p.y);

	_scanList.set(p.x, p.y, (value | state));

	//map.storage().cell(p).state = (ScanState)(map.storage().cell(p).state | state);

//	ScanState newScanState = (ScanState)(map.storage().cell(p).state | state);
//	map.storage().cell(p).setScanState(newScanState);
}

void WFD_Opt::removeFromList(const POINT &p, ScanState state) {



	int value = _scanList.get(p.x, p.y);
	_scanList.set(p.x, p.y, value & (~state));

//	map.storage().cell(p).state = (ScanState)(map.storage().cell(p).state & ~state);

//	ScanState newScanState = (ScanState)(map.storage().cell(p).state & ~state);
//	map.storage().cell(p).setScanState(newScanState);
}

bool WFD_Opt::isInside(POINT &p, const MAP &map) {
//	return map.isInside(p.x, p.y); //MATAN
	return (map.storage().cellState(p) == Inside );
}

bool WFD_Opt::isInside(int x, int y, int width, int height) {
	if (x < 0 || y < 0)
		return false;

	if (x < width && y < height)
		return true;

	return false;
}

bool WFD_Opt::isAllocated(POINT &p, const MAP &map) {
//	return true; //MATAN
	return (map.storage().cellState(p) == (Inside | Allocated) );
}

void WFD_Opt::printPoint(const POINT &p) {
	cout << "(" << p.x << "," << p.y << ")" ;
}

bool WFD_Opt::isOpenSpace(POINT &p, const MAP &map) {

	if (!isAllocated(p, map))
		return false;

//	if (map.storage().cell(p) == EXPLORATION_UNKNOWN)
//		return false;

	if (getCellData(p, map) == EXPLORATION_OPEN_SPACE)
		return true;

	return false;

//	if (c == OPEN_SPACE || c == CURR_POSITION)
//		return true;

//	return false;
}

//void WFD_Opt::printMapValues(PointHashMap &map) {
//	PointHashMap::iterator itr = map.begin();
//
//	if (map.size() == 0)
//		return;
//
//	cout << "Close list: " ;
//	for (; itr != map.end(); ++itr) {
//		if (isInList(itr->first, CloseList))
//			cout << "(" << itr->first.x << "," << itr->first.y << ") ";
//	}
//
//	cout << endl;
//
//	cout << "Open list: " ;
//	itr = _scanList.begin();
//	for (; itr != map.end(); ++itr) {
//		if (isInList(itr->first, OpenList))
//			cout << "(" << itr->first.x << "," << itr->first.y << ") ";
//	}
//
//	cout << endl;
//}

list<POINT> WFD_Opt::getNeighbors(POINT &p, const MAP &map) {
	return getNeighbors(p.x, p.y, map);
}

list<POINT> WFD_Opt::getNeighbors(int x, int y, const MAP &map) {
	list<POINT> neighbours;

	if(!map.isInside(x, y)) //validate coordinates of the given cell
		return neighbours;

	if (map.isInside(x - 1, y - 1)) //top left
		neighbours.push_back(POINT(x - 1, y - 1));

	if (map.isInside(x - 1, y)) 	//top
			neighbours.push_back(POINT(x - 1, y));

	if (map.isInside(x - 1, y + 1)) //top right
			neighbours.push_back(POINT(x - 1, y + 1));

	if (map.isInside(x, y - 1)) 	//left
			neighbours.push_back(POINT(x, y - 1));

	if (map.isInside(x, y + 1)) 	//right
			neighbours.push_back(POINT(x, y + 1));

	if (map.isInside(x + 1, y - 1))	//bottom left
			neighbours.push_back(POINT(x + 1, y - 1));

	if (map.isInside(x + 1, y))		//bottom
			neighbours.push_back(POINT(x + 1, y));

	if (map.isInside(x + 1, y + 1))	//bottom right
				neighbours.push_back(POINT(x + 1, y + 1));

	return neighbours;
}


list<POINT> WFD_Opt::getNeighbors(int &x, int &y, int &width, int &height) {
	list<POINT> neighbours;

	if(!isInside(x, y, width, height)) //validate coordinates of the given cell
		return neighbours;

	if (isInside(x - 1, y - 1, width, height)) //top left
		neighbours.push_back(POINT(x - 1, y - 1));

	if (isInside(x - 1, y, width, height)) 	//top
			neighbours.push_back(POINT(x - 1, y));

	if (isInside(x - 1, y + 1, width, height)) //top right
			neighbours.push_back(POINT(x - 1, y + 1));

	if (isInside(x, y - 1, width, height)) 	//left
			neighbours.push_back(POINT(x, y - 1));

	if (isInside(x, y + 1, width, height)) 	//right
			neighbours.push_back(POINT(x, y + 1));

	if (isInside(x + 1, y - 1, width, height))	//bottom left
			neighbours.push_back(POINT(x + 1, y - 1));

	if (isInside(x + 1, y, width, height))		//bottom
			neighbours.push_back(POINT(x + 1, y));

	if (isInside(x + 1, y + 1, width, height))	//bottom right
				neighbours.push_back(POINT(x + 1, y + 1));

	return neighbours;
}

vector<POINT_LIST> WFD_Opt::getDetectedFrontiers() {

	vector<POINT_LIST> frontiers;

	FRONTIER_LIST::iterator itr;

	for (itr = _frontiers.begin(); itr != _frontiers.end(); ++itr) {
		frontiers.push_back(*itr);
	}

	//sort(frontiers.begin(), frontiers.end(), FrontierSorter);
	return frontiers;
}

vector<Point> WFD_Opt::planPath(POINT &dest, int** map, int width, int height) {

	int* grid = GridConverter::convertExp2Astar(map, width, height);

	// Assumption: both current robot location and destination are OPEN_SPACE
	grid[dest.x + dest.y*height] = 1;
	grid[_currentPos.x + _currentPos.y*height] = 1;

	vector<node> nodePath = _planner.get_graph(grid, width, height, _currentPos.x, _currentPos.y, dest.x, dest.y);

	//cout << "size of nodePath: " << nodePath.size() << endl;

	vector<Point> pointPath;

	for (int i = 0; i < (int) nodePath.size(); ++i) {
		node curr = nodePath[i];
		//cout << "x: " << nodePath[i].x << "y: " << nodePath[i].y << endl;
		pointPath.push_back(Point(curr.x, curr.y));
	}

	//DEBUG
//	for (int y = 270; y > 230; --y) {
//		for (int x = 230; x < 270; ++x) {
//			if (x == 250 && y == 250)
//				cout << "X";
//			else if (x == _currentPos.x && y == _currentPos.y)
//				cout << "S";
//			else if (x == dest.x && y == dest.y)
//				cout << "T";
//			else
//				cout << grid[x + (y*height)];
//		}
//		cout << endl;
//	}

	delete grid;

	return pointPath;
}

vector<POINT> WFD_Opt::planPath(POINT &dest, MAP &map) {

	int cellSize = ROBOT_SIZE / map.getDelta();

	int* grid = GridConverter::createAstar(map, cellSize);
	int cols = map.getMapSizeX() / cellSize;
	int rows = map.getMapSizeY() / cellSize;

	int sourceX = _currentPos.x / cellSize;
	int sourceY = _currentPos.y / cellSize;

	int destX = dest.x / cellSize + 1;
	int destY = dest.y / cellSize - 1;

	vector<node> pathGrid = _planner.get_graph(grid, cols, rows, sourceX, sourceY, destX, destY);

	delete grid;

	//DEBUG
	//cout << "Source is " << sourceX << "," << sourceY << endl;
	//cout << "Dest is " << destX << "," << destY << endl;
	//End DEBUG

	vector<POINT> pathMap;
	//TODO: need to fix 90 orientation and transform to map coordinations
	for (int i = 0; i < (int) pathGrid.size(); ++i) {

		int x = (double)pathGrid[i].x / cellSize + (double)cellSize / 2;
		int y = (double)pathGrid[i].y / cellSize + (double)cellSize / 2;

		POINT p(x, y);
		pathMap.push_back(p);
	}


	//DEBUG
//	int limitX, limitY;
//	limitX = map.getMapSizeX() / cellSize;
//	limitY = map.getMapSizeY() / cellSize;
//	int* testGrid = GridConverter::createAstar(map, cellSize);
//	for (int y = 270; y > 230; --y) {
//		for (int x = 230; x < 270; ++x) {
//			if (x == 250 && y == 250)
//				cout << "X";
//			else if (x == sourceX && y == sourceY)
//				cout << "S";
//			else if (x == destX && y == destY)
//				cout << "T";
//			else
//				cout << testGrid[x + (y*limitX)];
//		}
//		cout << endl;
//	}
//	delete testGrid;
	//End DEBUG

	return pathMap;
}

