/*
 * Grid.h
 *
 *  Created on: 23/09/2009
 *      Author: Matan
 */

#ifndef GRID_H_
#define GRID_H_

//#include "Exploration.h"

#include "Hashing.h"
#include <list>
#include <queue>

using namespace std;


#define G_FRONTIER_THRESHOLD 0

#define G_GRID_SIZE 10
#define G_BORDER '*'
#define G_OPEN_SPACE ' '
#define G_UNKNOWN '?'
#define G_CURR_POSITION 'x'

typedef char CELL;



/**
 * Represents a grid that contains CELL values
 */
class Grid {
private:
	//Data members
	CELL** _data; 					//matrix that represents the world
	POINT _currPos; 				//the robot location in the grid
	int _size; 						//size of one edge
	POINT_HASHSET _closeList;		//keeps tracking of scanned points on grid

	//Methods

	bool isKnown(Point&);
	bool isFrontierPoint(Point&);
	bool isFrontierPoint(int,int);
	POINT_HASHSET getFrontier(Point&);
	bool contains(POINT_HASHSET &, Point&);
public:
	//Methods
	Grid(int size = G_GRID_SIZE); //Ctor
	Grid(const Grid&);			//Copy Ctor
	virtual ~Grid();			//Destructor
	bool isInside(int, int) const;
	//Getters and Setters
	//list<Point> getNeighbors(int, int);
	//list<Point> getNeighbors(Point &);
	int getCell(int, int) const;

	void setCell(int, int, CELL);
	void setCell(POINT&, CELL);
	void setCurrentPosition(int x, int y);
	int size() {return _size;}

	//Manipulations
	list<POINT_HASHSET > findFrontiers();

	//Operators
	friend ostream& operator << (ostream &os, const Grid &grid);
};

#endif /* GRID_H_ */
