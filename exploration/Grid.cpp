/*
 * Grid.cpp
 *
 *  Created on: 23/09/2009
 *      Author: Matan
 */

#include "Grid.h"

/**
 * Constructor
 * size: size of one edge of the grid (grid is a square)
 */
Grid::Grid(int size) {
	//cout << "DEBUG: constructor called" << endl;
	_data = new CELL*[size];

	_size = size;

	//Allocate memory to grid cells
	for (int i = 0; i < _size; ++i) {

		CELL* row = new CELL[_size];
		_data[i] = row;

		//cout << "\tallocating row " << i << endl;

		for (int j = 0; j < _size; ++j)
			row[j] = G_UNKNOWN;
	}
}

Grid::Grid(const Grid& grid) {
	cout << "DEBUG: copy constructor called" << endl;
	_data = new CELL*[grid._size];
	_size = grid._size;

	//Allocate memory to grid cells
	for (int i = 0; i < _size; ++i) {
		cout << "\tcopy row " << i << endl;

		CELL* newRow = new CELL[_size];
		CELL* oldRow = grid._data[i];

		_data[i] = newRow;

		for (int j = 0; j < _size; ++j)
			newRow[j] = oldRow[j];
	}
}

/**
 * Destructor
 */
Grid::~Grid() {
	//Delete each line
	for (int i = 0; i < _size; ++i)
		delete [] _data[i];

	delete [] _data;
}

/**
 * Validates if given coordinations are in the grid range
 */
inline bool Grid::isInside(int row, int col) const{
	if (row >= _size || col >= _size)
		return false;

	if (row < 0 || col < 0)
		return false;

	return true;
}

/**
 * Gets cell data from grid
 */
int Grid::getCell(int row, int col) const{
	if (!isInside(row, col))
		return -1;

	if (_data[row][col] == G_BORDER)
		return 1;
	else if (_data[row][col] == G_OPEN_SPACE)
		return 0;

	return -1;
}


/**
 * Prints a set (for debugging)
 */
void printSet(POINT_HASHSET set) {
	POINT_HASHSET::iterator itr;
	cout << "{ " ;
	for (itr = set.begin(); itr != set.end(); ++itr) {
		cout << itr->x << "," << itr->y << " ";
	}

	cout << " }" << endl;

}

/**
 * Sets the current robot position in the grid
 */
void Grid::setCurrentPosition(int x, int y) {
	_currPos = POINT(x, y);
	//setCell(x, y, G_CURR_POSITION);
}

//list<Point> Grid::getNeighbors(Point &p) {
//	return getNeighbors((int)p.x, (int)p.y);
//}


ostream& operator << (ostream &os, const Grid &grid) {
	for (int i = 0; i < grid._size; ++i) {
		for (int j = 0; j < grid._size; ++j)
			os << grid._data[i][j];
		os << endl;
	}

	return os;
}

/**
 * Sets the value of the given cell
 */
void Grid::setCell(int row, int col, CELL val) {
	if (!isInside(row, col))
		return;

	_data[row][col] = val;
}

void Grid::setCell(POINT &p, CELL value) {
	setCell((int)p.x, (int)p.y, value);
}
