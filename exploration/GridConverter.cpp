/*
 * GridConverter.cpp
 *
 *  Created on: Nov 5, 2009
 *      Author: matan
 */

#include "GridConverter.h"

list<POINT> getNeighbors(int x, int y, const MAP &map) {
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

int** GridConverter::createExploration(const MAP &map, int cellWidthPixels, int cellHeightPixels, int** grid) {

	//get the world dimensions
	int rows = map.getMapSizeX() / cellWidthPixels;
	int cols = map.getMapSizeY() / cellHeightPixels;

	//create a new grid
	//int** grid = new int*[rows];

	//fill each cell in grid
	for (int x = 0; x < rows; ++x) {

//		grid[x] = new int[cols];

		for (int y = 0; y < cols; ++y) {

			//MIN
			int result = classify(x * cellWidthPixels,
								 (rows*cellWidthPixels < (x+1)*cellWidthPixels) ? rows*cellWidthPixels - 1 : (x+1)*cellWidthPixels - 1,
								  y * cellHeightPixels,
								 (cols*cellHeightPixels < (y+1)*cellHeightPixels) ? cols*cellHeightPixels - 1 : (y+1) * cellHeightPixels - 1,
								map,
								ExplorationGrid);

			//int result = Exploration::getCellData(myPoint,map); //classify(i,i,j,j,cellSize, map);

//			if (result != -1)
//				cout << "result: " << result << endl;

			grid[x][y] = result;
		}
	}

//	list<POINT> neighbors;
//	list<POINT>::iterator itr;
//	for (int x = 1; x < rows-2; ++x) {
//		for (int y = 1; y < cols-2; ++y) {
//			if (grid[x][y] == OPEN_SPACE) {
//				neighbors = getNeighbors(x, y, map);
//				for (itr = neighbors.begin(); itr != neighbors.end(); ++itr) {
//					//check if current point is wall
//					if (grid[itr->x][itr->y] == OCCUPIED) {
//						cout << grid[x][y] << endl;
//						grid[x][y] = 10;
//						continue;
//					}
//				}
//			}
//		}
//	}

	return grid;
}

int GridConverter::getCellWeight(int** map, int width, int height, int x, int y) {
	if (x == 0 || x == (width-1) || y == 0 || y == (height-1)) {
		return (map[x][y] == EXPLORATION_OPEN_SPACE) ? 1 : 0;
	}
	//bool isNeighbourUnknown = false;

	if (map[x][y] != EXPLORATION_OPEN_SPACE) {
		return 0;
	} else {
		// If cell not on edge and cell is open-space, go over neighbours
		for (int ix = -1; ix <= 1; ix++)
			for (int iy = -1; iy <= 1; iy++)
				// If neighbour cell is occupied, return higher weight (more costly)
				if (map[x+ix][y+iy] == EXPLORATION_OCCUPIED) {
					return 4;
				} /*else if (map[x+ix][y+iy] == UNKNOWN) {
					isNeighbourUnknown = true;
				}*/
		/*if (isNeighbourUnknown) {
			return 1;
		}*/
		return 1;
	}
}

int* GridConverter::convertExp2Astar(int** map, int width, int height) {
	int* grid = new int[width * height];
	int data = 0;

	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			//data = (map[x][y] == OPEN_SPACE) ? 1 : 0;
			data = getCellWeight(map, width, height, x, y);
			grid[x + y*height] = data;
		}
	}

//	for (int x = width/2-20; x < width/2+20; ++x) {
//		for (int y = height/2-20; y < height/2+20; ++y) {
//			cout << ((map[x][y] == OPEN_SPACE)? " " : "X");
//		}
//		cout << endl;
//	}

	return grid;
}

int* GridConverter::createAstar(const MAP &map, int cellSize) {

	//get the world dimensions
	int rows = map.getMapSizeX() / cellSize;
	int cols = map.getMapSizeY() / cellSize;

	//create a new grid
	int* grid = new int[rows * cols];

	//fill each cell in grid
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {

			//MIN
			int result = classify(i * cellSize,
								 (rows*cellSize < (i+1)*cellSize) ? rows*cellSize - 1 : (i+1)*cellSize - 1,
								  j * cellSize,
								 (cols*cellSize < (j+1)*cellSize) ? cols*cellSize - 1 : (j+1) * cellSize - 1,
								 map,
								 AstarGrid);

			//int result = Exploration::getCellData(myPoint,map); //classify(i,i,j,j,cellSize, map);
			grid[i + j*cols] = result;
		}
	}

	return grid;
}

int GridConverter::classify(int startX, int targetX, int startY, int targetY, const MAP &map, GridFormats type) {

//	POINT testPoint(startX, startY);
//
//	return Exploration::getCellData(testPoint, map);

	//if (startX == 2000 && startY == 2000)
	//	cout << "DEBUG" << endl;

//	POINT center((targetX - startX) / 2 + startX, (targetY - startY) / 2 + startY);

	int data;
// = Exploration::getCellData(center, map);

	int lengthX = targetX - startX;
	int lengthY = targetY - startY;

	//Sample top and bottom edge pixels
	bool flagUnknown = false;
	for (int x = startX; x <= targetX; ++x) {
		POINT p1(x, startY);
		POINT p2(x, targetY);

		data = Exploration::getCellData(p1, map);

		switch(data) {
		case EXPLORATION_OCCUPIED:
			switch (type) {
				case AstarGrid:
					return 0;
					break;
				case ExplorationGrid:
					return EXPLORATION_OCCUPIED;
					break;
			}
		case EXPLORATION_UNKNOWN:
			flagUnknown = true;
			break;
		}

		data = Exploration::getCellData(p2, map);
		switch(data) {
			case EXPLORATION_OCCUPIED:
				switch (type) {
					case AstarGrid:
						return 0;
						break;
					case ExplorationGrid:
						return EXPLORATION_OCCUPIED;
						break;
				}
			case EXPLORATION_UNKNOWN:
				flagUnknown = true;
				break;
		}
	}

	//Sample left and right edge pixels
	flagUnknown = false;
	for (int y = startY; y <= targetY; ++y) {
		POINT p1(startX, y);
		POINT p2(targetX, y);

		data = Exploration::getCellData(p1, map);

		switch(data) {
		case EXPLORATION_OCCUPIED:
			switch (type) {
				case AstarGrid:
					return 0;
					break;
				case ExplorationGrid:
					return EXPLORATION_OCCUPIED;
					break;
			}
		case EXPLORATION_UNKNOWN:
			flagUnknown = true;
			break;
		}

		data = Exploration::getCellData(p2, map);
		switch(data) {
			case EXPLORATION_OCCUPIED:
				switch (type) {
					case AstarGrid:
						return 0;
						break;
					case ExplorationGrid:
						return EXPLORATION_OCCUPIED;
						break;
				}
			case EXPLORATION_UNKNOWN:
				flagUnknown = true;
				break;
		}
	}

	if (flagUnknown)
		switch (type) {
			case AstarGrid:
				return 0;
				break;
			case ExplorationGrid:
				return EXPLORATION_UNKNOWN;
		}

	//At this line we've found out that all sample points are OPEN_SPACE
	return (type == AstarGrid) ? 1 : EXPLORATION_OPEN_SPACE;

//	//Sampling pixels
//	int numPoints = 3; //Points per row
//	for (int x = 0; x < numPoints; ++x) {
//		for (int y = 0; y < numPoints; ++y) {
//
//			POINT p(lengthX * x / (numPoints - 1) + startX,
//				    lengthY * y / (numPoints - 1) + startY);
//
//			data = Exploration::getCellData(p, map);
//
//			switch(data) {
//			case OCCUPIED:
//			case UNKNOWN:
//				switch (type) {
//					case AstarGrid:
//						return 0;
//						break;
//					case ExplorationGrid:
//						return data;
//				}
//				break;
//			}
//		}
//	}
//
//	//At this line we've found out that all sample points are OPEN_SPACE
//	return (type == AstarGrid) ? 1 : OPEN_SPACE;




//	int counter = 0;
//	for (int x = 1; x <= 3; ++x) {
//		for (int y = 1; y <= 3; ++y) {
//			POINT p((targetX - startX) * x / 4 + startX, y/4 + (targetY - startY) * y / 4 + startY);
//			data = Exploration::getCellData(p, map);
//
//			if (type == AstarGrid && data == OPEN_SPACE)
//				counter++;
//			else if (type == ExplorationGrid && result > 0 && result < CELL_THRESHOLD)
//				counter++;
//		}
//	}

//	switch (type) {
//	case AstarGrid:
//		if (data == OPEN_SPACE)
//			result = 1;
//		else
//			result = 0;
//		break;
//	case ExplorationGrid:
//		result = data;
//	}
//
//	return result;
}
