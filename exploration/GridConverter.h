/*
 * GridConverter.h
 *
 *  Created on: Nov 5, 2009
 *      Author: matan
 */

#ifndef GRIDCONVERTER_H_
#define GRIDCONVERTER_H_

#include "Exploration.h"

enum GridFormats {
	AstarGrid, ExplorationGrid
};

class GridConverter {
public:
	static int* createAstar(const MAP &, int);
	static int** createExploration(const MAP &, int, int, int**);
	static int* convertExp2Astar(int**, int, int);
private:
	static int classify(int, int, int, int, const MAP &, GridFormats);
	static int getCellWeight(int** map, int width, int height, int x, int y);

};

#endif /* GRIDCONVERTER_H_ */
