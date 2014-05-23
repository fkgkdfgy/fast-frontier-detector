/*
 * GridGenerator.h
 *
 *  Created on: 23/09/2009
 *      Author: Matan
 */

#ifndef GRIDGENERATOR_H_
#define GRIDGENERATOR_H_

#include "Grid.h"
#include <fstream>

using namespace std;

class GridGenerator {
public:
	static Grid create();
	static Grid readFile(char*);
};

#endif /* GRIDGENERATOR_H_ */
