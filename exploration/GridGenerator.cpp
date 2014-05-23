/*
 * GridGenerator.cpp
 *
 *  Created on: 23/09/2009
 *      Author: Matan
 */

#include "GridGenerator.h"


Grid GridGenerator::create() {
	Grid sample(G_GRID_SIZE);

	//sample.setCurrentPosition(1, 1); //HACK

	int turn = (int)(G_GRID_SIZE * 0.75);
	int width = 4;

	for (int i = 0; i < turn; ++i)
		sample.setCell(width, i, G_BORDER);

	for (int i = width; i < G_GRID_SIZE; ++i)
		sample.setCell(i, turn, G_BORDER);

	return sample;
}

Grid GridGenerator::readFile(char* fileName) {
	ifstream in;


	in.open(fileName, ifstream::in);

	cout << in.good() << endl;


	int size;
	in >> size;

	Grid grid(size);
	CELL buffer[size + 1];

	in.getline(buffer, size + 1); //garbage

	for (int i = 0; i < size; ++i) {

		in.getline(buffer, size + 1);

		for (int j = 0; j < size; ++j) {
			if (buffer[j] == G_CURR_POSITION)
				grid.setCell(i, j, G_OPEN_SPACE);
			else
				grid.setCell(i, j, buffer[j]);
		}
	}

	in.close();

	return grid;
}
