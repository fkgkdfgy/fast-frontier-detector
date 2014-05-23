/*
 * BitMap.cpp
 *
 *  Created on: Nov 18, 2009
 *      Author: matan
 */

#include "BitMap.h"

BitMap::BitMap(int sizeX, int sizeY) {
	_sizeX = sizeX;
	_sizeY = sizeY;

	_data = new int[sizeX*sizeY];

	//initialization
	int size = _sizeX * _sizeY;
	for (int i = 0; i < size; ++i)
		_data[i] = 0;
}

BitMap::~BitMap() {
	delete [] _data;
}

bool BitMap::validate(int x, int y) {
	if (x < 0 || y < 0)
		return false;

	if (x < _sizeX && y < _sizeY)
		return true;

	return false;
}

int BitMap::get(int x, int y) {
	if (!validate(x, y))
		return -1;

	return _data[x + y * _sizeY];
}

void BitMap::set(int x, int y, int value) {
	if (!validate(x, y))
		return;

	_data[x + y * _sizeY] = value;
}

bool BitMap::isState(int x, int y, int state) {
	if (!validate(x, y))
		return false;

	int value = _data[x + y * _sizeY];

	return (value & state);
}

void BitMap::clear() {
	//initialization
	int size = _sizeX * _sizeY;
	for (int i = 0; i < size; ++i)
		_data[i] = 0;
}
