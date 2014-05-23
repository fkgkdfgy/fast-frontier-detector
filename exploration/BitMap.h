/*
 * BitMap.h
 *
 *  Created on: Nov 18, 2009
 *      Author: matan
 */

#ifndef BITMAP_H_
#define BITMAP_H_

class BitMap {
private:
	int* _data;
	int _sizeX;
	int _sizeY;

	bool validate(int, int);
public:
	BitMap(int, int);
	virtual ~BitMap();

	int get(int, int);
	bool isState(int, int, int);
	void set(int, int, int);
	void clear();
};

#endif /* BITMAP_H_ */
