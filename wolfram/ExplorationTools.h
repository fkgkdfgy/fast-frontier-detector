#define WOLFRAM_THRESHOLD 0.5
#define WOLFRAM_OPEN_SPACE  0
#define WOLFRAM_OCCUPIED  1
#define WOLFRAM_UNKNOWN  -1


#include <list>
#include <scanmatcher/smmap.h>
using namespace std;
using namespace GMapping;

typedef GMapping::IntPoint POINT;
typedef GMapping::ScanMatcherMap MAP;

bool isAllocated(POINT p, const MAP &map) {
	return (map.storage().cellState(p) == (Inside | Allocated) );
}

int getCellData(POINT p, const MAP &map) {

	if (!isAllocated(p, map))
		return WOLFRAM_UNKNOWN;

	double data = map.storage().cell(p);

	if (data == WOLFRAM_UNKNOWN)
		return WOLFRAM_UNKNOWN;

	return (data > WOLFRAM_THRESHOLD ? WOLFRAM_OCCUPIED : WOLFRAM_OPEN_SPACE);
}

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

bool isFrontierCell(MAP &map, POINT &p) {
	if (getCellData(p, map) != WOLFRAM_UNKNOWN) //HACK
			return false;

	//get the neighbors of current cell
	list<POINT> neighbors = getNeighbors(p.x, p.y, map);

	//ignore errors
	if (neighbors.size() == 0)
		return false;

	//collect the adjacent cells that are known and not limits
	list<POINT>::iterator itr;
	for (itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		//if (getCellData(map.storage().cell(*itr)) == OPEN_SPACE)
		//if ( isKnownCell(*itr, map))
		if (getCellData(*itr, map) == WOLFRAM_OPEN_SPACE)
			return true;
	}

	return false;
}
