/*
 * PointHashSet.h
 *
 *  Created on: 12/10/2009
 *      Author: Matan
 */

#ifndef HASHING_H_
#define HASHING_H_

#include <utils/point.h>
#include <ext/hash_set>
#include <ext/hash_map>
#include <list>

using namespace GMapping;
using namespace __gnu_cxx;
using namespace std;

struct hash_point;
struct eq_point;
struct hash_point_hash_set;
struct eq_point_hash_set;


typedef IntPoint POINT;
typedef hash_map<POINT, int, hash_point, eq_point> PointHashMap;
typedef hash_set<POINT, hash_point, eq_point> POINT_HASHSET;
typedef list<POINT > POINT_LIST;
//typedef hash_set<POINT_HASHSET, hash_point_hash_set, eq_point_hash_set> FrontierSet;
typedef list<POINT_LIST > FRONTIER_LIST;
typedef pair<POINT, int> PointIntPair;

/** Hashing for OrientedPoint **/

//Hash code
struct hash_point {
	size_t operator() (const POINT& p) const {
		return p.x * 17 + p.y * 31;
	}
};

//Equals
struct eq_point {
	bool operator() (const POINT& p1, const POINT& p2) const {
		return p1.x == p2.x && p1.y == p2.y;
	}
};
/********************************/

/** Hashing for PointHashSet **/

//Hash code
struct hash_point_hash_set {
	size_t operator() (const POINT_HASHSET& phs) const {
		return hash_point().operator ()(*phs.begin());
	}
};

//Equals
struct eq_point_hash_set {
	bool operator() (const POINT_HASHSET& phs1, const POINT_HASHSET& phs2) const {
		return eq_point().operator ()(*phs1.begin(), *phs1.begin());
	}
};

/*********************************/



#endif /*HASHING_H_ */
