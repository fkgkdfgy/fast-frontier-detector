#include "Exploration.h"
#include "GridGenerator.h"

#include <ext/hash_map>



int main() {



	Exploration exp(4000, 4000,"tester.txt");
//
//	Grid grid = GridGenerator::readFile("grid.txt");
//
//	cout << grid << endl;
//
//	POINT p(5,7);
//	POINT next = exp.calcNext(p, grid);
//
//	cout << "Next move: " ;
//	Exploration::printPoint(next);
//	cout << endl;
//
//	cout << "frontiers found: " << exp.getDetectedFrontiers().size() << endl;
//	cout << endl;
//
//	vector<PointHashSet> frontiers = exp.getDetectedFrontiers();
//
//	for (int i = 0; i < frontiers.size(); ++i) {
//		PointHashSet::iterator itr;
//
//		cout << "{";
//		for (itr = frontiers[i].begin(); itr != frontiers[i].end(); ++itr) {
//			Exploration::printPoint(*itr);
//			cout << ", " ;
//		}
//		cout << "}" << endl;
//	}


}

