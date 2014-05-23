/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 2.0)" 
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/2.0/
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include <unistd.h>
#include "gsp_thread.h"
#include <exploration/Exploration.h>
#include <exploration/WFD_Opt.h>
#include "../wolfram/FrontierExtractor.h"

#include <qapplication.h>
//#include <utils/point.h>
#include "qparticleviewer.h"
#include <qmainwindow.h>
#include <qstring.h>
#include <qcolor.h>
#include <exploration/GridConverter.h>

#include <utils/point.h>

using namespace GMapping;
using namespace mre;
//int intMap[4000][4000];

enum FrontierType {
	Wolfram,
	WFD,
	FFD
};

class MapViewer: public QMainWindow{
public:
	GridSlamProcessorThread* gsp_thread;
	QParticleViewer* pviewer;

	MapViewer(GridSlamProcessorThread* t){
	    gsp_thread=t;

	    format = "PNG";
	    prefix = "MAP";
	    frame = 0;

	    pviewer=new QParticleViewer(this, 0, 0, gsp_thread);
	    pviewer->setGeometry(0,0,500,500);
	    pviewer->setFocusPolicy(QParticleViewer::NoFocus);
	}

	void saveToFile() {
		cout << "SAVING TO FILE!" << endl;
		sprintf(buf,"%s-%05d.%s", prefix.c_str(), frame++, format.c_str());
		pviewer->saveToFile(buf);
		cout << "MAP SAVED TO FILE!" << endl;
	}

	int getCurrentFrame() { return frame;}

protected:
	char buf[512];
	std::string format;
	std::string prefix;
	int frame;
};

void drawActiveArea(QPainter& painter, vector<GMapping::IntPoint>& area, QPixmap* pixmap, ScanMatcherMap* bestMap, MapViewer* mapViewer, const QColor& color=Qt::blue) {
	double mapDelta = bestMap->getDelta();
	int robotWidthPixels = ROBOT_SIZE / mapDelta;
	int robotHeightPixels = ROBOT_SIZE / mapDelta;

	for (size_t i = 1; i < area.size(); ++i) {
//	for (size_t i = 0; i < area.size(); ++i) {
		GMapping::IntPoint currPoint = area[i];
		GMapping::IntPoint prevPoint = area[i-1];

//		QPen penFrontier(color, 1);
		QPen penFrontier(color, 2);
		painter.setPen(penFrontier);

		POINT pointGUI = currPoint;

		GMapping::Point lineSrcWorld;
		GMapping::Point lineDstWorld;

		lineSrcWorld = bestMap->map2world(  prevPoint.x, prevPoint.y);
		lineDstWorld = bestMap->map2world(  currPoint.x, currPoint.y);
		//DP("one = " << lineSrcWorld << " " << (mapViewer->pviewer->map2pic(lineSrcWorld)).x<<","<<(mapViewer->pviewer->map2pic(lineSrcWorld)).y)
//
//		lineSrcWorld = GMapping::Point(pointGUI.x* robotWidthPixels  + robotWidthPixels/2,
//									   pointGUI.y* robotHeightPixels  + robotHeightPixels/2);
//		DP("two = " << lineSrcWorld << " " <<(mapViewer->pviewer->map2pic(lineSrcWorld)).x<<","<<(mapViewer->pviewer->map2pic(lineSrcWorld)).y)
//
//		lineSrcWorld = bestMap->map2world(  pointGUI.x ,pointGUI.y );
//
//		DP("three = " << lineSrcWorld<< " " << (mapViewer->pviewer->map2pic(lineSrcWorld)).x<<","<<(mapViewer->pviewer->map2pic(lineSrcWorld)).y)
//
//		DP("offset = " << (GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2)).x<<","<<(GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2)).y)

		//GMapping::IntPoint lineSrcPic = mapViewer->pviewer->map2pic(lineSrcWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
		GMapping::IntPoint lineSrcPic = mapViewer->pviewer->map2pic(lineSrcWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
		GMapping::IntPoint lineDstPic = mapViewer->pviewer->map2pic(lineDstWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
		DP("drawActiveArea raw="<<pointGUI<<" map2world="<<lineSrcWorld<<" map2pic="<<lineSrcPic)
//		painter.drawEllipse(lineSrcPic.x, lineSrcPic.y, 3, 3);
		painter.drawLine(lineSrcPic.x, lineSrcPic.y, lineDstPic.x, lineDstPic.y);
		//cout << ", " ;
	}

	GMapping::Point lineSrcWorld = bestMap->map2world(  area[area.size()-1].x, area[area.size()-1].y);
	GMapping::Point lineDstWorld = bestMap->map2world(  area[0].x, area[0].y);
	GMapping::IntPoint lineSrcPic = mapViewer->pviewer->map2pic(lineSrcWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
	GMapping::IntPoint lineDstPic = mapViewer->pviewer->map2pic(lineDstWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
	painter.drawLine(lineSrcPic.x, lineSrcPic.y, lineDstPic.x, lineDstPic.y);
		//cout << "}" << endl;
}

void drawFrontiers(QPainter& painter, vector<POINT_LIST>& frontiers, QPixmap* pixmap, ScanMatcherMap* bestMap, MapViewer* mapViewer, FrontierType type) {

	double mapDelta = bestMap->getDelta();
	int robotWidthPixels = ROBOT_SIZE / mapDelta;
	int robotHeightPixels = ROBOT_SIZE / mapDelta;
//	ofstream fout("frontier_data.txt");
//	ofstream fmap("map_data.txt");
//	Map<double, DoubleArray2D, false>* pMap = bestMap->toDoubleMap();
//	for (int x = 0; x < pMap->getMapSizeX(); ++x) {
//		for (int y = 0; y < pMap->getMapSizeY(); ++y) {
//			fmap << pMap->cell(x,y) << " ";
//		}
//		fmap << endl;
//	}
//	delete pMap;
//
//	for (size_t i = 0; i < frontiers.size(); ++i) {
//		for (POINT_LIST::iterator frontierItr = frontiers[i].begin(); frontierItr != frontiers[i].end(); ++frontierItr) {
//			fout << frontierItr->x << " ";
//		}
//		fout << endl;
//		for (POINT_LIST::iterator frontierItr = frontiers[i].begin(); frontierItr != frontiers[i].end(); ++frontierItr) {
//			fout << frontierItr->y << " ";
//		}
//		fout << endl;
//	}

	DP("drawFrontiers got " << frontiers.size()<< " frontiers to draw")
	for (size_t i = 0; i < frontiers.size(); ++i) {
		POINT_LIST::iterator frontierItr;

		POINT_LIST currFrontier = frontiers[i];

		frontierItr = currFrontier.begin();
		POINT_LIST::iterator lastPoint = frontierItr;
		++frontierItr;
		QColor color;
		int currColor = i*255/frontiers.size();
		color.setHsv(currColor,255,255);
//		QPen penFrontier(color, 1);
		QPen penFrontier(Qt::darkGreen, 10);
		painter.setPen(penFrontier);

		POINT pointGUI;
		for (; frontierItr != currFrontier.end(); ++frontierItr) {
			pointGUI = POINT(frontierItr->x, frontierItr->y);

			GMapping::Point lineSrcWorld;

			switch (type) {
			case Wolfram:
				lineSrcWorld = bestMap->map2world(  lastPoint->x * robotWidthPixels  + robotWidthPixels/2,
													lastPoint->y * robotHeightPixels + robotHeightPixels/2);
				break;
			case WFD:
				lineSrcWorld = bestMap->map2world(  lastPoint->x * robotWidthPixels  + robotWidthPixels/2,
													lastPoint->y * robotHeightPixels + robotHeightPixels/2);
				break;
			case FFD:

//				lineSrcWorld = bestMap->map2world(  lastPoint->x * robotWidthPixels  + robotWidthPixels/2,
//													lastPoint->y * robotHeightPixels + robotHeightPixels/2);
				lineSrcWorld = bestMap->map2world(  lastPoint->x ,lastPoint->y );

//				GMapping::IntPoint p = bestMap->world2map(  lastPoint->x,
////														    lastPoint->y);
//				lineSrcWorld = GMapping::Point(p.x, p.y);
//				lineSrcWorld = GMapping::Point(lastPoint->x, lastPoint->y);
				break;
			}

			GMapping::Point lineDstWorld = bestMap->map2world(frontierItr->x * robotWidthPixels  + robotWidthPixels/2,
														      frontierItr->y * robotHeightPixels + robotHeightPixels/2);

			GMapping::IntPoint lineSrcPic = mapViewer->pviewer->map2pic(lineSrcWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);
			GMapping::IntPoint lineDstPic = mapViewer->pviewer->map2pic(lineDstWorld) + GMapping::IntPoint(pixmap->width()/2, pixmap->height()/2);


			GMapping::IntPoint myWorld2Map 	= bestMap->world2map(pointGUI.x, pointGUI.y);
			GMapping::IntPoint myMap2Pic 	= mapViewer->pviewer->map2pic(GMapping::Point(myWorld2Map.x,myWorld2Map.y));
//			DP("drawFrontiers raw="<<pointGUI<<" map2world="<<lineSrcWorld<<" map2pic="<<lineSrcPic)

//					" map2pic="<<(mapViewer->pviewer->map2pic(GMapping::Point(pointGUI.x, pointGUI.y))).x<<
//					         ","<<(mapViewer->pviewer->map2pic(GMapping::Point(pointGUI.x, pointGUI.y))).y)
	//					//Exploration::printPoint(*frontierItr);
	//
	//					Point lineSrcWorld = bestMap->map2world(lastPoint->x,lastPoint->y);
	//					Point lineDstWorld = bestMap->map2world(frontierItr->x,frontierItr->y);
	//
	//					IntPoint lineSrcPic = mapViewer->pviewer->map2pic(lineSrcWorld) + IntPoint(pixmap->width()/2, pixmap->height()/2);
	//					IntPoint lineDstPic = mapViewer->pviewer->map2pic(lineDstWorld) + IntPoint(pixmap->width()/2, pixmap->height()/2);

			//painter.drawLine(lineSrcPic.x, lineSrcPic.y, lineDstPic.x, lineDstPic.y);

			//painter.drawPoint(lineSrcPic.x, lineSrcPic.y);

//			DP("drawFrontiers draw lineSrcPic from " << lineSrcPic.x << " to " << lineSrcPic.y)
//			DP("drawFrontiers draw lineSrcWorld from " << lineSrcWorld.x << " to " << lineSrcWorld.y)
			painter.drawEllipse(lineSrcPic.x, lineSrcPic.y, 1, 1);

			lastPoint = frontierItr;
			//cout << ", " ;
		}

		//cout << "}" << endl;
	}
}

int  main (int argc, char ** argv){
  cerr << "GMAPPING copyright 2004 by Giorgio Grisetti, Cyrill Stachniss," << endl ;
  cerr << "and Wolfram Burgard. To be published under the CreativeCommons license," << endl ;
  cerr << "see: http://creativecommons.org/licenses/by-nc-sa/2.0/" << endl << endl;

  ScanMatcherMap * bestMap = 0;
  GMapping::OrientedPoint bestParticlePose;

	GridSlamProcessorThread* gsp=  new GridSlamProcessorThread;
	if (gsp->init(argc, argv)){
		cout << "GSP INIT ERROR" << endl;
		return -1;
	}
	cout <<"GSP INITIALIZED"<< endl;
	if (gsp->loadFiles()){
		cout <<"GSP READFILE ERROR"<< endl;
		return -2;
	}
	cout <<"FILES LOADED"<< endl;
	//gsp->setMapUpdateTime(1000000);

	gsp->setMapUpdateTime(1);
	gsp->setEventBufferSize(10000);

	gsp->start();
	cout <<"THREAD STARTED"<< endl;
	bool done=false;
	bool hasNewMap = false;

	ofstream fout("partial_ffd_executions.txt");
	ofstream foutContours("contour_lengths.txt");

	QApplication app(argc, argv);
	MapViewer* mapViewer = new MapViewer(gsp);

	double totalFFD = 0.0;
	int** grid = NULL;
	double prevFFDTime = totalFFD;

	WFD_Opt* wfd_opt = NULL;

	while (!done){
		GridSlamProcessorThread::EventDeque events=gsp->getEvents();
		for (GridSlamProcessorThread::EventDeque::iterator it=events.begin(); it!=events.end(); it++){
			cout << flush;

			// matan
			GridSlamProcessorThread::MapEvent* mapEvent=dynamic_cast<GridSlamProcessorThread::MapEvent*>(*it);
			if (mapEvent){
				hasNewMap=true;
			} else {
				GridSlamProcessorThread::DoneEvent* doneEvent=dynamic_cast<GridSlamProcessorThread::DoneEvent*>(*it);
				if (doneEvent){
					done=true;
					cout <<"DONE!"<< endl;
					gsp->stop();
				}
			}

		}

		if (hasNewMap) {
			hasNewMap = false;

			// DRAW MAP
			mapViewer->pviewer->drawFromEvents(events);

			bestMap = mapViewer->pviewer->getBestMap();
			bestParticlePose = mapViewer->pviewer->getBestParticlePose();
			//printf("\nBest particle pose: %f,%f,%f\n",bestParticlePose.x, bestParticlePose.y, bestParticlePose.theta);

			QPixmap* pixmap = mapViewer->pviewer->getPixmap();
			QPainter painter(pixmap);

			FrontierManager manager = gsp->m_particles[gsp->getBestParticleIndex()].frontierManager;
			vector<IntLine> ffdFrontiersTemp = manager.getFrontiers();

			long scanIndex = manager._sampleCounter;

			// scan each particle
//			for (size_t i = 0; gsp->m_particles.size(); ++i) {
//				totalFFD += gsp->m_particles[i].frontierManager.getLastExecution().currRuntime;
//			}
//			double totalFFD 	= manager.getLastExecution().totalTime;

			double lastFFD 		= manager.getLastExecution().currRuntime;
			double framesFFD 	= manager.getLastExecution().frames;

			double ffd_particle_times[30];

			int limit = gsp->m_particles.size();
			size_t lengths = 0;

			for (int i = 0; i < limit; ++i) {
				ffd_particle_times[i] 	= gsp->m_particles[i].frontierManager.getLastExecution().currRuntime;
				lengths 				+= gsp->m_particles[i].frontierManager.getLastExecution().length;
				totalFFD 				+= ffd_particle_times[i];//gsp->m_particles[i].frontierManager.getLastExecution().currRuntime;
				gsp->m_particles[i].frontierManager.resetBatchRuns();
				//			gsp->m_matcher.manager.resetBatchRuns();
			}

			foutContours << lengths << " " << totalFFD - prevFFDTime << " " << mapViewer->getCurrentFrame() << endl;
			prevFFDTime = totalFFD;

			cout << "MATAN MATAN MAP EVENT\n" ;
			fout << lastFFD << " " << totalFFD << " " << framesFFD << " " <<  scanIndex << " ";
			for (int i = 0; i < 30; ++i) {
				if (i >= limit) {
					fout << 0 << " ";
				} else {
					fout << ffd_particle_times[i] << " ";
				}
			}
			fout << endl;

			double mapDelta = bestMap->getDelta();
			int robotWidthPixels = ROBOT_SIZE / mapDelta;
			int robotHeightPixels = ROBOT_SIZE / mapDelta;


			int rows = bestMap->getMapSizeX() / robotWidthPixels;
			int cols = bestMap->getMapSizeY() / robotHeightPixels;

			// allocate memory to grid
			if (grid == NULL) {
				DP("mapSize = ("<<bestMap->getMapSizeX()<<","<<bestMap->getMapSizeY()<<")")
				DP("rows = "<<rows << " cols = "<<cols)
				grid = new int*[rows];

				for (int x = 0; x < rows; ++x) {
						grid[x] = new int[cols];
				}

			}

//#define RUN_WFD_OPT
#ifdef RUN_WFD_OPT
			if (wfd_opt == NULL) {
				wfd_opt = new WFD_Opt(bestMap->getMapSizeX() /robotWidthPixels, bestMap->getMapSizeY() / robotHeightPixels, "exploration_execution_wfd_opt.txt");
			}

			// run WFD Opt
			for (int i = 0; i < 30; ++i) {
				ScanMatcherMap* map = &(gsp->m_particles[i].map);
				GridConverter::createExploration(*map, robotWidthPixels, robotHeightPixels, grid);
				GMapping::OrientedPoint currParticlePose = gsp->m_particles[i].pose;
				POINT p = map->world2map(currParticlePose.x, currParticlePose.y);
				POINT currPose(p.x /robotWidthPixels, p.y /robotHeightPixels);

				wfd_opt->calcNext(currPose, grid, map->getMapSizeX() /robotWidthPixels, map->getMapSizeY() /robotHeightPixels, i);
			}
#endif

			// run WFD normal
			GridConverter::createExploration(*bestMap, robotWidthPixels, robotHeightPixels, grid);
//			GridConverter::createExploration(*bestMap, 1, 1, grid);

			Exploration rafaelReal(bestMap->getMapSizeX() /robotWidthPixels, bestMap->getMapSizeY() / robotHeightPixels, "exploration_execution_rafael.txt");
//			Exploration rafaelReal(bestMap->getMapSizeX() , bestMap->getMapSizeY(), "exploration_execution_rafael.txt");
			POINT p = bestMap->world2map(bestParticlePose.x, bestParticlePose.y);

			POINT currPose(p.x /robotWidthPixels, p.y /robotHeightPixels);
//			POINT currPose(p.x, p.y);

//			rafaelReal.calcNext(currPose, *(bestMap));
			rafaelReal.calcNext(currPose, grid, bestMap->getMapSizeX() /robotWidthPixels, bestMap->getMapSizeY() /robotHeightPixels);

//			for (int i = 0; i < bestMap->getMapSizeX() / robotWidthPixels; ++i) {
//			for (int i = 0; i < bestMap->getMapSizeX() ; ++i) {
//				delete [] grid[i];
//			}
//			delete [] grid;

			FrontierExtractor wolfram(FRONTIER_THRESHOLD);
			FrontierList wolframList;
			wolfram.computeFrontierCells(bestMap, wolframList);
			vector<POINT_LIST> wfdFrontiers = rafaelReal.getDetectedFrontiers();

			// draw map
//			mapViewer->pviewer->drawFromEvents(events);
//			bestMap = mapViewer->pviewer->getBestMap();
//			bestParticlePose = mapViewer->pviewer->getBestParticlePose();
//
//			QPixmap* pixmap = mapViewer->pviewer->getPixmap();
//			QPainter painter(pixmap);

			/*double mapDelta = bestMap->getDelta();
			int robotWidthPixels = ROBOT_SIZE / mapDelta;
			int robotHeightPixels = ROBOT_SIZE / mapDelta;
			int ** grid = GridConverter::createExploration(*bestMap, robotWidthPixels, robotHeightPixels);

			int imageWidth = mapViewer->pviewer->width();
			int imageHeight = mapViewer->pviewer->height();

			// Calculate image center (pixels, currently 250x250)
			GMapping::Point imageCenter = GMapping::Point(imageWidth >> 1, imageHeight >> 1);

			// DRAW GRID
			int gridWidth = bestMap->getMapSizeX() / robotWidthPixels;
			int gridHeight = bestMap->getMapSizeY() / robotHeightPixels;

			// World coordinates, in meters (taking into account image scale and viewCenter)
			GMapping::Point world_topleft_meter = GMapping::Point(
					mapViewer->pviewer->pic2map(GMapping::IntPoint(-pixmap->width()/2,pixmap->height()/2)));
			GMapping::Point world_bottomright_meter = GMapping::Point(
					mapViewer->pviewer->pic2map(GMapping::IntPoint(pixmap->width()/2,-pixmap->height()/2)));

			unsigned int curr_color = 0;
			int curr_cell = 0;
			GMapping::IntPoint curr_point_grid;

			double gridDeltaX = (world_bottomright_meter.x - world_topleft_meter.x) / imageWidth;
			double gridDeltaY = (world_bottomright_meter.y - world_topleft_meter.y) / imageHeight;
			int gridX, gridY;


			for (int y = 0; y < imageHeight; y++) {
				for(int x = 0; x < imageWidth; x++) {
					gridX = imageCenter.x + floor((world_topleft_meter.x + x*gridDeltaX)/ROBOT_SIZE);
					//gridY = imageCenter.y + floor((world_topleft_meter.y + y*gridDeltaY)/ROBOT_SIZE);
					gridY = imageCenter.y + floor((world_topleft_meter.y + (imageHeight-1-y)*gridDeltaY)/ROBOT_SIZE); // vertically mirrored gridY

					curr_point_grid = POINT(gridX, gridY);

					curr_cell = Exploration::getCellData(curr_point_grid, grid, gridWidth, gridHeight);


					if (curr_cell != EXPLORATION_OPEN_SPACE) {
						if (curr_cell == EXPLORATION_OCCUPIED)
							curr_color = 0;
						else if (curr_cell == EXPLORATION_UNKNOWN)
							curr_color = 128;
						painter.setBackgroundMode(Qt::TransparentMode);
						//painter.fillRect(x*grid_cell_width_pixels, y*grid_cell_height_pixels, grid_cell_width_pixels, grid_cell_height_pixels, QBrush(QColor(curr_color, curr_color, curr_color), Qt::Dense4Pattern));
						//painter.drawPoint(x, y);
						painter.fillRect(x, y, 1, 1, QBrush(QColor(curr_color, curr_color, curr_color), Qt::Dense4Pattern));
					} else {
						//cout << curr_point_grid.x << "," << curr_point_grid.y << ": " << grid[curr_point_grid.x][curr_point_grid.y] << endl;
					}

				}
			}*/

			vector<POINT_LIST> wolframFrontiers;
			vector<POINT_LIST> ffdFrontiers;

			for (FrontierListIterator i = wolframList.begin(); i != wolframList.end(); ++i) {
				POINT_LIST currFrontier;
				for (PointListIterator j = (*i)->points.begin(); j != (*i)->points.end(); ++j) {
					currFrontier.push_back(POINT(j->x, j->y));
				}
				//DP("wolfram frontier size " <<currFrontier.size())
				wolframFrontiers.push_back(currFrontier);
			}

			for (vector<IntLine>::iterator i = ffdFrontiersTemp.begin(); i != ffdFrontiersTemp.end(); ++i) {
				POINT_LIST currFrontier;

				for (IntLine::iterator j = i->begin(); j != i->end(); ++j) {
					currFrontier.push_back(POINT(j->x, j->y));
				}

				ffdFrontiers.push_back(currFrontier);
			}

			DP("wolfram has "<<wolframFrontiers.size()<<" frontiers")
			DP("rafael has "<<wfdFrontiers.size()<<" frontiers")
			DP("FFD has " << ffdFrontiers.size() << " frontiers")

			DP("_activeArea.size = " << manager._activeArea.size());
			drawFrontiers(painter, ffdFrontiers, pixmap, bestMap, mapViewer, FFD);
//			drawFrontiers(painter, wfdFrontiers, pixmap, bestMap, mapViewer, WFD);
//			drawFrontiers(painter, wolframFrontiers, pixmap, bestMap, mapViewer, Wolfram);
			QColor activeAreaColor = (lengths > 150000) ? Qt::red : Qt::blue;
			drawActiveArea(painter, manager._activeAreaDebug, pixmap, bestMap, mapViewer, activeAreaColor);

//			drawFrontiers(painter, frontiers, pixmap, bestMap, mapViewer);


			mapViewer->saveToFile();

			/*DoubleArray2D* pMap = bestMap->toDoubleArray();
			int sizeX, sizeY;
			sizeX = pMap->getXSize();
			sizeY = pMap->getYSize();
			DP("pMap size = [" << sizeX<<","<<sizeY<<"]")
			DP("after allocation of pMap")
			for (int x = 0; x < sizeX; ++x) {
				for (int y = 0; y < sizeY; ++y) {
					int v = pMap->cell(x,y);
					if (v >= 0) {
						int grayValue = 255-(int)(255.*v);
						intMap[x][(y+2000)%4000] = v;
					} else {
						intMap[x][(y+2000)%4000] = 100;
					}
				}
			}
			DP("before intarray")
			char buff[20];
			static int counter = 0;
			sprintf(buff,"foo%d.bmp", counter++);
			bool result = intarray2bmp::intarray2bmp( buff, &(intMap[0][0]), sizeX,sizeY, 0, 255 );
			DP("after intarray")
			delete pMap;*/


			// HACK: clear frontiers
			for (FrontierListIterator itr = wolframList.begin(); itr != wolframList.end(); ++itr) {
				delete *itr;
			}
			wolframList.clear();
		}
//			if (*it)
//				delete(*it);

	}

	fout.close();
	foutContours.close();
}
