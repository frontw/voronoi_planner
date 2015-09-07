#include "dynamicvoronoi.h"

#include <iostream>
#include <fstream>
#include <string.h>


#include <voronoi_planner/planner_core.h>

using namespace voronoi_planner;

void loadPGM( std::istream &is, int *sizeX, int *sizeY, bool ***map ) {
    std::string tag;
    is >> tag;
    if (tag!="P5") {
        std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
        exit(-1);
    }

    while (is.peek()==' ' || is.peek()=='\n') is.ignore();
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> *sizeX;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> *sizeY;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> tag;
    if (tag!="255") {
        std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
        exit(-1);
    }
    
    *map = new bool*[*sizeX];

    for (int x=0; x<*sizeX; x++) {
        (*map)[x] = new bool[*sizeY];
    }
    for (int y=*sizeY-1; y>=0; y--) {
        for (int x=0; x<*sizeX; x++) {
            int c = is.get();
            if ((double)c<255-255*0.2) (*map)[x][y] = true; // cell is occupied
            else (*map)[x][y] = false; // cell is free
            if (!is.good()) {
                std::cerr << "Error reading pgm map.\n";
                exit(-1);
            }
        }
    }
}


int main( int argc, char *argv[] ) {

    if(argc<2 || argc>3 || (argc==3 && strcmp(argv[2],"prune")!=0)) {
        std::cerr<<"usage: "<<argv[0]<<" <pgm map> [prune]\n";
        exit(-1);
    }

    bool doPrune = false;
    if (argc==3) doPrune = true;
    // LOAD PGM MAP AND INITIALIZE THE VORONOI
    std::ifstream is(argv[1]);
    if (!is) {
        std::cerr << "Could not open map file for reading.\n";
        exit(-1);
    }

    bool **map=NULL;
    int sizeX, sizeY;
    loadPGM( is, &sizeX, &sizeY, &map );
    is.close();
    fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

    // create the voronoi object and initialize it with the map
    DynamicVoronoi voronoi;
    voronoi.initializeMap(sizeX, sizeY, map);
    voronoi.update(); // update distance map and Voronoi diagram
    if (doPrune) voronoi.prune();  // prune the Voronoi









//    std::vector<std::pair<float, float> > path1;
//    std::vector<std::pair<float, float> > path2;
//    std::vector<std::pair<float, float> > path3;


//    int goal_x = 876, goal_y = 156;
//    int start_x = 94, start_y = 51;

//    std::cout << "size xy " << voronoi.getSizeX() << " " << voronoi.getSizeY() << std::endl;

//    if( !voronoi.isVoronoi(goal_x,goal_y) )
//    {
//        //        path3 = findPath( goal, init, A, 0, 1 );
//        findPath( &path3, goal_x, goal_y, start_x, start_y, &voronoi, 0, 1 );
//        //        goal = path3(end,:);
//        goal_x = std::get<0>( path3[path3.size()-1] );
//        goal_y = std::get<1>( path3[path3.size()-1] );
//        //        path3 = flipud(path3);
//        std::reverse(path3.begin(), path3.end());
//    }

//    if( !voronoi.isVoronoi(start_x,start_y) )
//    {
//        findPath( &path1, start_x, start_y, goal_x, goal_y, &voronoi, 0, 1 );
//        start_x = std::get<0>( path1[path1.size()-1] );
//        start_y = std::get<1>( path1[path1.size()-1] );

//        std::cout << "voronoi.isVoronoi(start_x,start_y) " << voronoi.isVoronoi(start_x,start_y) << std::endl;
//    }

//    findPath( &path2, start_x, start_y, goal_x, goal_y, &voronoi, 1, 0 );

////    path = [path1;path2;path3];
//    path1.insert( path1.end(), path2.begin(), path2.end() );
//    path1.insert( path1.end(), path3.begin(), path3.end() );


//    std::vector<IntPoint> newObstacles;

//    for(int i = 0; i < path1.size(); i++)
//    {
//        int x = std::get<0>(path1[i]);
//        int y = std::get<1>(path1[i]);
//        newObstacles.push_back(IntPoint(x,y));
//    }
//    voronoi.exchangeObstacles(newObstacles);




//    voronoi.visualize("initial.ppm");
//    std::cerr << "Generated initial frame.\n";







    // now perform some updates with random obstacles
    char filename[20];
    int numPoints = 10 + sizeX*sizeY*0.005;
    for (int frame=1; frame<=10; frame++) {
        std::vector<IntPoint> newObstacles;
        for (int i=0; i<numPoints; i++) {
            double x = 2+rand()/(double)RAND_MAX*(sizeX-4);
            double y = 2+rand()/(double)RAND_MAX*(sizeY-4);
            newObstacles.push_back(IntPoint(x,y));
        }
        voronoi.exchangeObstacles(newObstacles); // register the new obstacles (old ones will be removed)
        voronoi.update();
        if (doPrune) voronoi.prune();
        sprintf(filename, "update_%03d.ppm", frame);
        voronoi.visualize(filename);
        std::cerr << "Performed update with random obstacles.\n";
    }

    // now remove all random obstacles again.
    // final.pgm should be very similar to initial.pgm, except for ambiguous spots
    std::vector<IntPoint> empty;
    voronoi.exchangeObstacles(empty);
    voronoi.update();
    if (doPrune) voronoi.prune();
    voronoi.visualize("final.ppm");
    std::cerr << "Done with final update (all random obstacles removed).\n";
    std::cerr << "Check initial.ppm, update_???.ppm and final.ppm.\n";















    return 0;
}
