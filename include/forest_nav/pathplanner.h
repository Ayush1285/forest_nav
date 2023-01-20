#ifndef FOREST_NAV_PATHPLANNER_H
#define FOREST_NAV_PATHPLANNER_H

#include <utility>
#include "genLocalMap.h"
#include "math_lib.h"

class AStar
{
    public:
        AStar(const LocalMap& localmap);
        bool isDestination(const GridPoint& node);
        bool isStart(const GridPoint& node);
        void tracePath(int k, int l);
        //Eigen::MatrixX2i processPath();
        bool bresenhamlineAlgo(int x0, int x1, int y0, int y1);
        Eigen::MatrixX2i returnPath();

    private:
        int i,j;
        LocalMap localmap_;
        Eigen::Matrix<int,-1,2> path;
        Eigen::MatrixX2i refinedPath;
        Eigen::Matrix<GridCell, -1, -1> cell;
};

/*class JPS{
    public:
        JPS(const LocalMap& localmap);
        bool isDestination();
        std::vector<node> pruneNeighbours();
        void addSuccessorNodes();
        node jumpPoint(node);

    private:
        node current_node;
        std::vector<node> neighbours;
        std::set<cellCost> openList;

};*/



int segmentLength=12;

#endif
