#include "pathplanner.h"
#include "genLocalMap.h"
#include <set>

AStar::AStar(const LocalMap& localmap_) : localmap_(localmap_){
    cell.conservativeResize(localmap_.height, localmap_.width);
    path.conservativeResize(1,2);
    refinedPath.conservativeResize(1,2);
}

bool AStar::isDestination(const GridPoint& node){
    if(node.y==localmap_.goal.y && node.x==localmap_.goal.x){
        return true;
    }
    else{
        return false;
    }
}

bool AStar::isStart(const GridPoint& node){
    if(node.y==localmap_.start.y && node.x==localmap_.start.x){
        return true;
    }
    else{
        return false;
    }
}

bool AStar::bresenhamlineAlgo(int x0, int x1, int y0, int y1){
    int stepx, stepy;
    int dx = x1 - x0;
    int dy = y1 - y0;
    if(dy < 0){
        dy = -dy;
        stepy = -1;
    }
    else{
        stepy = 1;
    }
    if(dx < 0){
        dx = -dx;
        stepx = -1;
    }
    else{
        stepx = 1;
    }
    dy <<= 1;
    dx <<= 1;
    if(dx > dy){
        int fraction = dy - (dx >> 1);
        while(x0 != x1){
            x0 += stepx;
            if(fraction >= 0){
                y0 += stepy;
                fraction -= dx;
            }
            fraction += dy;
            if(localmap_.grid(y0, x0) == 100){
                return false;
            }
            else{ if((x0 < 0)||(x0 >= localmap_.grid.cols())||(y0 < 0)||(y0 >= localmap_.grid.rows())){
                    return false;
                }
            }
        }
        return true;
    }
    else{
        int fraction = dx - (dy >> 1);
        while(y0 != y1){
            y0 += stepy;
            if(fraction >= 0){
                x0 += stepx;
                fraction -= dy;
            }
            fraction += dx;
            if(localmap_.grid(y0, x0) == 100){
                return false;
            }
            else{ if((x0 < 0)||(x0 >= localmap_.grid.cols())||(y0 < 0)||(y0 >= localmap_.grid.rows())){
                    return false;
                }
            }
        }
        return true;
    }
}

void AStar::tracePath(int k, int l){
    path(0,0) = k;
    path(0,1) = l;
    while(isStart({path(path.rows()-1,0),path(path.rows()-1,1)})==false){
        k = path(path.rows()-1,0);
        l = path(path.rows()-1,1);
        GridPoint next_node = cell(k,l).parent;
        if(isStart(next_node)){
            path.conservativeResize(path.rows()+1,2);
            path(path.rows()-1,0) = next_node.y;
            path(path.rows()-1,1) = next_node.x;
            return;
        }
        GridPoint last_node;
        while((calNorm({k,l},next_node) < segmentLength)&&bresenhamlineAlgo(l,next_node.x,k,next_node.y)){
            if(isStart(next_node)){
                path.conservativeResize(path.rows()+1,2);
                path(path.rows()-1,0) = next_node.y;
                path(path.rows()-1,1) = next_node.x;
                return;
            }
            last_node = next_node;
            next_node = cell(next_node.y, next_node.x).parent;
        }
        //std::cout << "current node traced" << k << " "<< l << std::endl;
        path.conservativeResize(path.rows()+1,2);
        path(path.rows()-1,0) = last_node.y;
        path(path.rows()-1,1) = last_node.x;
    }
    //return path_;
}

/*Eigen::MatrixX2i AStar::processPath(){
    int counter1 = 1;
    int counter2 = 2;
    //Eigen::MatrixX2i refinedPath(1,2);
    while(counter2 < path.rows() - 1){
        refinedPath(0,0) = path(counter1,0);
        refinedPath(0,1) = path(counter1,1);
        if(bresenhamlineAlgo(path(counter1,1), path(counter2+1,1), path(counter1,0), path(counter2+1,0))&&
        (calNorm(path(counter1,1), path(counter1,0), path(counter2+1,1), path(counter2+1,0))<segmentLength)){
            counter2 += 1;
            continue;
        }
        else{ if(bresenhamlineAlgo(path(counter1,1), path(counter2+1,1), path(counter1,0), path(counter2+1,0))&&
        (calNorm(path(counter1,1), path(counter1,0), path(counter2+1,1), path(counter2+1,0))>segmentLength)){
            refinedPath.conservativeResize(refinedPath.rows()+1,2);
            refinedPath(refinedPath.rows()-1,0) = path(counter2+1,0);
            refinedPath(refinedPath.rows()-1,1) = path(counter2+1,1);
            counter1 = counter2+1;
            counter2 = counter1+1;
            continue;
        }
        else{
            refinedPath.conservativeResize(refinedPath.rows()+1,2);
            refinedPath(refinedPath.rows()-1,0) = path(counter2,0);
            refinedPath(refinedPath.rows()-1,1) = path(counter2,1);
            counter1 = counter2;
            counter2 = counter1+1;
            continue;
        }
        }
    }
    return refinedPath;
}*/

Eigen::MatrixX2i AStar::returnPath(){
    std::cout << "starting node = " << localmap_.start.y << " " << localmap_.start.x  << std::endl;
    std::cout << "goal node = " << localmap_.goal.y << " " << localmap_.goal.x  << std::endl;
    i = localmap_.start.y;
    j = localmap_.start.x;
    if(isDestination({i,j})){
        path(0,0) = localmap_.start.y;
        path(0,1) = localmap_.start.x;
        return path;
    }
    bool closedList[localmap_.height][localmap_.width];
    memset(closedList, false, sizeof(closedList));
    //const int row = localmap_.height;
    //const int col = localmap_.width;
    
    for (int k = 0; k < localmap_.height; k++) {
        for (int l = 0; l < localmap_.width; l++) {
            cell(k,l).f = FLT_MAX;
            cell(k,l).g = FLT_MAX;
            cell(k,l).h = FLT_MAX;
            cell(k,l).parent = {-1,-1};
            //cell(k,l).parent_j = -1;
        }
    }
    
    cell(localmap_.start.y, localmap_.start.x).g = 0.0;
    cell(localmap_.start.y, localmap_.start.x).h = calNorm(localmap_.start, localmap_.goal);
    cell(localmap_.start.y, localmap_.start.x).f = cell(localmap_.start.y, localmap_.start.x).h;
    cell(localmap_.start.y, localmap_.start.x).parent = localmap_.start;
    //cell(localmap_.start.y, localmap_.start.x).parent_j = localmap_.start.x;

    std::set<cellCostWrapper> openList;
    //std::set<cellCostWrapper>::iterator itr;
    std::cout << "just to start exploring nodes" << std::endl;
    openList.insert(cellCostWrapper(std::make_pair(cell(localmap_.start.y, localmap_.start.x).f, localmap_.start)));
    while (!openList.empty()) {
        /*for(itr = openList.begin(); itr!= openList.end(); itr++){
            std::cout << itr->first << std::endl;
        }*/
        //itr = openList.begin();
        //std::cout << itr->first << "  " ;
        cellCostWrapper p = *openList.begin();
        
        i = p.obj.second.y;
        j = p.obj.second.x;
        openList.erase(openList.begin());
        closedList[i][j] = true;
        double gNew, hNew, fNew;
        //std::cout << "current node" << i << " "<< j<< std::endl;
//////////////////////////////////////////////////////////////////////  (i-1, j)
        if (i-1 >= 0) {
            if (isDestination({i-1,j})) {
                cell(i - 1, j).parent = {i,j};
                //cell(i - 1, j).parent_j = j;
                std::cout << "destination node found, tracing path! " << i-1 << " "<< j<< std::endl;
                //foundDest = true;
                tracePath(i-1, j);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i - 1][j] == false
                     && localmap_.grid(i - 1,j) != 100 ) {
                gNew = cell(i,j).g + 1.0;
                hNew = calNorm({i - 1, j}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i - 1, j).f == FLT_MAX || cell(i - 1, j).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i-1,j})));
                    //std::cout << fNew << " ";
                    cell(i - 1, j).f = fNew;
                    cell(i - 1, j).g = gNew;
                    cell(i - 1, j).h = hNew;
                    cell(i - 1, j).parent = {i,j};
                    //cell(i - 1, j).parent_j = j;
                }
            }
        }
////////////////////////////////////////////////////////////////////////////// (i+1, j)
        if (i+1 < localmap_.height) {
            if (isDestination({i+1,j})) {
                cell(i + 1, j).parent = {i,j};
                //cell(i + 1, j).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                
                std::cout << "destination node found, tracing path! " << i+1 << " "<< j<< std::endl;
                //foundDest = true;
                tracePath(i+1,j);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i + 1][j] == false
                     && localmap_.grid(i + 1,j) != 100 ) {
                gNew = cell(i,j).g + 1.0;
                hNew = calNorm({i + 1, j}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i + 1, j).f == FLT_MAX || cell(i + 1, j).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i + 1, j})));
                    //std::cout << fNew << " ";
                    cell(i + 1, j).f = fNew;
                    cell(i + 1, j).g = gNew;
                    cell(i + 1, j).h = hNew;
                    cell(i + 1, j).parent = {i,j};
                    //cell(i + 1, j).parent_j = j;
                }
            }
        }
////////////////////////////////////////////////////////////////////////// (i, j+1)
        if (j+1 < localmap_.width) {
            if (isDestination({i, j+1})) {
                cell(i, j + 1).parent = {i,j};
                //cell(i, j + 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i << " "<< j+1<< std::endl;
                //foundDest = true;
                tracePath(i,j+1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i][j + 1] == false
                     && localmap_.grid(i,j + 1) != 100 ) {
                gNew = cell(i, j).g + 1.0;
                hNew = calNorm({i, j + 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i, j + 1).f == FLT_MAX || cell(i, j + 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i, j + 1})));
                    //std::cout << fNew << " ";
                    cell(i, j + 1).f = fNew;
                    cell(i, j + 1).g = gNew;
                    cell(i, j + 1).h = hNew;
                    cell(i, j + 1).parent = {i,j};
                    //cell(i, j + 1).parent_j = j;
                }
            }
        }
//////////////////////////////////////////////////////////////////////// (i, j-1)
        if (j-1 >= 0) {
            if (isDestination({i, j-1})) {
                cell(i, j - 1).parent = {i,j};
                //cell(i, j - 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i << " "<< j-1<< std::endl;
                //foundDest = true;
                tracePath(i,j-1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i][j - 1] == false
                     && localmap_.grid(i, j - 1) != 100 ) {
                gNew = cell(i, j).g + 1.0;
                hNew = calNorm({i, j - 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i, j - 1).f == FLT_MAX || cell(i, j - 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i, j - 1})));
                    //std::cout << fNew << " ";
                    cell(i, j - 1).f = fNew;
                    cell(i, j - 1).g = gNew;
                    cell(i, j - 1).h = hNew;
                    cell(i, j - 1).parent = {i,j};
                    //cell(i, j - 1).parent_j = j;
                }
            }
        }
/////////////////////////////////////////////////////////////////////////////// (i-1, j+1)
        if (i-1 >=0 && j+1 < localmap_.width) {
            if (isDestination({i-1, j+1})) {
                cell(i - 1, j + 1).parent = {i,j};
                //cell(i - 1, j + 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i-1 << " "<< j+1<< std::endl;
                //foundDest = true;
                tracePath(i-1,j+1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i - 1][j + 1] == false
                     && localmap_.grid(i - 1,j + 1) != 100 ) {
                gNew = cell(i, j).g + 1.414;
                hNew = calNorm({i - 1, j + 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i - 1, j + 1).f == FLT_MAX || cell(i - 1, j + 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i - 1, j + 1})));
                    //std::cout << fNew << " ";
                    cell(i - 1, j + 1).f = fNew;
                    cell(i - 1, j + 1).g = gNew;
                    cell(i - 1, j + 1).h = hNew;
                    cell(i - 1, j + 1).parent = {i,j};
                    //cell(i - 1, j + 1).parent_j = j;
                }
            }
        }
////////////////////////////////////////////////////////////////////////////// (i-1, j-1)
        if (i-1 >=0 && j-1 >= 0) {
            if (isDestination({i-1, j-1})) {
                cell(i - 1, j - 1).parent = {i,j};
                //cell(i - 1, j - 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i-1 << " "<< j-1<< std::endl;
                //foundDest = true;
                tracePath(i-1,j-1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i - 1][j - 1] == false
                     && localmap_.grid(i - 1,j - 1) != 100 ) {
                gNew = cell(i, j).g + 1.414;
                hNew = calNorm({i - 1, j - 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i - 1, j - 1).f == FLT_MAX || cell(i - 1, j - 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i - 1, j - 1})));
                    //std::cout << fNew << " ";
                    cell(i - 1, j - 1).f = fNew;
                    cell(i - 1, j - 1).g = gNew;
                    cell(i - 1, j - 1).h = hNew;
                    cell(i - 1, j - 1).parent = {i,j};
                    //cell(i - 1, j - 1).parent_j = j;
                }
            }
        }
///////////////////////////////////////////////////////////////////////////////// (i+1, j+1)
        if (i+1 < localmap_.height && j+1 < localmap_.width) {
            if (isDestination({i+1, j+1})) {
                cell(i + 1, j + 1).parent = {i,j};
                //cell(i + 1, j + 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i+1 << " "<< j+1<< std::endl;
                //foundDest = true;
                tracePath(i+1,j+1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i + 1][j + 1] == false
                     && localmap_.grid(i + 1,j + 1) != 100 ) {
                gNew = cell(i, j).g + 1.414;
                hNew = calNorm({i + 1, j + 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i + 1, j + 1).f == FLT_MAX || cell(i + 1, j + 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i + 1, j + 1})));
                    //std::cout << fNew << " ";
                    cell(i + 1, j + 1).f = fNew;
                    cell(i + 1, j + 1).g = gNew;
                    cell(i + 1, j + 1).h = hNew;
                    cell(i + 1, j + 1).parent = {i,j};
                    //cell(i + 1, j + 1).parent_j = j;
                }
            }
        }
//////////////////////////////////////////////////////////////////////////////// (i+1, j-1)
        if (i+1 < localmap_.height && j-1 >= 0) {
            if (isDestination({i+1, j-1})) {
                cell(i + 1, j - 1).parent = {i,j};
                //cell(i + 1, j - 1).parent_j = j;
                //tracePath(cell, dest);
                //foundDest = true;
                std::cout << "destination node found, tracing path! " << i+1 << " "<< j-1<< std::endl;
                //foundDest = true;
                tracePath(i+1,j-1);
                std::cout << "path tracing successful " << std::endl;
                return path;
            }
            else if (closedList[i + 1][j - 1] == false
                     && localmap_.grid(i + 1,j - 1) != 100 ) {
                gNew = cell(i, j).g + 1.414;
                hNew = calNorm({i + 1, j - 1}, localmap_.goal);
                fNew = gNew + hNew;
                if (cell(i + 1, j - 1).f == FLT_MAX || cell(i + 1, j - 1).f > fNew) {
                    openList.insert(cellCostWrapper(std::make_pair<float, GridPoint>(fNew, {i + 1, j - 1})));
                    //std::cout << fNew << " ";
                    cell(i + 1, j - 1).f = fNew;
                    cell(i + 1, j - 1).g = gNew;
                    cell(i + 1, j - 1).h = hNew;
                    cell(i + 1, j - 1).parent = {i,j};
                    //cell(i + 1, j - 1).parent_j = j;
                }
            }
        }
        //std::cout << "\n";
    }         

    path(0,0) = localmap_.start.y;
    path(0,1) = localmap_.start.x;
    return path;
}




/*std::vector<node> JPS::pruneNeighbours(){

}

void JPS::addSuccessorNodes(){
    neighbours = pruneNeighbours();
    for(node i : neighbours){
        node successor_node;
        successor_node = jumpPoint(i);

    }
}

node JPS::jumpPoint(const node& neighbour){

}*/


