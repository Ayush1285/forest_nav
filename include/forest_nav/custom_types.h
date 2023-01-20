#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H 


struct GridPoint
{       
    int x,y;
    GridPoint(){}
    GridPoint(int y_, int x_) : y(y_), x(x_){}
    void operator= (const GridPoint& point){x = point.x; y = point.y;}
    bool operator==(const GridPoint& point)const{return(x==point.x && y==point.y);}
    bool operator!=(const GridPoint& point)const{return(x!=point.x || y!=point.y);}
};

typedef std::pair<double, GridPoint> cellCost;

struct cellCostWrapper
{   
    cellCost obj;
    cellCostWrapper(const cellCost& obj_) : obj(obj_){};
    bool operator==(const cellCostWrapper& obj1)const{ if(obj.first==obj1.obj.first){ return true;} else{ return false;}}
    bool operator<(const cellCostWrapper& obj1)const{ if(obj.first<obj1.obj.first){ return true;} else{ return false;}}
    bool operator>(const cellCostWrapper& obj1)const{ if(obj.first>obj1.obj.first){ return true;} else{ return false;}}
};

struct GridCell 
{
    GridPoint parent;
    double f, g, h;
};

struct SquareBasic{
    GridPoint centre;
    int length;
    SquareBasic(){}
    SquareBasic(const GridPoint& centre_, int& length_) : centre(centre_), length(length_){}
    bool operator==(const SquareBasic& square)const{return(square.centre==centre && square.length==length);}
};

struct Square{
    GridPoint centre, obstacle;   // t_cost : transition cost from start/prev node to current
    int length;         // g_cost : distance to goal cost,  o_cost : distance to obstacle cost
    double t_cost=0, g_cost=FLT_MAX, o_cost=0, s_cost=100, cost, i_cost=0;   // s_cost : size of expanded square cost,  cost : total cost
    bool operator>(const Square& square)const{ return cost > square.cost;}
    bool operator<(const Square& square)const{ return cost < square.cost;}
    bool operator==(const Square& square)const{ return cost == square.cost;}
};

struct Circle{
    GridPoint centre;
    float radius;
};

struct Quad{
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};

struct GlobalMap{
    Eigen::MatrixXi grid;
    float resolution;
    uint32_t width;
    uint32_t height;
    geometry_msgs::Pose origin;
    Eigen::Vector2d goal;
};

struct LocalMap{
    Eigen::MatrixXi grid;
    GridPoint start={0,0};
    GridPoint goal={0,0};
    GridPoint origin;
    uint32_t width;
    uint32_t height;
    float resolution;
    geometry_msgs::Pose global_origin;
};

#endif