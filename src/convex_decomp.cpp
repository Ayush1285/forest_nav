#include "convex_decomp.h"

std::unique_ptr<pair<int, GridPoint>> spiralSearch(const GridPoint& centre, const Eigen::MatrixXi& map)
{
    
    int step_size = 1;
    GridPoint current_node = centre;
    std::unique_ptr<pair<int, GridPoint>> square_return_ptr = std::make_unique<pair<int, GridPoint>>();
    while (centre.x - current_node.x < 30 && centre.x - current_node.x > -30)
    {
        //cout << " start - current = " << start.y - current_node.y << endl;
        //if(step_size%2==1)
        //{
            for(int i = 0; i < step_size; i++)
            {
                current_node.x = current_node.x + 1;
                if(map(current_node.y,current_node.x)==100)
                {
                    //cout << "Into rightward motion, node value = " << current_node.y <<" "<<current_node.x<< "  i = " <<i<<endl;
                    *square_return_ptr = std::make_pair(2*(centre.y-current_node.y-1), GridPoint(current_node.y, current_node.x)); 
                    return  square_return_ptr;

                }
            }
            for(int i = 0; i < step_size; i++)
            {
                current_node.y = current_node.y + 1;
                if(map(current_node.y,current_node.x)==100)
                {
                    //cout << "Into upward motion, node value = " << current_node.y <<" "<<current_node.x<< "  i = " <<i<<endl;
                    *square_return_ptr = std::make_pair(2*(centre.y-current_node.y+1+i), GridPoint(current_node.y, current_node.x));
                    return  square_return_ptr ;
                }
            }
            step_size += 1;        
        //}
        //else
        //{
            for(int i = 0; i < step_size; i++)
            {
                current_node.x = current_node.x - 1;
                if(map(current_node.y,current_node.x)==100)
                {
                    //cout << "Into leftward motion, node value = " << current_node.y <<" "<<current_node.x<< "  i = " <<i<<endl;
                    *square_return_ptr = std::make_pair(2*(centre.y-current_node.y+1), GridPoint(current_node.y, current_node.x)) ;
                    return  square_return_ptr;
                }
            }
            for(int i = 0; i < step_size; i++)
            {   
                current_node.y = current_node.y - 1;
                if(map(current_node.y,current_node.x)==100)
                {
                    //cout << "Into downward motion, node value = " << current_node.y <<" "<<current_node.x<< "  i = " <<i<<endl;
                    *square_return_ptr = std::make_pair(2*(current_node.y+i-centre.y)+1, GridPoint(current_node.y, current_node.x)) ;
                    return square_return_ptr ;
                }
            }
            step_size += 1;        
        //}
    }
    *square_return_ptr = std::make_pair(2*(centre.y-current_node.y), GridPoint(-1, -1));
    return square_return_ptr;
}

inline Square genSquare(const GridPoint& centre, const GridPoint& parent, const GridPoint& goal)
{
    Square square;
    //std::unique_ptr<pair<int, GridPoint>> get_ptr = spiralSearch(centre, map);
    //square.length = get_ptr->first;
    //square.obstacle = get_ptr->second;
    square.centre = centre;
    square.g_cost = calNorm(centre, goal);
    square.t_cost = calNorm(centre, parent);
    //square.s_cost = cal_scost(square.length);
    //square.cost = square.g_cost + square.t_cost + square.o_cost;
    return square;
}

bool updateObstacleCost(const vector<GridPoint>& obs_list, Square& square)
{
    for(int j = 0 ; j < obs_list.size(); j++)
    {
        /*if(obs_list[j].x == -1 && obs_list[j].y == -1)
        {
            double o_cost = 0;
            if(o_cost >square.o_cost){square.o_cost = o_cost;}
        }
        else
        {*/
            double dist_to_obs = calNorm(square.centre, obs_list[j]);
            if(dist_to_obs < 5){ return false;}
            //double o_cost = cal_ocost(dist_to_obs);
            //if(o_cost >square.o_cost){square.o_cost = o_cost;}
        //}
    }
    return true; 
}
void infeasibilityCost(const vector<GridPoint>& infeasible_nodes, Square& square)
{
    for(int j = 0 ; j < infeasible_nodes.size(); j++)
    {
       
        double dist_to_infeasible_node = calNorm(square.centre, infeasible_nodes[j]);
        double i_cost = ConvexCorridor::cal_icost(dist_to_infeasible_node);
        if(i_cost >square.i_cost){square.i_cost = i_cost;}
    }
}

std::vector<SquareBasic> traceConvexPath
(std::unordered_map<GridPoint, SquareBasic>& stored_squares, std::unique_ptr<SquareBasic> curr_square_ptr, const GridPoint& start)
{
    std::vector<SquareBasic> squares;
    SquareBasic curr_square = *curr_square_ptr;
    squares.push_back(curr_square);
    while(curr_square.centre!=start)
    {
        curr_square = stored_squares[curr_square.centre];
        squares.push_back(curr_square);
    }
    squares.push_back(curr_square);
    return squares;
}

std::vector<SquareBasic> convexPlanner(const GridPoint& start, const GridPoint& goal, const Eigen::MatrixXi& map)
{
    std::set<Square> openlist;
    vector<SquareBasic> squares;
    Square start_square = genSquare(start, start, goal);
    auto start1 = std::chrono::steady_clock::now();
    std::unique_ptr<pair<int, GridPoint>> get_ptr = spiralSearch(start_square.centre, map);
    auto end1 = std::chrono::steady_clock::now();
    //cout << "Elapsed time in spiral search: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
      //  << " micro sec" << endl;
    int64_t duration = std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1).count();
    //int count = 1;
    start_square.length = get_ptr->first;
    start_square.obstacle = get_ptr->second;
    start_square.s_cost = ConvexCorridor::cal_scost(start_square.length);
    start_square.cost = start_square.g_cost + start_square.t_cost + start_square.o_cost + start_square.s_cost;
    openlist.insert(start_square);
    squares.push_back(SquareBasic(start_square.centre, start_square.length));
    std::unordered_map<GridPoint, bool> closedlist;
    std::unordered_map<GridPoint, SquareBasic> store_squares;
    vector<GridPoint> obstacle_list;
    //vector<GridPoint> infeasible_nodes;
    obstacle_list.push_back(start_square.obstacle);
    while(!openlist.empty())
    {
        Square curr_square = *openlist.begin();
        openlist.erase(openlist.begin());
        //cout << "Current Node = " << curr_square.centre.x <<" " << curr_square.centre.y <<" "<< "Len = " 
          //          <<curr_square.length<<" "<<"Cost = "<<curr_square.cost<<" "<<"O_cost = "<<curr_square.o_cost<<" "<<"s_cost = "
            //        <<curr_square.s_cost<<" "<<"g_cost = "<<curr_square.g_cost<< endl;

        closedlist[curr_square.centre] = true;
        std::set<Square> next_squares;
        for(int i = 0; i < 22; i++)
        {
            GridPoint centre = {curr_square.centre.y + std::floor((curr_square.length/2)*std::sin(i*2*pi/22)), 
                                curr_square.centre.x + std::floor((curr_square.length/2)*std::cos(i*2*pi/22))};
            
            if(closedlist[centre]==true){continue;}
            Square square = genSquare(centre, curr_square.centre, goal);
            bool flag = updateObstacleCost(obstacle_list, square);
            //infeasibilityCost(infeasible_nodes, square);
            if(flag==false){ continue;}
            square.cost = square.g_cost + square.t_cost + square.s_cost + curr_square.t_cost; // + square.o_cost;
            next_squares.insert(square);
            //cout << "Next Possible Nodes = " << square.centre.x <<" " << square.centre.y <<" "<<"Cost = "<<square.cost<<" "
              //                  <<"i_cost = "<<square.i_cost <<" "<<"g_cost = "<<square.g_cost<< endl;
        }
        while(!next_squares.empty())
        {
            Square next_square = *next_squares.begin();
            next_squares.erase(next_squares.begin());
            start1 = std::chrono::steady_clock::now();
            get_ptr = spiralSearch(next_square.centre, map);
            end1 = std::chrono::steady_clock::now();
            duration += std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1).count();
            //cout << "Elapsed time in spiral search: " << std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count()
            //<< " micro sec" << endl;
            //count += 1;
            next_square.length = get_ptr->first;
            if(next_square.length/2 > next_square.g_cost)
            { 
                store_squares[next_square.centre] = SquareBasic(curr_square.centre, curr_square.length);
                cout << "Total time elapsed in spiral Searches = " << duration <<" micro sec"<< endl;
                return traceConvexPath(store_squares, std::make_unique<SquareBasic>(next_square.centre, next_square.length), start);
            }
            next_square.obstacle = get_ptr->second;
            obstacle_list.push_back(next_square.obstacle);
             /*if(next_square.length<10)
            {
                infeasible_nodes.push_back(next_square.centre);
            }*/
            if(next_square.length<12)
            {
                continue;
            }
            next_square.s_cost = ConvexCorridor::cal_scost(next_square.length);
            next_square.cost = next_square.cost + next_square.s_cost - 100;
            openlist.insert(next_square);
            //cout << "Node added to OL = " << next_square.centre.x <<" " << next_square.centre.y <<" "<< "Len = " 
              //      <<next_square.length<<" "<<"Cost = "<<next_square.cost<<" "<<"i_cost = "<<next_square.i_cost<<" "<<"s_cost = "
                //    <<next_square.s_cost<<" "<<"g_cost = "<<next_square.g_cost<< endl;
            store_squares[next_square.centre] = SquareBasic(curr_square.centre, curr_square.length);
        }

    }
    return squares;

}