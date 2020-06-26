#ifndef MAZE_ROUTER_H
#define MAZE_ROUTER_H

#include <iostream>
#include <list>
#include <utility>
#include <queue>
#include <vector>
#include <functional>
#include "congestion.h"
#include "MinHeap.h"
#include "struc.h"

// #define Max_Distance = 100 ;
using namespace std;
enum SP_direction{
    p_x,
    n_x,
    p_y,
    n_y,
};

class Coordinate{
public:
    Coordinate(){}
    Coordinate(int i_x,int i_y, int i_z, double d):x(i_x),y(i_y),z(i_z),distance(d),bend(10e4){
        H_bend = false;
        V_bend = false;
    }
    void update_position( int idx ) { index = idx;}
    int x;
    int y;
    int z;
    int bend;       //the total bend made when touch the point
    bool H_bend;    //the min_distance and min total bend can be acheive by the last line is horizotal
    bool V_bend;    //the min_distance and min total bend can be acheive by the last line is horizotal
    // int id(int width , int height) { return x + y * width + z * width * height ;}
    double distance;
    double estimate_distance;
    int index;      // store the index in min_heap
    Coordinate* H_predecessor;
    Coordinate* V_predecessor;
    Coordinate* predecessor;
    bool operator > (const Coordinate& b){return (distance + estimate_distance > b.distance + b.estimate_distance);}
    bool operator < (const Coordinate& b){return (distance + estimate_distance < b.distance + b.estimate_distance);}
    bool operator == (const Coordinate& b){return (x == b.x && y == b.y);}
    friend ostream& operator<<(ostream& os, const Coordinate& a) { os<<"("<<a.x<<", "<<a.y<<") "<< " idx: " << a.index << " d is "<<(a.distance + a.estimate_distance)<<" bend: "<<a.bend; return os;}
};

class Coordinate_Compartor{
public:
    Coordinate_Compartor(){}
    bool operator() (const Coordinate& a, const Coordinate& b) const
    {
        return(a.distance > b.distance);
    }
};


class Shortest_Path     //2D
{
public:
    Shortest_Path():num_vertex(0){ InitializeSingleSource();   }
    Shortest_Path(int s_x , int s_y , int s_z , int t_x , int t_y , int t_z , double a_factor, 
        Congestion_Row* r_map , Congestion_Col* c_map , Congestion* g_map )
        :source_x(s_x), source_y(s_y) , source_z(s_z) , target_x(t_x) , target_y(t_y) , target_z(t_z),
    expand_factor(a_factor), row_map(r_map), col_map(c_map), grid_map(g_map)
    {
        InitializeSingleSource(); 
    }
    void PrintDataArray(vector<int> array);
    void PrintIntArray(int *array);


    void InitializeSingleSource();      // initialize
    void Set_boundary();                // set boundary of seraching region
    void Construct_Edge();
    void Construct_H_Edge();
    void Construct_V_Edge();
    void Dijkstra();                                            // 需要Min-Priority Queue
    bool Relax( Coordinate* , Coordinate* , double weight );    //dijkstra function
    void AddEdge(int from, int to, int weight);
    void Build_the_path();
    void Build_the_path_old();
    void reverse_path();
    Coordinate* End_point();
    Bend* target() { return target_bend;}
    Bend* source() { return source_bend;} 
    int Estimate_distance_to_target(int,int);       // function of estimate distance to target
private:
    int num_vertex;
    int source_x, source_y, source_z;
    int target_x, target_y, target_z;
    int left_bound,right_bound, bottom_bound,top_bound;
    int width, height;
    double expand_factor;
    Congestion_Row* row_map;
    Congestion_Col* col_map;
    Congestion*     grid_map;
    Bend*           target_bend;
    Bend*           source_bend;
    vector<vector<list<pair<SP_direction,double> > > > AdjList;
    vector<SP_direction> predecessor;
    vector<Coordinate> grid_point_Min_Heap;
    vector<vector<Coordinate*>> grid_point_pointer;
    vector<vector<bool>> visited;
    vector<vector<bool>> inside_min_heap;
};





#endif  //MAZE_ROUTER_H