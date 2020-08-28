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
class Maze_router;
 
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
    friend ostream& operator<<(ostream& os, const Coordinate& a) { os<<"("<<a.x<<", "<<a.y<<") "<< " idx: " << a.index << " d is "<<(a.distance ) <<" estimate_d is "<<( a.estimate_distance)<<" bend: "<<a.bend; return os;}
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
    friend Maze_router;
public:
    Shortest_Path():num_vertex(0){ InitializeSingleSource();   }
    Shortest_Path(int s_x , int s_y , int s_z , int t_x , int t_y , int t_z , double a_factor, int left, int o_width, int bottom, int o_height,
        Congestion_Row* r_map , Congestion_Col* c_map , Congestion* g_map )
        :source_x(s_x), source_y(s_y) , source_z(s_z) , target_x(t_x) , target_y(t_y) , target_z(t_z),
    expand_factor(a_factor), left_bound(left), width(o_width), bottom_bound(bottom), height(o_height), row_map(r_map), col_map(c_map), grid_map(g_map)
    {
        // InitializeSingleSource(); 
        InitializeMultiSource();  
    }
    void PrintDataArray(vector<int> array);
    void PrintIntArray(int *array);


    void InitializeSingleSource();      // initialize
    void InitializeMultiSource();       // multi source intialize
    void Setup_multi_source(vector<pair<int,int>>,MinHeap_pointer<Coordinate>& );
    void Setup_multi_end(vector<pair<int,int>>);
    void Set_boundary();                // set boundary of seraching region
    void Construct_Edge();
    void Construct_H_Edge();
    void Construct_V_Edge();
    void Dijkstra();         
    void Dijkstra_multi();                                   // 需要Min-Priority Queue
    bool Relax( Coordinate* , Coordinate* , double weight );    //dijkstra function
    bool is_start_point( Coordinate* );                         //check starting point
    bool is_end_point( Coordinate* );                           //check ending point
    void AddEdge(int from, int to, int weight);
    void Build_the_path();
    void Build_the_path_multi();
    void Build_the_path_old();
    void reverse_path();
    void set_start(vector<pair<int,int>> a) { source_vector = a;}
    void set_end(vector<pair<int,int>> a) { end_vector = a;}

    
    Coordinate* End_point();
    Bend* target() { return target_bend;}
    Bend* source() { return source_bend;} 
    int Estimate_distance_to_target(int,int);       // function of estimate distance to target
private:
    int num_vertex;
    int source_x, source_y, source_z;
    int final_sx, final_sy;
    int final_tx, final_ty;
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
    vector<vector<bool>> set_of_start_point;
    vector<vector<bool>> set_of_end_point;
    vector<pair<int,int>> source_vector;
    vector<pair<int,int>> end_vector;

};

class Maze_router{
public:
    Maze_router(){}
    Maze_router(int s_x , int s_y , int s_z , int t_x , int t_y , int t_z, double a_factor, int left, int o_width, int bottom, int o_height, 
    Congestion_Row* r_map , Congestion_Col* c_map , Congestion* g_map, SubTree* a, SubTree* b)
        :SP( s_x, s_y, s_z, t_x, t_y, t_z, a_factor, left, o_width, bottom, o_height, r_map, c_map, g_map ), width(o_width), height(o_height),
        left_bound(left), bottom_bound(bottom), start_x(s_x), start_y(s_y), end_x(t_x), end_y(t_y), start_tree(a), end_tree(b){
            Initialize();
        }
    void SP_setup();
    void Initialize();
    void Initialize_start();
    void Initialize_end();
    void Connect_two_tree();
    SubTree* Connect_two_tree(SubTree*,SubTree*);
    void adjust_tree(SubTree*,int,int,branch*);
    SubTree* get_final_tree() {return final_tree;}
    Shortest_Path SP;
private:
    vector<pair<int,int>> start_point; 
    vector<pair<int,int>> end_point;
    vector<vector<int>>   two_pin_net_id;
    vector<vector<branch*>>  branch_map;
    SubTree* start_tree;
    SubTree* end_tree;
    SubTree* final_tree;
    branch* b_start, *b_end;
    two_pin_net* new_net;
    int width;
    int height;
    int left_bound, bottom_bound;
    int start_x, start_y;
    int end_x, end_y;
};





#endif  //MAZE_ROUTER_H