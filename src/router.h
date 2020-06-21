#ifndef ROUTER_H
#define ROUTER_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
#include "struc.h"
#include "placement.h"
#include "congestion.h"
#include "maze_router.h"
using namespace std;

class bend{
public:
    bend() {}
    bend( int x , int y ,int z , bend* p = NULL , bend* n = NULL):_x(x),_y(y),_z(z),prev_bend(p),next_bend(n){}
    void set_prev( bend* p ) { prev_bend = p ;}
    void set_next( bend* n ) { next_bend = n ;}
    bend* get_prev() const { return prev_bend;}
    bend* get_next() const { return next_bend;}
    void print() { cout<<"x is: "<<setw(5)<< _x <<" y is: "<<setw(5)<< _y <<setw(5)<<" z is "<<setw(5)<< _z <<endl;}
    int _x ;
    int _y ; 
    int _z ;
    bend* prev_bend;
    bend* next_bend;
};

class branch{
public:
    branch() {}
    branch(int x, int y, int z ,int n):_x(x),_y(y),_z(z),_n(n){}
    int _x;     //row index
    int _y;     //column index
    int _z;     //layer index
    int _n;     //neighbor
};

class two_pin_net
{
public:
    two_pin_net(Pin* a , Pin* b) : p_source(a),p_target(b){}
    bend* get_source() { return source;};
    void set_source( bend* root ) { source = root;}
    void print_bend() { 
        bend* temp = source ; 
        while(temp != NULL){
            temp->print();
            temp = temp->get_next();
        }
    }
private:
    Pin* p_source;
    Pin* p_target;
    bend* source; //staring point

};
//to do list
//2D L routing ( only two path ) done 
//2D Z routing ( BFS serach ) done 
//congestion map visualization
//3D Z routing ( only six path )
//congestion supply and demand map construction

class Grid
{
public:
    Grid() {}
    Grid(int x , int y, int num_master):_x(x),_y(y){
        // master_list.resize(num_master);
    }
    ~Grid(){}

    int getx()  {return _x;}
    int gety()  {return _y;}
    void add_Cell( int cell_id)     { cell_list.push_back(cell_id) ;}
    void add_MCell( int MCell_id ) { 
        map<int,int>::iterator iter = master_list.find(MCell_id);
        if(iter == master_list.end()){
            master_list.insert(pair<int,int>(MCell_id,1) );
        }   
        else 
            ++(iter->second);
    }
    map<int,int> get_MCell_list() const {return master_list; }
    // int& getMCell( int MCell_id) { return master_list[MCell_id] ;}
private:
    int   _x ;      //normalized x
    int   _y ;      //normalized y
    vector<int> cell_list;      //cell index list
    int cell_num;               //number of cells
    // vector<int> master_list;    //the number of each matser cell
    map<int,int> master_list;   //map from master cell index to its number 
};

class Router
{
public:
    // constructor and destructor
    Router(Placement * placement) : _placement(placement)
    {
        int width = placement->_boundary_width;
        int height = placement->_boundary_height;
        int layer = placement->_numLayers;
        
        int default_supply = 0;
        
        grid_map = vector<vector<Grid*>>( width, vector<Grid*>(height) );

        supply_grid_map = new Congestion( width,height,layer );
        // supply_grid_map = new Congestion( 1,1,1 );
        demand_grid_map = new Congestion( width,height,layer );
        
        layer = 1;
        row_map = new Congestion_Row( width,height,layer );
        col_map = new Congestion_Col( width,height,layer );

    }
    ~Router()
    {
        clear();
    }
    void two_pin_net_L_routing(Pin* , Pin* );           // perform L routing in 2D plane with two pin
    void two_pin_net_Z_routing(Pin* , Pin* );           // perform Z routing in 2D plane with two pin
    bend* L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z );     // perform L routing in 2D plane
    bend* Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z);        // perform Z routing in 2D plane
    double Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , bend*& Z_bend );  //2D plane Z routing with two Horizontal and one vertical route
    double Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , bend*& Z_bend );  //2D plane Z routing with one Horizontal and two vertical route

    double V_route(size_t x , size_t y1, size_t y2 , size_t z );    // straight route in Vertical ( cross Row )
    double H_route(size_t x1 , size_t x2, size_t y , size_t z );    // straight route in Horizontal ( cross Col )
    
    double V_route_edge(size_t x , size_t y1, size_t y2 , size_t z );    // straight route in Vertical ( Row ) computed cost by edge
    double V_route_grid(size_t x , size_t y1, size_t y2 , size_t z );    // straight route in Vertical ( Row ) computed cost by grid
    
    double H_route_edge(size_t x1 , size_t x2, size_t y , size_t z );    // straight route in Horizontal ( Col ) computed cost by edge
    double H_route_grid(size_t x1 , size_t x2, size_t y , size_t z );    // straight route in Horizontal ( Col ) computed cost by grid

    void construct_supply_demand_map();
    void construct_grid_map();
    void maze_routing();
    void A_star_search_routing();
    void route() ;
    // {
    //     cout << "Routing ..." << endl;
    //     cout<<(*row_map)(2,3,1)<<endl;

    //     row_map->print_congestion();
    //     col_map->print_congestion();
        
    //     construct_grid_map();
    //     construct_supply_demand_map();
    //     supply_grid_map->print_congestion();
    //     demand_grid_map->print_congestion();
    //     }

private:
    Placement * _placement;
    vector<two_pin_net*> twopin_netlist_L;
    vector<two_pin_net*> twopin_netlist_Z;
    Congestion_Row* row_map;            //congestion of row
    Congestion_Col* col_map;            //congestion of column
    Congestion* supply_grid_map;        //supply of total grid
    Congestion* demand_grid_map;        //demand of total grid
    vector<vector<Grid*>> grid_map;      //grid map include cell information 
    // Clean up Router
    void clear();
};

#endif // Router_H
