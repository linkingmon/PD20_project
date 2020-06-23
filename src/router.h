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
extern "C" {
    #include "../flute/flute.h"
    // #include "flute.h"
    void    readLUT();
}

using namespace std;
template<class data>
class branch{
public:
    branch() {}
    branch(data x, data y, data z ,data n = 0):_x(x),_y(y),_z(z),_n(n){}
    data _x;     //row index
    data _y;     //column index
    data _z;     //layer index
    data _n;     //neighbor
};

template<class data>
class two_pin_net
{
    friend class Router;
public:
    two_pin_net(branch<data>* a, branch<data>* b) : b_source(a), b_target(b){}
    two_pin_net() {}
    ~two_pin_net(){}
    Bend* get_source() { return source;};
    void set_source( Bend* root ) { source = root;}
    void print_bend() { 
        Bend* temp = source ; 
        while(temp != NULL){
            temp->print();
            temp = temp->get_next();
        }
    }
private:
    branch<data>* b_source;
    branch<data>* b_target;
    Bend* source; //staring point

};

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
        supply_row_map = new Congestion_Row( width,height,layer );
        demand_row_map = new Congestion_Row( width,height,layer );
        cost_row_map = new Congestion_Row( width,height,layer );
        supply_col_map = new Congestion_Col( width,height,layer );
        demand_col_map = new Congestion_Col( width,height,layer );
        cost_col_map = new Congestion_Col( width,height,layer );


    }
    ~Router()
    {
        clear();
    }
    void two_pin_net_L_routing(Pin* , Pin* );           // perform L routing in 2D plane with two pin
    void two_pin_net_Z_routing(Pin* , Pin* );           // perform Z routing in 2D plane with two pin
    void two_pin_net_Both_L_routing(two_pin_net<int>*,bool);
    void two_pin_net_L_routing(two_pin_net<int>*);
    Bend* L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z );     // perform L routing in 2D plane
    Bend* Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z);        // perform Z routing in 2D plane
    double Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , Bend*& Z_bend );  //2D plane Z routing with two Horizontal and one vertical route
    double Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , Bend*& Z_bend );  //2D plane Z routing with one Horizontal and two vertical route

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
    Tree Flute_function(vector<double> , vector<double> );
    void expansion();
    void update_x_expansion(int , double);
    void update_y_expansion(int , double);
    int  find_expand_x_position(double);
    int  find_expand_y_position(double);
    void construct_two_pin_net( Net* , int );
    void construct_two_pin_net_with_expansion(Net*, int);
    void construct_total_two_pin_net(bool);
    void Add_demand( Bend* , double );
    void Add_demand_H(int,int,int,int,double);
    void Add_demand_V(int,int,int,int,double);
    void Exclude_demand( Bend* , double );
    void Exclude_demand_H(int,int,int,int,double);
    void Exclude_demand_V(int,int,int,int,double);
    // main function
    void route() ;
    void construct_congestion_map();
private:
    Placement * _placement;
    vector<two_pin_net<int>*> twopin_netlist_L;
    vector<two_pin_net<int>*> twopin_netlist_Z;
    Congestion_Row* supply_row_map,*demand_row_map,*cost_row_map;            //congestion of row
    Congestion_Col* supply_col_map,*demand_col_map,*cost_col_map;            //congestion of column
    Congestion* supply_grid_map;        //supply of total grid
    Congestion* demand_grid_map;        //demand of total grid
    vector<vector<Grid*>> grid_map;     //grid map include cell information 
    vector<double> x_expand_factor;     //expansion factor of x
    vector<double> x_expand_result;     //expansion result of x
    vector<double> y_expand_factor;     //expansion factor of y
    vector<double> y_expand_result;     //expansion result of y
    vector<vector<two_pin_net<int>>> two_pin_netlist;
    
    // Clean up Router
    void clear();
};

#endif // Router_H
