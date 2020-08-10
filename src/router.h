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
        demand_grid_map = new Congestion( width,height,layer );
        
        layer = 1;
        supply_row_map = new Congestion_Row( width,height,layer );
        demand_row_map = new Congestion_Row( width,height,layer );
        cost_row_map =   new Congestion_Row( width,height,layer );
        supply_col_map = new Congestion_Col( width,height,layer );
        demand_col_map = new Congestion_Col( width,height,layer );
        cost_col_map =   new Congestion_Col( width,height,layer );

        supply_grid_2Dmap = new Congestion( width,height,layer );
        demand_grid_2Dmap = new Congestion( width,height,layer );

       
    }
    ~Router()
    {
        clear();
    }
    void two_pin_net_L_routing(Pin* , Pin* );           // perform L routing in 2D plane with two pin
    void two_pin_net_Z_routing(Pin* , Pin* );           // perform Z routing in 2D plane with two pin
    void two_pin_net_Both_L_routing(two_pin_net*,bool);
    void two_pin_net_L_routing(two_pin_net*);
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
    void Add_demand_3D(Bend*,double);
    void Add_demand_3D_H(int,int,int,int,double f);
    void Add_demand_3D_V(int,int,int,int,double f);
    void Add_demand_3D_Z(int,int,int,int,double f);
    void Exclude_demand( Bend* , double );
    void Exclude_demand_H(int,int,int,int,double);
    void Exclude_demand_V(int,int,int,int,double);
    void projection_to_2D();
    void supply_from_grid_to_edge();
    
    void compute_total_net_length();
    void compute_one_net_length(int);
    void sort_net();
    void update_distance_of_branch();
    void update_distance_of_branch_in_one_net(int);

    void layer_assignment_straight_line(Bend*, Bend*,int, int);
    int  layer_assignment_straight_line_old_method(Bend* , Bend*, int);
    int  layer_assignment_straight_line_V(Bend*,Bend*,int,double);
    int  layer_assignment_straight_line_H(Bend*,Bend*,int,double);
    void layer_assignment_one_net(int);
    void layer_assignment_two_pin_net(int,two_pin_net);
    void layer_assignment_of_pin(int);
    void layer_assignment_layer_range(int);
    void z_dirertion_layer_assignment(branch*,branch*);
    void update_cost_map();
    
    
    // main function
    void route() ;
    void construct_congestion_map();
    void layer_assignment();

    //ouput
    int write_result_to_cmd();
    int write_result(int,int&);
    void writeResult(fstream& );
    void writeResult(int,fstream&);

    //check function
    bool check_demand();
    
private:
    Placement * _placement;
    vector<two_pin_net*> twopin_netlist_L;
    vector<two_pin_net*> twopin_netlist_Z;
    Congestion_Row* supply_row_map,*demand_row_map,*cost_row_map;            //congestion of row
    Congestion_Col* supply_col_map,*demand_col_map,*cost_col_map;            //congestion of column
    Congestion* supply_grid_map, * supply_grid_2Dmap;       //supply of total grid
    Congestion* demand_grid_map, * demand_grid_2Dmap;       //demand of total grid
    Two_Dimension_map< pair<int,int>>   layer_range;        //the layer range of total 2D grid of one net
    vector<pair<int,int>>               LR_index;           //the modified layer range index
    vector<vector<Grid*>> grid_map;     //grid map include cell information 
    vector<double> x_expand_factor;     //expansion factor of x
    vector<double> x_expand_result;     //expansion result of x
    vector<double> y_expand_factor;     //expansion factor of y
    vector<double> y_expand_result;     //expansion result of y
    vector<vector<two_pin_net>> two_pin_netlist;
    vector<vector<branch*>> branch_of_netlist;
    vector<vector<Segment>> segment_of_netlist;
    int fail_segment;
    // Clean up Router
    void clear();
};

#endif // Router_H
