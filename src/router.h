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
using namespace std;

class coordinate{
public:
    coordinate() {}
    coordinate( int x , int y ,int z , coordinate* p = NULL , coordinate* n = NULL):_x(x),_y(y),_z(z),prev_bend(p),next_bend(n){}
    void set_prev( coordinate* p ) { prev_bend = p ;}
    void set_next( coordinate* n ) { next_bend = n ;}
    coordinate* get_prev() const { return prev_bend;}
    coordinate* get_next() const { return next_bend;}
    void print() { cout<<"x is: "<<setw(5)<< _x <<" y is: "<<setw(5)<< _y <<setw(5)<<" z is "<<setw(5)<< _z <<endl;}
    int _x ;
    int _y ; 
    int _z ;
    coordinate* prev_bend;
    coordinate* next_bend;
};

class two_pin_net
{
public:
    two_pin_net(Pin* a , Pin* b) : p_source(a),p_target(b){}
    coordinate* get_source() { return source;};
    void set_source( coordinate* root ) { source = root;}
    void print_bend() { 
        coordinate* temp = source ; 
        while(temp != NULL){
            temp->print();
            temp = temp->get_next();
        }
    }
private:
    Pin* p_source;
    Pin* p_target;
    coordinate* source; //staring point

};
//to do list
//2D L routing ( only two path ) done 
//2D Z routing ( BFS serach ) done 
//congestion map visualization
//3D Z routing ( only six path )
//congestion supply and demand map construction
class Router
{
public:
    // constructor and destructor
    Router(Placement * placement) : _placement(placement)
    {
        int width = placement->_boundary_width;
        int height = placement->_boundary_height;
        int layer = placement->_numLayers;
        
        int defult_supply = 0;
        // supply_grid_map = new Congestion( width,height,layer );
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
    coordinate* L_route_2D(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z );     // perform L routing in 2D plane
    coordinate* Z_routing(size_t x1, size_t x2, size_t y1 ,size_t y2 ,size_t z);        // perform Z routing in 2D plane
    double Z_route_2D_H(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , coordinate*& Z_bend );  //2D plane Z routing with two Horizontal and one vertical route
    double Z_route_2D_V(size_t x1, size_t x2, size_t y1, size_t y2 , size_t z , coordinate*& Z_bend );  //2D plane Z routing with one Horizontal and two vertical route

    double H_route(size_t x , size_t y1, size_t y2 , size_t z );    // straight route in Horizontal ( Row )
    double V_route(size_t x1 , size_t x2, size_t y , size_t z );    // straight route in Vertical ( Col )
    void construct_supply_demand_map();
    void route() {
        cout << "Routing ..." << endl;
        (*row_map)(2,3,1) = 3;
        // row_map->cong_map_3D[2][3][1] = 0;
        cout<<(*row_map)(2,3,1)<<endl;
        // cout<<row_map->block_to_edge_p(2,3,1);

        // cout<<row_map->block_to_edge_n(2,3,1);
        row_map->print_congestion();
        col_map->print_congestion();
        
        construct_supply_demand_map();
        supply_grid_map->print_congestion();
        // coordinate* bug_test = Z_routing(3,1,3,1,0);
        // // coordinate* bug_L_test = L_route_2D(1,3,1,3,0);
        // coordinate* temp = bug_test;
        // while (temp != NULL){
        //     temp->print(); 
        //     temp = temp->get_next();
        // }
        // bug_L_test->print();
        }

private:
    Placement * _placement;
    vector<two_pin_net*> twopin_netlist_L;
    vector<two_pin_net*> twopin_netlist_Z;
    Congestion_Row* row_map;
    Congestion_Col* col_map;
    Congestion* supply_grid_map;
    Congestion* demand_grid_map;
    // Clean up Router
    void clear();
};

#endif // Router_H
