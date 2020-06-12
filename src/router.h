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
    coordinate( int x , int y ,int z):_x(x),_y(y),_z(z){}
    void set_prev( coordinate* p ) { prev_bend = p ;}
    void set_next( coordinate* n ) { next_bend = n ;}
    coordinate* get_prev() const { return prev_bend;}
    coordinate* get_next() const { return next_bend;}
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
private:
    Pin* p_source;
    Pin* p_target;
    coordinate* source;

};
//to do list
//2D L routing ( only two path )
//2D Z routing ( BFS serach )
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
        row_map = new Congestion_Row( width,height,layer);
        col_map = new Congestion_Col( width,height,layer);
    }
    ~Router()
    {
        clear();
    }
    void two_pin_net_L_routing(Pin* , Pin* );
    void two_pin_net_Z_routing(Pin* , Pin* );
    
    void H_route(size_t x , size_t y1, size_t y2 , size_t z );

    void route() {
        cout << "Routing ..." << endl;
        (*row_map)(2,3,1) = 3;
        // row_map->cong_map_3D[2][3][1] = 0;
        cout<<(*row_map)(2,3,1)<<endl;
        // cout<<row_map->block_to_edge_p(2,3,1);
        
        // cout<<row_map->block_to_edge_n(2,3,1);
        row_map->print_congestion();
        col_map->print_congestion();
        }

private:
    Placement * _placement;
    vector<two_pin_net> twopin_netlist;
    Congestion_Row* row_map;
    Congestion_Col* col_map;
    // Clean up Router
    void clear();
};

#endif // Router_H
