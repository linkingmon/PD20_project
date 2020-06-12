#include "router.h"
#include "net.h"
#include "struc.h"
#include "congestion.h"

using namespace std;

void Router::two_pin_net_L_routing(Pin* a  , Pin* b){
    
    Cell* c_a = _placement->getCell(a->getcellId());
    Cell* c_b = _placement->getCell(b->getcellId());
    int a_x = c_a->getx();
    int a_y = c_a->gety();
    int a_z = a->get_layer();
    int b_x = c_b->getx();
    int b_y = c_b->gety();
    int b_z = b->get_layer();
    two_pin_net* L_net = new two_pin_net(a,b);
    L_net->set_source ( new coordinate(a_x,a_y,a_z) );
    
}

void Router::two_pin_net_Z_routing(Pin*a , Pin*b){
    
}

void Router::H_route(size_t x , size_t y1, size_t y2 , size_t z ){
       
}