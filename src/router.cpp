#include "router.h"
#include "net.h"
#include "struc.h"

using namespace std;

void Router::two_pin_net_L_routing(Pin* a  , Pin* b){
    
    Cell* c_a = _placement->getCell(a->getcellId());
    Cell* c_b = _placement->getCell(b->getcellId());
    int a_x = c_a->getx();
    int a_y = c_a->gety();
    int b_x = c_b->getx();
    int b_y = c_b->gety();
    two_pin_net* L_net = new two_pin_net(a,b);
}

void Router::two_pin_net_Z_routing(Pin*a , Pin*b){
    
}

