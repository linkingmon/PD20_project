#ifndef ROUTER_H
#define ROUTER_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
#include "struc.h"
#include "placement.h"
using namespace std;

class coordinate{
public:
    coordinate( int x , int y ,int z):_x(x),_y(y),_z(z){}
    
    int _x ;
    int _y ; 
    int _z ;
    coordinate* next_bend;
};

class two_pin_net
{
public:
    two_pin_net(Pin* a , Pin* b) : p_source(a),p_target(b){}
    coordinate* get_source() { return source;};

private:
    Pin* p_source;
    Pin* p_target;
    coordinate* source;

};


class Router
{
public:
    // constructor and destructor
    Router(Placement * placement) : _placement(placement)
    {
    }
    ~Router()
    {
        clear();
    }
    void two_pin_net_L_routing(Pin* , Pin* );
    void two_pin_net_Z_routing(Pin* , Pin* );

    void route() {cout << "Routing ..." << endl;}

private:
    Placement * _placement;
    vector<two_pin_net> twopin_netlist;
    // Clean up Router
    void clear();
};

#endif // Router_H
