#ifndef ROUTER_H
#define ROUTER_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
#include "placement.h"
using namespace std;

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
    void route() {cout << "Routing ..." << endl;}

private:
    Placement * _placement;
    // Clean up Router
    void clear();
};

#endif // Router_H
