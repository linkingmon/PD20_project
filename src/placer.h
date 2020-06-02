#ifndef PLACER_H
#define PLACER_H

#include <fstream>
#include <vector>
#include <map>
#include "cell.h"
#include "net.h"
#include "placement.h"
using namespace std;

class Placer
{
public:
    // constructor and destructor
    Placer(Placement * placement) : _placement(placement)
    {
    }
    ~Placer()
    {
        clear();
    }
    void place() {cout << "Placing ..." << endl;}

private:
    Placement * _placement;
    // Clean up Placer
    void clear();
};

#endif // Placer_H
