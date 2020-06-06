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
    Placer(Placement * placement);
    ~Placer()
    {
        clear();
    }
    void place();

private:
    Placement * _placement;
    void cal_congestion_force();
    void cal_wirelen_force();
    void cal_extrademand_force();
    int cal_net_cost(Net* cur_net);
    vector<vector<int> > _supply;
    vector<vector<int> > _demand;
    vector<vector<int> > _congestion;

    double cal_bd_congestion(Net* cur_net);
    void bd_congestion_init(double dw);

    // Clean up Placer
    void clear();
};

#endif // Placer_H
