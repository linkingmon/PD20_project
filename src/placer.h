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
    // void cal_congestion_force();
    // void cal_wirelen_force();
    // void cal_extrademand_force();
    vector<vector<int> > _supply;
    vector<vector<int> > _demand;
    vector<vector<int> > _congestion;

    double cal_bd_congestion(Net* cur_net);
    void bd_congestion_init(double dw);


    // retrace solution
    vector<Cell> _best_solution;
    double _best_cost;
    double _best_wire;
    double _best_congestion;
    void keep_best_cost(double& cur_cost, double& congestion, double& wire);
    void keep_best_solution();
    void recover_best_solution();

    // SA related
    int _step_cnt;
    int _iteration_cnt;
    int _reject_num;
    int _move_num;
    int _uphill_num;
    double SA_iteration(double &cur_temperature);
    void perturb();
    void deperturb();

    // initial SA related
    int N;
    int _norm_num = 100;
    double _congestion_norm_factor;
    double _wire_length_norm_factor;
    double random_place(int steps);
    void calculate_norm_factor();

    // cost function related
    int _feasible_chain_size = 100;
    vector<bool> _feasible_chain;
    double _beta; // history cost
    int _feasible_count; // history cost
    void set_feasible(bool feasible);
    int cal_net_cost(Net* cur_net);
    void calculate_congestion_cost(double& congestion);
    void calculate_wire_length_cost(double& wire);
    double calculate_total_cost(double& congestion, double& wire);
    double calculate_total_cost();

    // SA partial recovery
    int _perturb_type;
    int _perturb_val1;
    int _perturb_val2;
    int _temp_x;
    int _temp_y;

    // Clean up Placer
    void clear();
    double rand_01() {return double(rand() % 100000) * double(1e-5);};
};

#endif // Placer_H
