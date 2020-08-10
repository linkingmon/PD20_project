#ifndef PLACER_H
#define PLACER_H

#include <fstream>
#include <vector>
#include <map>
#include <deque>
#include "cell.h"
#include "net.h"
#include "placement.h"
#include "mst.h"
#include "myUsage.h"
#include <set>
#include "utils.h"
#include <cmath>
using namespace std;

extern MyUsage myusage;

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
    void writeResult(fstream &outFile);

private:
    Placement * _placement;

    void init_supply_map();
    void init_demand_map();
    void init_mcell_list();
    void init_fixCell_list();
    vector<vector<vector<double> > > _supply;
    vector<vector<vector<double> > > _demand;
    vector<vector<map<int, int> > > _mcell_list;
    vector<Cell> _init_cellAry;
    vector<int> _fixCell_list;

    // retrace solution
    vector<Cell> _best_solution;
    double _best_cost;
    double _best_wire;
    double _best_congestion;
    void keep_best_cost(double& cur_cost, double& congestion, double& wire);
    void keep_best_solution();
    void recover_best_solution();

    // SA related
    double T1;
    double cur_temperature;
    int x_range;
    int y_range;
    int _step_cnt;
    int _iteration_cnt;
    int _reject_num;
    int _move_num;
    int _uphill_num;
    double SA_iteration();
    void perturb();
    void deperturb();
    void set_range() {
        // recover_best_solution();
        // if(_iteration_cnt % 5 == 0) recover_best_solution();
        x_range = _placement->_boundary_width / (_iteration_cnt / 5 + 1); x_range = x_range < 2 ? 2 : x_range;
        y_range = _placement->_boundary_height / (_iteration_cnt / 5 + 1); y_range = y_range < 2 ? 2 : y_range;
        // x_range = int(_placement->_boundary_width * exp(-1/cur_temperature));
        // y_range = int(_placement->_boundary_height * exp(-1/cur_temperature));
    };

    // initial SA related
    int N;
    int _norm_num = 100;
    double _congestion_norm_factor;
    double _wire_length_norm_factor;
    double random_place(int steps);
    void calculate_norm_factor();

    // cost function related
    void set_feasible(bool feasible);
    int cal_net_cost(Net* cur_net);
    bool calculate_congestion_cost(double& congestion);
    void calculate_wire_length_cost(double& wire);
    double calculate_total_cost(double& congestion, double& wire, bool check_feasible);
    double calculate_total_cost(bool check_feasible);
    vector<int> _netlength;
    vector<MST*> _msts;
    void fill_demand(int x1, int y1, int z1, int x2, int y2, int z2, bool inc);
    void congestion_incremental_update(set<int>, bool);
    void adjH_incremental_update(Cell* cur_cell, bool inc);
    void record_cell_place();
    double cal_move_cell_num();
    void set_congest_feasible(bool feasible);
    void set_movecell_feasible(bool feasible);
    deque<bool> _congest_feasible_chain;
    deque<bool> _movecell_feasible_chain;
    int _congest_feasible_count;
    int _movecell_feasible_count;
    int _congest_feasible_chain_size = 100;
    int _movecell_feasible_chain_size = 100;
    bool movcell_overflow;

    // SA partial recovery
    int _perturb_type;
    int _perturb_val1;
    int _perturb_val2;
    int _temp_x;
    int _temp_y;
    int _best_iteration;

    // log lookup table
    vector<double> _log_table;

    // Clean up Placer
    void print_temp(){
        cout << "Temp " << fixed << setprecision(5) << cur_temperature << " Iteration " << _iteration_cnt << " has [Cost / Congetstion / Wire / Range / Move cell]: " 
            << setprecision(5) << _best_cost << " " << _best_congestion << " " << _best_wire << " (" << x_range << "," << y_range << ") " 
            << int(cal_move_cell_num()) << "/" << _placement->_maxMoveCell << endl;
        myusage.report(true, true);    
        };
    void print_congestion();
    void print_mcell_list();
    void print_nets(){
        print_start("Two pin nets");
        for(int i = 0 ; i < _placement->_numNets ; ++i){
            cout << "NET " << i << endl;
            vector<EDGE> two_pin_net = _msts[i]->get2pinnets();
            for(int j = 0, end_j = two_pin_net.size() ; j < end_j ; ++j){
                Pin* p1 = two_pin_net[j].first;
                Pin* p2 = two_pin_net[j].second;
                Cell* c1 = _placement->_cellArray[p1->getcellId()];
                Cell* c2 = _placement->_cellArray[p2->getcellId()];
                cerr << "   " << c1->getx()-_placement->_leftBoundary << "," << c1->gety()-_placement->_bottomBoundary << " ~ "
                    << c2->getx()-_placement->_leftBoundary << "," << c2->gety()-_placement->_bottomBoundary << endl;
            }
        }
        double wire;
        calculate_wire_length_cost(wire);
        cout << "Total MST wire length: " << wire << endl;
        print_end();
    }
    void clear();
    double rand_01() {return double(rand() % 100000) * double(1e-5);};

};

#endif // Placer_H
