#include "placer.h"
#include <cmath>
#include "utils.h"
#include <cstdlib>
#define PRINT_MODE

enum {
    MOVE,
    SWAP
};

Placer::Placer(Placement * placement)  : _placement(placement) {
    srand(0);
    _iteration_cnt = 1;
    _supply = vector<vector<int> >(_placement->_boundary_width, vector<int>(_placement->_boundary_height));
    _demand = vector<vector<int> >(_placement->_boundary_width, vector<int>(_placement->_boundary_height));
    _congestion = vector<vector<int> >(_placement->_boundary_width, vector<int>(_placement->_boundary_height));
    _netlength = vector<int>(_placement->_numNets, -1);
    _best_solution.reserve(_placement->_numCells);
}

void Placer::place(){
    double wire;
    // keep the initial placement
    _best_cost = calculate_total_cost(_best_congestion, _best_wire);
    keep_best_solution();
    // calculate the normalize factor of congestion and wirelength (highly congested condition will not be present in random floorplan with high probalbility)
    calculate_norm_factor();

    // specify prameters
    int k = 10;
    double P = 0.95;
    N = _placement->_numCells * k;
    int c = 100 ; // a small number for stage 2 in fast SA
    // random floorplanning for calculate avg cost
    double avg_cost = random_place(N);
    T1 = avg_cost / log(1 / P); // set initial temperature
    double terminate_temperature = 1e-2;
    int num_stage_1 = 2; // 1 num for 7 iteration
    int num_stage_2 = num_stage_1 + _placement->_numCells;
    // stage I : aims to put into the outline (minimize area)
    print_start("SA Floorplanning - Stage I");
    cur_temperature = T1;
    set_range();
#ifdef PRINT_MODE
    print_temp();
#endif
    SA_iteration();
    avg_cost = 0;
    print_end();
    // stage II : aims to put into the outline (minimize area)
    print_start("Fast-SA Floorplanning - Stage II");
    recover_best_solution();
    for (_iteration_cnt = num_stage_1; _iteration_cnt < num_stage_2; ++_iteration_cnt)
    {
        cur_temperature = T1 * avg_cost / _iteration_cnt / c;
        set_range();
#ifdef PRINT_MODE
    print_temp();
#endif
        avg_cost = SA_iteration();
    }
    print_end();
    // stage III : classical SA
    print_start("Fast-SA Floorplanning - Stage III");
    recover_best_solution();
    for (_iteration_cnt = num_stage_2;; ++_iteration_cnt)
    {
        cur_temperature = T1 * avg_cost / _iteration_cnt * 100 / c;
        set_range();
#ifdef PRINT_MODE
    print_temp();
#endif
        avg_cost = SA_iteration();
        if (cur_temperature < terminate_temperature || (double(_reject_num) / double(_move_num) > 0.97))
            break;
    }
    print_end();
    recover_best_solution();
    _placement->reportCell();
}

double Placer::SA_iteration()
{
    // local parameters
    double prev_cost, cur_cost;
    prev_cost = calculate_total_cost();

    _reject_num = 0;
    _uphill_num = 0;
    _move_num = 0;
    // initialize the feasible chain with one false
    _feasible_chain.push_back(false);
    double avg_cost = 0;
    do
    {
        ++_step_cnt;
        ++_move_num;
        perturb();

        double wire;
        double congestion;
        _beta = 1 - _feasible_count / _feasible_chain.size();
        cur_cost = calculate_total_cost(congestion, wire);
        cout << "Current [Cost / Congetstion / Wire]: " << cur_cost << " " << congestion << " " << wire << '\r';
        double d_cost = cur_cost - prev_cost;
        avg_cost += d_cost;
        float p = 1 / (exp(d_cost / cur_temperature)); // probapility for uphill move
        p = p > 1 ? 1 : p;
        if (d_cost > 0 && rand_01() > p) // reject
        {
            ++_reject_num;
            deperturb();
            set_feasible(false);
        }
        else
        {
            if (d_cost > 0)
                ++_uphill_num;
            prev_cost = cur_cost;
            if (cur_cost < _best_cost)
            {
                keep_best_cost(cur_cost, congestion, wire);
                keep_best_solution();
            }
            set_feasible(true);
        }
    } while (_move_num < 2 * N && _uphill_num < N);
    return avg_cost / double(_move_num);
}

void Placer::set_feasible(bool feasible)
{
    // if (_feasible_chain.size() > _feasible_chain_size)
    // {
    //     if (_feasible_chain.front() == true)
    //         --_feasible_count;
    //     _feasible_chain.pop_front();
    // }
    // _feasible_chain.push_back(feasible);
    // if (feasible)
    //     ++_feasible_count;
    // return;
}

void Placer::bd_congestion_init(double dw){
    for(int i = 0, end_i = _placement->_boundary_width ; i < end_i ; ++i){
        for(int j = 0, end_j = _placement->_boundary_height ; j < end_j ; ++j){
            _congestion[i][j] = _supply[i][j] - _demand[i][j] * dw;
        }
    }
}

int Placer::cal_net_cost(Net* cur_net){
    vector<Pin*> pinarray = cur_net->getPinArray();
    int max_x = 0;
    int max_y = 0;
    int max_z = 0;
    int min_x = 2147483647;
    int min_y = 2147483647;
    int min_z = 2147483647;
    for(int i = 0, end_i = pinarray.size() ; i < end_i ; ++i){
        Pin* cur_pin = pinarray[i];
        Cell* cur_cell = _placement->_cellArray[cur_pin->getcellId()];
        if(max_x < cur_cell->getx()) max_x = cur_cell->getx();
        if(min_x > cur_cell->getx()) min_x = cur_cell->getx();
        if(max_y < cur_cell->gety()) max_y = cur_cell->gety();
        if(min_y > cur_cell->gety()) min_y = cur_cell->gety();
        if(max_z < cur_pin->get_layer()) max_z = cur_pin->get_layer();
        if(min_z > cur_pin->get_layer()) min_z = cur_pin->get_layer();
    }
    return (max_x - min_x) + (max_y - min_y);
    // return (max_x - min_x) + (max_y - min_y) + (max_z - min_z);
}

void Placer::keep_best_cost(double &cost, double &congestion, double &wire){
    // record best cost
    _best_cost = cost;
    _best_wire = wire;
    _best_congestion = congestion;
    return;
}

void Placer::keep_best_solution(){
    _best_solution.clear();
    for(int i = 0, end_i = _placement->_numCells ; i < end_i ; ++i)  
        _best_solution.push_back(*(_placement->_cellArray[i]));
    
}

void Placer::recover_best_solution(){
    for(int i = 0, end_i = _placement->_numCells ; i < end_i ; ++i){
        delete _placement->_cellArray[i];
        _placement->_cellArray[i] = new Cell(_best_solution[i]);
    }
}

void Placer::perturb(){
    _perturb_type = rand_01() < 1 ? SWAP : MOVE;
    _perturb_val1 = rand() % (_placement->_numCells);
    _perturb_val2 = rand() % (_placement->_numCells);

    _temp_x = _placement->_cellArray[_perturb_val1]->getx();
    _temp_y = _placement->_cellArray[_perturb_val1]->gety();
    switch(_perturb_type){
        case MOVE:{
            int cur_x = _placement->_cellArray[_perturb_val1]->getx();
            cur_x = cur_x + rand() % x_range - x_range / 2;
            cur_x = cur_x < _placement->_leftBoundary ? _placement->_leftBoundary : cur_x;
            cur_x = cur_x > _placement->_rightBoundary ? _placement->_rightBoundary : cur_x;

            int cur_y = _placement->_cellArray[_perturb_val1]->gety();
            cur_y = cur_y + rand() % y_range - y_range / 2;
            cur_y = cur_y < _placement->_bottomBoundary ? _placement->_bottomBoundary : cur_y;
            cur_y = cur_y > _placement->_topBoundary ? _placement->_topBoundary : cur_y;

            _placement->_cellArray[_perturb_val1]->setx(rand() % _placement->_boundary_width + _placement->_leftBoundary);
            _placement->_cellArray[_perturb_val1]->sety(rand() % _placement->_boundary_height + _placement->_rightBoundary);}
            break;
        case SWAP:
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) _netlength[pin_ary1[i]->getNetId()] = -1;
            vector<Pin*> pin_ary2 = _placement->_cellArray[_perturb_val2]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) _netlength[pin_ary2[i]->getNetId()] = -1;

            _placement->_cellArray[_perturb_val1]->setx(_placement->_cellArray[_perturb_val2]->getx());
            _placement->_cellArray[_perturb_val1]->sety(_placement->_cellArray[_perturb_val2]->gety());
            _placement->_cellArray[_perturb_val2]->setx(_temp_x);
            _placement->_cellArray[_perturb_val2]->sety(_temp_y);
            break;
    }
}

void Placer::deperturb(){
    switch(_perturb_type){
        case MOVE:
            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);
            break;
        case SWAP:
            _placement->_cellArray[_perturb_val2]->setx(_placement->_cellArray[_perturb_val1]->getx());
            _placement->_cellArray[_perturb_val2]->sety(_placement->_cellArray[_perturb_val1]->gety());
            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);
            break;
    }
}

double Placer::random_place(int steps){
    print_start("Random Placement");
    double total_cost = 0; // recording the average uphill move cost
    double best_cost, prev_cost, cur_cost;
    int t = 0; // times of uphill moves
    best_cost = prev_cost = calculate_total_cost();
    // keep best
    do
    {
        for (int i = 0; i < steps; ++i)
        {
            perturb();
            cur_cost = calculate_total_cost();
            if (cur_cost - prev_cost > 0)
            {
                ++t;
                total_cost += (cur_cost - prev_cost);
                prev_cost = cur_cost;
            }
            if (best_cost > cur_cost)
            {
                best_cost = cur_cost;
            }
        }
    } while (total_cost == 0);
    // recover bset
    // update node
    total_cost /= t;
    cout << "Avg cost: " << total_cost << endl;
    print_end();
    return total_cost;
}

void Placer::calculate_norm_factor(){
    cout << endl;
    cout << "============ Calculate norm factor ==============" << endl;
    _congestion_norm_factor = 0;
    _wire_length_norm_factor = 0;
    for (size_t i = 0; i < _norm_num; ++i)
    {
        perturb();
        double congestion;
        double wire;
        calculate_congestion_cost(congestion);
        calculate_wire_length_cost(wire);
        _congestion_norm_factor += congestion;
        _wire_length_norm_factor += wire;
    }
    _congestion_norm_factor /= double(_norm_num);
    _wire_length_norm_factor /= double(_norm_num);
    cout << "Average congestion: " << setw(10) << _congestion_norm_factor << '\n';
    cout << "Average wire length: " << setw(8) << _wire_length_norm_factor << '\n';
    cout << "=================================================" << endl;
}

double Placer::calculate_total_cost(){
    double congestion;
    double wire;
    calculate_congestion_cost(congestion);
    calculate_wire_length_cost(wire);
    // calculate cost
    _beta = 0;
    double cost_ratio = 0.5 + (1 - _beta);
    cost_ratio = cost_ratio > 0.9 ? 0.9 : cost_ratio;
    return wire * (1 - cost_ratio) + congestion * cost_ratio;
}

double Placer::calculate_total_cost(double& congestion, double& wire){
    calculate_congestion_cost(congestion);
    calculate_wire_length_cost(wire);
    // calculate cost
    _beta = 0;
    double cost_ratio = 0.5 + (1 - _beta);
    cost_ratio = cost_ratio > 0.9 ? 0.9 : cost_ratio;
    return wire * (1 - cost_ratio) + congestion * cost_ratio;
}

void Placer::calculate_congestion_cost(double& congestion){
    congestion = 0;
}

void Placer::calculate_wire_length_cost(double& wire){
    wire = 0;
    for(int i = 0, end_i = _placement->_numNets ; i < end_i ; ++i){
        if(_netlength[i] == -1) {int wire_len = cal_net_cost(_placement->_netArray[i]); wire += wire_len; _netlength[i] = wire_len;}
        else wire += _netlength[i];
    }
}