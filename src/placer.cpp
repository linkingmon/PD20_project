#include "placer.h"
#include <cmath>
#include <cstdlib>
#include <cassert>
#define PRINT_MODE
#define EXP 2.71828182

enum {
    MOVE,
    SWAP
};

Placer::Placer(Placement * placement)  : _placement(placement) {
    srand(0);
    _iteration_cnt = 1;
    _netlength = vector<int>(_placement->_numNets, -1);
    _best_solution.reserve(_placement->_numCells);
    // initialize log lookup table
    _log_table.reserve(_placement->_boundary_height + _placement->_boundary_width + _placement->_numLayers);
    _log_table.push_back(0); // dummy log(0)
    for(int i = 1, end_i = _placement->_boundary_height + _placement->_boundary_width + _placement->_numLayers ; i < end_i ; ++i)
        _log_table.push_back(_log_table[i-1]+log(i));
    // initialize MST per Net (quadratic time) // but incremental is linear
    _msts.reserve(_placement->_numNets);
    for(int i = 0, end_i = _placement->_numNets ; i < end_i ; ++i){
        _msts.push_back(new MST(_placement->_netArray[i]->getPinArray(), _placement));
    }
    // initialize mcell list
    init_mcell_list();
    // initialize the congection map
    init_supply_map();
    init_demand_map();
    // record cell place for checking max cell move
    record_cell_place();
    // init fix Cell list
    init_fixCell_list();
    // init history chain
    _congest_feasible_count = 0;
    _movecell_feasible_count = 0;
    // initialize the feasible chain with one false
    // double wire;
    // calculate_wire_length_cost(wire);
    // cout << wire<<endl;exit(-1);
    // print_congestion(); exit(-1);
}

void Placer::place(){
    set_range();
    double wire;
    // keep the initial placement
    keep_best_solution();
    // calculate the normalize factor of congestion and wirelength (highly congested condition will not be present in random floorplan with high probalbility)
    calculate_norm_factor();
    recover_best_solution();
    _best_cost = calculate_total_cost(_best_congestion, _best_wire, true);
    // print_congestion();

    // specify prameters
    int k = 10;
    double P = 0.95;
    N = _placement->_numCells * k;
    int c = 10000 ; // a small number for stage 2 in fast SA
    // random floorplanning for calculate avg cost
    double avg_cost = random_place(N);
    T1 = avg_cost / log(1 / P); // set initial temperature
    double terminate_temperature = 1e-4;
    int num_stage_1 = 2; 
    // int num_stage_2 = num_stage_1 + _placement->_numCells;
    int num_stage_2 = num_stage_1 + 10;

    // stage I : aims to put into the outline (minimize area)
    print_start("SA Floorplanning - Stage I");
    cur_temperature = T1;
    set_range();
#ifdef PRINT_MODE
    print_temp();
#endif
    SA_iteration();
    print_end();

    // stage II : aims to put into the outline (minimize area)
    print_start("Fast-SA Floorplanning - Stage II");
    recover_best_solution();
    avg_cost = 1e-3;
    for (_iteration_cnt = num_stage_1;  ; ++_iteration_cnt)
    {
        cur_temperature = T1 * avg_cost / _iteration_cnt / c;
        set_range();
#ifdef PRINT_MODE
    print_temp();
#endif
        avg_cost = SA_iteration();
        if(_iteration_cnt > _best_iteration + 10)
            break;
    }
    print_end();

    // stage III : classical SA
    print_start("Fast-SA Floorplanning - Stage III");
    recover_best_solution();
    for (; ; ++_iteration_cnt)
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
    // print_congestion();
    print_nets();
}

double Placer::SA_iteration()
{
    // local parameters
    double prev_cost, cur_cost;
    prev_cost = calculate_total_cost(false);

    _reject_num = 0;
    _uphill_num = 0;
    _move_num = 0;
    double avg_cost = 0;
    do
    {
        ++_step_cnt;
        ++_move_num;
        perturb();

        double wire;
        double congestion;
        cur_cost = calculate_total_cost(congestion, wire, true);
        double d_cost = cur_cost - prev_cost;
        avg_cost += d_cost;
        float p = 1 / (exp(d_cost / cur_temperature)); // probapility for uphill move
        p = p > 1 ? 1 : p;
        // cout << "Current [Cost / Congetstion / Wire]: " << cur_cost << " " << congestion << " " << wire << " " << cur_cost << "　" << prev_cost << " " << p << '\n';
        cout << "Current [Cost / Congetstion / Wire]: " << setprecision(10) << cur_cost << " " << congestion << " " << wire << '\r';
        // if(_iteration_cnt == 2 && _move_num == 10) exit(-1);
        if (d_cost > 0 && rand_01() > p) // reject
        {
            // cout << "Reject" << endl;
            ++_reject_num;
            deperturb();
            // cout << "Wire after deperturb: " << wire_debug << endl;
            // set_feasible(false);
        }
        else
        {
            // cout << "Take" << endl;
            if (d_cost > 0)
                ++_uphill_num;
            prev_cost = cur_cost;
            if (cur_cost < _best_cost && !movcell_overflow)
            {
                keep_best_cost(cur_cost, congestion, wire);
                keep_best_solution();
            }
            // set_feasible(true);
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
}

void Placer::keep_best_cost(double &cost, double &congestion, double &wire){
    // record best cost
    _best_cost = cost;
    _best_wire = wire;
    _best_congestion = congestion;
    _best_iteration = _iteration_cnt;
    return;
}

void Placer::keep_best_solution(){
    _best_solution.clear();
    for(int i = 0, end_i = _placement->_numCells ; i < end_i ; ++i)  
        _best_solution.push_back(*(_placement->_cellArray[i]));
}

void Placer::recover_best_solution(){
    // retrieve the best cell location
    for(int i = 0, end_i = _placement->_numCells ; i < end_i ; ++i){
        delete _placement->_cellArray[i];
        _placement->_cellArray[i] = new Cell(_best_solution[i]);
    }
    // clear all nets wire length
    _netlength = vector<int>(_placement->_numNets, -1);
    // clear all MST in the net
    _msts.clear();
    for(int i = 0, end_i = _placement->_numNets ; i < end_i ; ++i){
        _msts.push_back(new MST(_placement->_netArray[i]->getPinArray(), _placement));
    }
    // initial mcell list
    init_mcell_list();
    // re calculate the congestion demand map
    init_demand_map();
}

void Placer::perturb(){
    _perturb_type = rand_01() < 0.2 ? MOVE : SWAP;
    _perturb_val1 = rand() % (_fixCell_list.size());
    _perturb_val1 = _fixCell_list[_perturb_val1];
    _perturb_val2 = rand() % (_fixCell_list.size());
    _perturb_val2 = _fixCell_list[_perturb_val2];

    _temp_x = _placement->_cellArray[_perturb_val1]->getx();
    _temp_y = _placement->_cellArray[_perturb_val1]->gety();

    // cout << "TAKE CELL: " << _perturb_val1 << " " << _perturb_val2 << '\n';
    // cout << ">>> TAKE TYPE: " << (_perturb_type == MOVE ? "MOVE" : "SWAP") << endl;
    // cout << "CELL 1 COORD: " << _temp_x << " " << _temp_y << '\n';
    // cout << "CELL 2 COORD: " << _placement->_cellArray[_perturb_val2]->getx() << " " << _placement->_cellArray[_perturb_val2]->gety() << '\n';

    switch(_perturb_type){
        case MOVE:{
            // for incremental update
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            // wire length inc.
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) _netlength[pin_ary1[i]->getNetId()] = -1;
            // record modify nets
            set<int> modify_netIds;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) modify_netIds.insert(pin_ary1[i]->getNetId());
            // remove net congestion before update 2 pin nets
            congestion_incremental_update(modify_netIds, false);
            // cal x, y
            int cur_x = _placement->_cellArray[_perturb_val1]->getx();
            int cur_y = _placement->_cellArray[_perturb_val1]->gety();
            // AdjH/Same demand
            adjH_incremental_update(_placement->_cellArray[_perturb_val1], false);

            // set moving range
            cur_x = cur_x + rand() % x_range - x_range / 2;
            cur_x = cur_x < _placement->_leftBoundary ? _placement->_leftBoundary : cur_x;
            cur_x = cur_x > _placement->_rightBoundary ? _placement->_rightBoundary : cur_x;
            cur_y = cur_y + rand() % y_range - y_range / 2;
            cur_y = cur_y < _placement->_bottomBoundary ? _placement->_bottomBoundary : cur_y;
            cur_y = cur_y > _placement->_topBoundary ? _placement->_topBoundary : cur_y;

            // cerr << "MOVE from (" <<  _placement->_cellArray[_perturb_val1]->getx() << "," << _placement->_cellArray[_perturb_val1]->gety() << ") to ("
            //     << cur_x << "," << cur_y << ")" << endl;
            _placement->_cellArray[_perturb_val1]->setx(cur_x);
            _placement->_cellArray[_perturb_val1]->sety(cur_y);

            // MST inc.
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                if(pin_ary1[i]->getNetId()!=-1)
                    _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            // ass net congestion after update 2 pin nets
            congestion_incremental_update(modify_netIds, true);
            // AdjH/Same demand
            adjH_incremental_update(_placement->_cellArray[_perturb_val1], true);

            }
            break;
        case SWAP:{
            // for incremental
            // cerr << "TEST 5" << endl;
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);
            // cerr << "TEST 5" << endl;

            vector<Pin*> pin_ary2 = _placement->_cellArray[_perturb_val2]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) if(pin_ary2[i]->getNetId()!=-1) _netlength[pin_ary2[i]->getNetId()] = -1;
            set<int> modify_netIds2;
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) if(pin_ary2[i]->getNetId()!=-1) modify_netIds2.insert(pin_ary2[i]->getNetId());
            congestion_incremental_update(modify_netIds2, false);

            adjH_incremental_update(_placement->_cellArray[_perturb_val1], false);
            adjH_incremental_update(_placement->_cellArray[_perturb_val2], false);

            _placement->_cellArray[_perturb_val1]->setx(_placement->_cellArray[_perturb_val2]->getx());
            _placement->_cellArray[_perturb_val1]->sety(_placement->_cellArray[_perturb_val2]->gety());
            _placement->_cellArray[_perturb_val2]->setx(_temp_x);
            _placement->_cellArray[_perturb_val2]->sety(_temp_y);
            // cerr << pin_ary1[1]->getNetId() << endl;
            
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                if(pin_ary1[i]->getNetId()!=-1)
                    _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            // cerr << "TEST 5" << endl;
            congestion_incremental_update(modify_netIds1, true);
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) 
                if(pin_ary2[i]->getNetId()!=-1)
                    _msts[pin_ary2[i]->getNetId()]->update(pin_ary2[i], _placement->_netArray[pin_ary2[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds2, true);
            adjH_incremental_update(_placement->_cellArray[_perturb_val1], true);
            adjH_incremental_update(_placement->_cellArray[_perturb_val2], true);
            }
            break;
    }
}

void Placer::deperturb(){

    // cout << "D TAKE CELL: " << _perturb_val1 << " " << _perturb_val2 << '\n';
    // cout << "D TAKE TYPE: " << _perturb_type << endl;
    // cout << "D CELL 1 COORD: " << _placement->_cellArray[_perturb_val1]->getx() << " " << _placement->_cellArray[_perturb_val1]->getx() << '\n';
    // cout << "D CELL 2 COORD: " << _placement->_cellArray[_perturb_val2]->getx() << " " << _placement->_cellArray[_perturb_val2]->gety() << '\n';

    // cerr << "DEPERTURB" << endl;
    switch(_perturb_type){
        case MOVE:{
            // for incremental
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);
            adjH_incremental_update(_placement->_cellArray[_perturb_val1], false);

            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);
            adjH_incremental_update(_placement->_cellArray[_perturb_val1], true);
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                if(pin_ary1[i]->getNetId()!=-1)
                    _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds1, true);
            }
            break;
        case SWAP:{
            // for incremental
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) if(pin_ary1[i]->getNetId()!=-1) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);

            vector<Pin*> pin_ary2 = _placement->_cellArray[_perturb_val2]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) if(pin_ary2[i]->getNetId()!=-1) _netlength[pin_ary2[i]->getNetId()] = -1;
            set<int> modify_netIds2;
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) if(pin_ary2[i]->getNetId()!=-1) modify_netIds2.insert(pin_ary2[i]->getNetId());
            congestion_incremental_update(modify_netIds2, false);

            adjH_incremental_update(_placement->_cellArray[_perturb_val1], false);
            adjH_incremental_update(_placement->_cellArray[_perturb_val2], false);

            _placement->_cellArray[_perturb_val2]->setx(_placement->_cellArray[_perturb_val1]->getx());
            _placement->_cellArray[_perturb_val2]->sety(_placement->_cellArray[_perturb_val1]->gety());
            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);

            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                if(pin_ary1[i]->getNetId()!=-1)
                    _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds1, true);
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) 
                if(pin_ary2[i]->getNetId()!=-1)
                    _msts[pin_ary2[i]->getNetId()]->update(pin_ary2[i], _placement->_netArray[pin_ary2[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds2, true);

            adjH_incremental_update(_placement->_cellArray[_perturb_val1], true);
            adjH_incremental_update(_placement->_cellArray[_perturb_val2], true);
            }
            break;
    }
}

double Placer::random_place(int steps){
    print_start("Random Placement");
    double total_cost = 0; // recording the average uphill move cost
    double best_cost, prev_cost, cur_cost;
    int t = 0; // times of uphill moves
    best_cost = prev_cost = calculate_total_cost(false);
    // keep best
    do
    {
        for (int i = 0; i < steps; ++i)
        {
            perturb();
            cur_cost = calculate_total_cost(true);
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
    // _congestion_norm_factor = 0;
    _wire_length_norm_factor = 0;
    for (size_t i = 0; i < _norm_num; ++i)
    {
        perturb();
        // double congestion;
        double wire;
        // calculate_congestion_cost(congestion);
        calculate_wire_length_cost(wire);
        // _congestion_norm_factor += congestion;
        _wire_length_norm_factor += wire;
    }
    // _congestion_norm_factor /= double(_norm_num);
    _wire_length_norm_factor /= double(_norm_num);
    // cout << "Average congestion: " << setw(10) << _congestion_norm_factor << '\n';
    cout << "Average wire length: " << setw(8) << _wire_length_norm_factor << '\n';
    // _congestion_norm_factor = 1/_congestion_norm_factor;
    _wire_length_norm_factor = 1/_wire_length_norm_factor;
    cout << "=================================================" << endl;
}

double Placer::calculate_total_cost(bool check_feasible = false){
    double congestion;
    double wire;
    bool congest_overflow = calculate_congestion_cost(congestion);
    calculate_wire_length_cost(wire);
    // calculate cost
    if(check_feasible) set_congest_feasible(congest_overflow);
    double cost_ratio_congest = _congest_feasible_count / _congest_feasible_chain.size();
    // cost_ratio_congest = cost_ratio_congest > 0.9 ? 0.9 : cost_ratio_congest;
    cost_ratio_congest *= 0.3;

    int excess_move_cell = cal_move_cell_num() - _placement->_maxMoveCell;
    movcell_overflow = excess_move_cell > 0;
    if(check_feasible) set_movecell_feasible(movcell_overflow);
    excess_move_cell = (excess_move_cell > 0) ? excess_move_cell : 0;
    double cost_ratio_move = double(_movecell_feasible_count) / _movecell_feasible_chain.size();
    cost_ratio_move *= 0.15;

    return wire * (1 - cost_ratio_congest - cost_ratio_move) * _wire_length_norm_factor 
                + congestion * cost_ratio_congest
                + (pow(2, cal_move_cell_num()/_placement->_maxMoveCell)-1) * cost_ratio_move ;
}

double Placer::calculate_total_cost(double& congestion, double& wire, bool check_feasible = false){
    bool congest_overflow = calculate_congestion_cost(congestion);
    calculate_wire_length_cost(wire);
    // calculate cost
    if(check_feasible) set_congest_feasible(congest_overflow);
    double cost_ratio_congest = _congest_feasible_count / _congest_feasible_chain.size();
    // cost_ratio_congest = cost_ratio_congest > 0.9 ? 0.9 : cost_ratio_congest;
    cost_ratio_congest *= 0.3;

    int excess_move_cell = cal_move_cell_num() - _placement->_maxMoveCell;
    movcell_overflow = excess_move_cell > 0;
    if(check_feasible) set_movecell_feasible(movcell_overflow);
    excess_move_cell = (excess_move_cell > 0) ? excess_move_cell : 0;
    double cost_ratio_move = double(_movecell_feasible_count) / _movecell_feasible_chain.size();
    // cost_ratio_move = cost_ratio_move > 0.9 ? 0.9 : cost_ratio_move;
    // cost_ratio_move = (1-cost_ratio_move*1.05 > 0) ? 1e8 : 0.3*log(1-cost_ratio_move*1.05);
    cost_ratio_move *= 0.15;
    // cout << wire*_wire_length_norm_factor << " " << (1 - cost_ratio_congest - cost_ratio_move) << "|" << congestion << " " << cost_ratio_congest << "|"
    //     << (pow(2, cal_move_cell_num()/_placement->_maxMoveCell)-1) << " " << cost_ratio_move << endl;
    // cout << wire * (1 - cost_ratio_congest - cost_ratio_move) * _wire_length_norm_factor 
    //             + congestion * cost_ratio_congest
    //             + (pow(2, cal_move_cell_num()/_placement->_maxMoveCell)-1) * cost_ratio_move << endl;
    //             // exit(-1);
    // cout << (pow(2, cal_move_cell_num()/_placement->_maxMoveCell)-1) << endl;
    // if(_iteration_cnt == 2) exit(-1);

    return wire * (1 - cost_ratio_congest - cost_ratio_move) * _wire_length_norm_factor 
                + congestion * cost_ratio_congest
                // + excess_move_cell * cost_ratio_move ;
                + (pow(2, cal_move_cell_num()/_placement->_maxMoveCell)-1) * cost_ratio_move ;
}

bool Placer::calculate_congestion_cost(double& congestion){
    // add all the overflow grid values
    // print_mcell_list();
    // print_congestion();
    bool overflow = false;
    congestion = 0;
    // int cnt = 0;
    for(int i = 0 ; i < _placement->_boundary_width ; ++i){
        for(int j = 0 ; j < _placement->_boundary_height ; ++j){
            for(int k = 0 ; k < _placement->_numLayers ; ++k){
                if(_demand[i][j][k] > _supply[i][j][k]){
                    congestion += (_demand[i][j][k] - _supply[i][j][k]) / _supply[i][j][k]; // the overflow ratio
                    overflow = true;
                    // cout << ++cnt << endl;
                }
            }
        }
    }
    congestion /= (_placement->_boundary_width * _placement->_boundary_height * _placement->_numLayers);
    return overflow;
}

void Placer::calculate_wire_length_cost(double& wire){
    // cerr << "CAL WIRE 1" << endl;
    wire = 0;
    for(int i = 0, end_i = _placement->_numNets ; i < end_i ; ++i){
        if(_netlength[i] == -1) {
            int wire_len = 0;
            // wire_len = cal_net_cost(_placement->_netArray[i]); 
            vector<EDGE> two_pin_net = _msts[i]->get2pinnets();
            // cerr << "GET NET" << endl;
            for(int j = 0, end_j = two_pin_net.size() ; j < end_j ; ++j){
                Pin* p1 = two_pin_net[j].first;
                Pin* p2 = two_pin_net[j].second;
                Cell* c1 = _placement->_cellArray[p1->getcellId()];
                Cell* c2 = _placement->_cellArray[p2->getcellId()];
                // cerr << "RUN NET" << j << endl;
                // cerr << p1->getcellId() << endl;
                // cerr << p1->get_name() << endl;
                // cerr << p2->getcellId() << endl;
                // cerr << p2->get_name() << endl;
                // cerr << "(" << p1->getcellId() << "/" << p1->get_name() << ") (" << p2->getcellId() << "/" << p2->get_name() << ")" << endl;
                // cerr << "Wire " << abs(c1->getx() - c2->getx()) + abs(c1->gety() - c2->gety()) + abs(p1->get_layer() - p2->get_layer()) << endl;
                // cerr << abs(c1->getx() - c2->getx()) + abs(c1->gety() - c2->gety()) + abs(p1->get_layer() - p2->get_layer()) << endl;
                wire_len += abs(c1->getx() - c2->getx()) + abs(c1->gety() - c2->gety()) + abs(p1->get_layer() - p2->get_layer());
            }
            // exit(-1);
            wire += wire_len; 
            _netlength[i] = wire_len;
        }
        else wire += _netlength[i];
        // wire += cal_net_cost(_placement->_netArray[i]);
    }
    // cerr << "CAL WIRE 2" << endl;
}

void Placer::init_supply_map(){
    // cerr << "INIT SUPPLY" << endl;
    // default supply
    _supply = vector<vector<vector<double> > >(_placement->_boundary_width, vector<vector<double> >(_placement->_boundary_height, vector<double>(_placement->_numLayers)));
    for(int i = 0 ; i < _placement->_numLayers ; ++i){ // z
        int cur_layer_supply = _placement->layers[i]->get_supply();
        for(int j = 0 ; j < _placement->_boundary_width ; ++j){ // x
            for(int k = 0 ; k < _placement->_boundary_height ; ++k){ // y
                _supply[j][k][i] = int(cur_layer_supply * 1.4);
            }
        }
    }
    // cerr << "INIT SUPPLY " << _placement->_numNonDefault << endl;
    // non default supply
    for(int i = 0; i < _placement->_numNonDefault ; ++i){
        NonDefault* cur_grid = _placement->nondefault[i];
        // cur_grid->print();
        double& cur_val = _supply[cur_grid->getx()-_placement->_leftBoundary][cur_grid->gety()-_placement->_bottomBoundary][cur_grid->getz()-1];
        cur_val += cur_grid->get_offset();
        if(cur_val <= 0) cur_val = 1e-5;
    }
    // cerr << "INIT SUPPLY DONE" << endl;
    return;
}

void Placer::init_demand_map(){
    // wire demand
    _demand = vector<vector<vector<double> > >(_placement->_boundary_width, vector<vector<double> >(_placement->_boundary_height, vector<double>(_placement->_numLayers, 0)));
    for(int i = 0 ; i < _placement->_numNets ; ++i){
        vector<Pin*> pin_ary = _placement->_netArray[i]->getPinArray();
        vector<EDGE> two_pin_net = _msts[i]->get2pinnets();
        for(int j = 0, end_j = two_pin_net.size() ; j < end_j ; ++j){
            Pin* p1 = two_pin_net[j].first;
            Pin* p2 = two_pin_net[j].second;
            Cell* c1 = _placement->_cellArray[p1->getcellId()];
            Cell* c2 = _placement->_cellArray[p2->getcellId()];
            // relative coordinate
            fill_demand(c1->getx()-_placement->_leftBoundary, c1->gety()-_placement->_bottomBoundary, p1->get_layer(), 
                        c2->getx()-_placement->_leftBoundary, c2->gety()-_placement->_bottomBoundary, p2->get_layer(), true);
        }
    }
    // blockage demand
    for(int i = 0 ; i < _placement->_numCells ; ++i){
        Master* mcell = _placement->_cellArray[i]->get_master();
        for(int j = 0, end_j = mcell->get_blockage() ; j < end_j ; ++j){
            // cerr << _placement->_cellArray[i]->getx()-_placement->_leftBoundary << " " << _placement->_cellArray[i]->gety()-_placement->_bottomBoundary << " "
            //     << mcell->get_block_layer(j) << " " << mcell->get_block_demand(j) << endl;
            _demand[_placement->_cellArray[i]->getx()-_placement->_leftBoundary][_placement->_cellArray[i]->gety()-_placement->_bottomBoundary][mcell->get_block_layer(j)] += mcell->get_block_demand(j);
        }
    }
    // adjH and Same demand
    map<int,int>::iterator iter, adj_iter, same_iter;
    for(int x = 0 ; x < _placement->_boundary_width - 1; ++x){
        for(int y = 0 ; y < _placement->_boundary_height ; ++y){
            map<int, int> cur_map = _mcell_list[x][y];
            map<int, int> adj_map = _mcell_list[x+1][y];
            for(iter = cur_map.begin() ; iter != cur_map.end() ; ++iter){
                int MC1_id = iter->first;
                int MC1_num = iter->second;
                vector<ExtraDemand> adjH = _placement->masters[MC1_id]->get_adjHDemand();
                vector<ExtraDemand> Same = _placement->masters[MC1_id]->get_sameDemand();
                
                for(size_t i = 0 ; i < adjH.size() ; i++){  // adjHDemand
                    int MC2_id = adjH[i]._extraId;
                    adj_iter = adj_map.find(MC2_id);
                    if( adj_iter == adj_map.end() ) continue;
                    int MC2_num = adj_iter->second;
                    int layer = adjH[i]._layer;
                    int demand = adjH[i]._demand;
                    int MC_num = min(MC1_num, MC2_num);
                    _demand[x][y][layer] += MC_num * demand;
                    _demand[x+1][y][layer] += MC_num * demand;
                }

                for(size_t i = 0 ; i < Same.size() ; i++){  // sameDemand
                    int MC2_id = Same[i]._extraId;
                    if(MC2_id < MC1_id ) continue;
                    same_iter = cur_map.find(MC2_id);
                    if( same_iter == cur_map.end() ) continue;
                    int MC2_num = same_iter->second;
                    int layer = Same[i]._layer;
                    int demand = Same[i]._demand;
                    int MC_num = min(MC1_num, MC2_num);
                    _demand[x][y][layer] += MC_num * demand;                
                }
            }
            
        }
    }
    return;
}

void Placer::fill_demand(int x1, int y1, int z1, int x2, int y2, int z2, bool inc){
    int reorder_x1 = x1;
    int reorder_x2 = x2;
    int reorder_y1 = y1;
    int reorder_y2 = y2;
    int reorder_z1 = z1;
    int reorder_z2 = z2;
    if(x1 > x2) swap(reorder_x1, reorder_x2);
    if(y1 > y2) swap(reorder_y1, reorder_y2);
    if(z1 > z2) swap(reorder_z1, reorder_z2);
    int x_dif = reorder_x2 - reorder_x1;
    int y_dif = reorder_y2 - reorder_y1;
    int z_dif = reorder_z2 - reorder_z1;
    // cerr << x_dif << " " << y_dif << " " << z_dif << endl;
    if(x_dif > 5 || y_dif > 5) return; // apporximatiion, if the pin is too far, the congestion is not important
    for(int i = reorder_x1 ; i <= reorder_x2 ; ++i){
        for(int j = reorder_y1 ; j <= reorder_y2 ; ++j){
            for(int k = reorder_z1 ; k <= reorder_z2 ; ++k){
                int x_dif1 = abs(i - x1);
                int y_dif1 = abs(j - y1);
                int z_dif1 = abs(k - z1);
                int x_dif2 = abs(i - x2);
                int y_dif2 = abs(j - y2);
                int z_dif2 = abs(k - z2);
                double log_prob = (_log_table[x_dif1 + y_dif1 + z_dif1] - _log_table[x_dif1] - _log_table[y_dif1] - _log_table[z_dif1])
                                    + (_log_table[x_dif2 + y_dif2 + z_dif2] - _log_table[x_dif2] - _log_table[y_dif2] - _log_table[z_dif2])
                                    - (_log_table[x_dif + y_dif + z_dif] - _log_table[x_dif] - _log_table[y_dif] - _log_table[z_dif]);
                if(inc) _demand[i][j][k] += exp(log_prob);
                else _demand[i][j][k] -= exp(log_prob);
                // cerr << "WIRE DEMAND " << i << " " << j << " " << k << ((inc) ? " INC" : " DEC") << exp(log_prob) << endl;
                // cerr << "FILL " << exp(log_prob) << endl;
            }
        }
    }
}

void Placer::congestion_incremental_update(set<int> netIds, bool inc){
    // cout << "Congestion incremental update !" << endl;
    // _placement->reportCell();
    set<int>::iterator iter;
    for(iter = netIds.begin() ; iter != netIds.end() ; ++iter){
        // cout << "NET " << *iter << endl;
        vector<EDGE> two_pin_net = _msts[*iter]->get2pinnets();
        for(int j = 0, end_j = two_pin_net.size() ; j < end_j ; ++j){
            Pin* p1 = two_pin_net[j].first;
            Pin* p2 = two_pin_net[j].second;
            Cell* c1 = _placement->_cellArray[p1->getcellId()];
            Cell* c2 = _placement->_cellArray[p2->getcellId()];
            // cerr << "   " << c1->getx()-_placement->_leftBoundary << "," << c1->gety()-_placement->_bottomBoundary << " ~ "
            //     << c2->getx()-_placement->_leftBoundary << "," << c2->gety()-_placement->_bottomBoundary << endl;
            fill_demand(c1->getx()-_placement->_leftBoundary, c1->gety()-_placement->_bottomBoundary, p1->get_layer(),
                    c2->getx()-_placement->_leftBoundary, c2->gety()-_placement->_bottomBoundary, p2->get_layer(), inc);
        }
    }
}

void Placer::print_congestion(){
#ifdef PRINT_MODE
    print_start("Congestion Map");
#endif
    int cnt = 0;
    int x_range = _placement->_boundary_width;
    // int x_range = 5;
    int y_range = _placement->_boundary_height;
    // int y_range = 5;
    for(int k = 0 ; k < _placement->_numLayers ; ++k){
        cout << "Layer " << k << " [Supply(with 0.2 margin) / Demand / Overflow]" << endl;
        for(int i = 0 ; i < x_range ; ++i){
            for(int j = 0 ; j < y_range ; ++j){
                cout << setw(5) << setprecision(3) << _supply[i][j][k] << " ";
            }
            cout << " | ";
            for(int j = 0 ; j < y_range ; ++j){
                cout << setw(5) << setprecision(3) << _demand[i][j][k] << " ";
            }
            cout << " | ";
            for(int j = 0 ; j < y_range ; ++j){
                cout << setw(5) << setprecision(3) << _supply[i][j][k] - _demand[i][j][k] << " ";
                if(_supply[i][j][k]/0.9 - _demand[i][j][k]<0) ++cnt;
            }
            cout << '\n';
        }
    }
    cout << "Total Overflow: " << cnt << endl;
#ifdef PRINT_MODE
    print_end();
#endif
}

void Placer::init_mcell_list(){
// #ifdef PRINT_MODE
//     print_start("Init Mcell List");
// #endif
    _mcell_list = vector<vector<map<int,int> > >(_placement->_boundary_width, vector<map<int,int> >(_placement->_boundary_height));
    for(int i = 0, end_i = _placement->_numCells ; i < end_i ; ++i){
        Cell* cur_cell = _placement->_cellArray[i];
        map<int,int>& cur_map = _mcell_list[cur_cell->getx() - _placement->_leftBoundary][cur_cell->gety() - _placement->_bottomBoundary];
        if(cur_map.count(cur_cell->get_master()->getId()) == 0)
            cur_map[cur_cell->get_master()->getId()] = 1;
        else
            cur_map[cur_cell->get_master()->getId()] += 1;
    }
// #ifdef PRINT_MODE
//     print_mcell_list();
//     print_end();
// #endif
}

void Placer::adjH_incremental_update(Cell* cur_cell, bool inc){
    // print_mcell_list();
    map<int,int>::iterator iter, adj_iter, same_iter;
    int cur_mcell_Id = cur_cell->get_master()->getId();
    int cur_x = cur_cell->getx() - _placement->_leftBoundary;
    int cur_y = cur_cell->gety() - _placement->_bottomBoundary;
    // cerr << "Cur x y inc: " << cur_x << " " << cur_y << " " << inc << endl;
    if(inc){
        // insert into map
        if(_mcell_list[cur_x][cur_y].count(cur_mcell_Id) == 0) _mcell_list[cur_x][cur_y][cur_mcell_Id] = 1;
        else _mcell_list[cur_x][cur_y][cur_mcell_Id] += 1;
        // update
        int MC1_num = _mcell_list[cur_x][cur_y][cur_mcell_Id];
        vector<ExtraDemand> adjH = _placement->masters[cur_mcell_Id]->get_adjHDemand();
        vector<ExtraDemand> Same = _placement->masters[cur_mcell_Id]->get_sameDemand();
        // adjG Demand
        if(cur_x > 0){ // have left side
            map<int, int>& adj_map = _mcell_list[cur_x-1][cur_y];
            for(size_t i = 0 ; i < adjH.size() ; i++){ 
                int MC2_id = adjH[i]._extraId;
                adj_iter = adj_map.find(MC2_id);
                if( adj_iter == adj_map.end() ) continue;
                int MC2_num = adj_iter->second;
                int layer = adjH[i]._layer;
                int demand = adjH[i]._demand;
                int MC_num = min(MC1_num, MC2_num) - min(MC1_num-1, MC2_num);
                // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                // cerr << "DEMAND " << cur_x-1 << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                _demand[cur_x][cur_y][layer] += MC_num * demand;
                _demand[cur_x-1][cur_y][layer] += MC_num * demand;
            }
        }
        if(cur_x < _placement->_boundary_width - 1){ // have right side
            map<int, int>& adj_map = _mcell_list[cur_x+1][cur_y];
            for(size_t i = 0 ; i < adjH.size() ; i++){ 
                int MC2_id = adjH[i]._extraId;
                adj_iter = adj_map.find(MC2_id);
                if( adj_iter == adj_map.end() ) continue;
                int MC2_num = adj_iter->second;
                int layer = adjH[i]._layer;
                int demand = adjH[i]._demand;
                int MC_num = min(MC1_num, MC2_num) - min(MC1_num-1, MC2_num);
                // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                // cerr << "DEMAND " << cur_x+1 << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                _demand[cur_x][cur_y][layer] += MC_num * demand;
                _demand[cur_x+1][cur_y][layer] += MC_num * demand;
            }
        }
        // Same Demand
        map<int, int>& cur_map = _mcell_list[cur_x][cur_y];
        for(size_t i = 0 ; i < Same.size() ; i++){  // sameDemand
            int MC2_id = Same[i]._extraId;
            same_iter = cur_map.find(MC2_id);
            if( same_iter == cur_map.end() ) continue;
            int MC2_num = same_iter->second;
            int layer = Same[i]._layer;
            int demand = Same[i]._demand;
            int MC_num = min(MC1_num, MC2_num) - min(MC1_num-1, MC2_num);
            // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
            _demand[cur_x][cur_y][layer] += MC_num * demand;                
        }
    }
    else{
        // update
        int MC1_num = _mcell_list[cur_x][cur_y][cur_mcell_Id];
        vector<ExtraDemand> adjH = _placement->masters[cur_mcell_Id]->get_adjHDemand();
        vector<ExtraDemand> Same = _placement->masters[cur_mcell_Id]->get_sameDemand();
        // adjG Demand
        if(cur_x > 0){ // have left side
            map<int, int>& adj_map = _mcell_list[cur_x-1][cur_y];
            for(size_t i = 0 ; i < adjH.size() ; i++){ 
                int MC2_id = adjH[i]._extraId;
                adj_iter = adj_map.find(MC2_id);
                if( adj_iter == adj_map.end() ) continue;
                int MC2_num = adj_iter->second;
                int layer = adjH[i]._layer;
                int demand = adjH[i]._demand;
                int MC_num = min(MC1_num-1, MC2_num) - min(MC1_num, MC2_num);
                // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                // cerr << "DEMAND " << cur_x-1 << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                _demand[cur_x][cur_y][layer] += MC_num * demand;
                _demand[cur_x-1][cur_y][layer] += MC_num * demand;
            }
        }
        if(cur_x < _placement->_boundary_width - 1){ // have right side
            map<int, int>& adj_map = _mcell_list[cur_x+1][cur_y];
            for(size_t i = 0 ; i < adjH.size() ; i++){ 
                int MC2_id = adjH[i]._extraId;
                adj_iter = adj_map.find(MC2_id);
                if( adj_iter == adj_map.end() ) continue;
                int MC2_num = adj_iter->second;
                int layer = adjH[i]._layer;
                int demand = adjH[i]._demand;
                int MC_num = min(MC1_num-1, MC2_num) - min(MC1_num, MC2_num);
                // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                // cerr << "DEMAND " << cur_x+1 << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
                _demand[cur_x][cur_y][layer] += MC_num * demand;
                _demand[cur_x+1][cur_y][layer] += MC_num * demand;
            }
        }
        // Same Demand
        map<int, int>& cur_map = _mcell_list[cur_x][cur_y];
        for(size_t i = 0 ; i < Same.size() ; i++){  // sameDemand
            int MC2_id = Same[i]._extraId;
            same_iter = cur_map.find(MC2_id);
            if( same_iter == cur_map.end() ) continue;
            int MC2_num = same_iter->second;
            int layer = Same[i]._layer;
            int demand = Same[i]._demand;
            int MC_num = min(MC1_num-1, MC2_num) - min(MC1_num, MC2_num);
            // cerr << "DEMAND " << cur_x << " " << cur_y << " " << layer << " : " << MC_num * demand << endl;
            _demand[cur_x][cur_y][layer] += MC_num * demand;                
        }
        // delete from map
        if(_mcell_list[cur_x][cur_y][cur_mcell_Id] == 1) _mcell_list[cur_x][cur_y].erase(cur_mcell_Id);
        else _mcell_list[cur_x][cur_y][cur_mcell_Id] -= 1;
    }
}

void Placer::print_mcell_list(){
    map<int,int>::iterator iter;
    for(int i = 0 ; i < _placement->_boundary_width ; ++i){
        for(int j = 0 ; j < _placement->_boundary_height ; ++j){
            map<int,int> cur_map = _mcell_list[i][j];
            cout << "[";
            for(int k = 0 ; k < _placement->_numMasterCell ; ++k){
                if(cur_map.count(k)) cout  << cur_map[k] << "  ";
                else cout << '*' << "  ";
            }
                
            cout << "]";
        }
        cout << endl;
    }
}

void Placer::record_cell_place(){
    for(int i = 0 ; i < _placement->_numCells ; ++i){
        _init_cellAry.push_back(*(_placement->_cellArray[i]));
    }
}

double Placer::cal_move_cell_num(){
    int move_num = 0;
    for(int i = 0 ; i < _placement->_numCells ; ++i){
        if(_placement->_cellArray[i]->getx() != _init_cellAry[i].getx() ||
            _placement->_cellArray[i]->gety() != _init_cellAry[i].gety() )
                ++move_num;
    }
    return move_num;
}

void Placer::writeResult(fstream &outFile){
    
    outFile << "NumMovedCellInst " << cal_move_cell_num() << endl;
    for(int i = 0 ; i < _placement->_numCells ; ++i){
        if(_placement->_cellArray[i]->getx() != _init_cellAry[i].getx() ||
            _placement->_cellArray[i]->gety() != _init_cellAry[i].gety() )
                outFile << "CellInst " << _placement->_cellArray[i]->get_name() << " " << _placement->_cellArray[i]->gety() << " " << _placement->_cellArray[i]->getx() << '\n';
    }

}

void Placer::init_fixCell_list(){
    _fixCell_list.reserve(_placement->_numCells);
    for(int i = 0 ; i < _placement->_numCells ; ++i){
        if(_placement->_cellArray[i]->is_movable()) _fixCell_list.push_back(i);
    }
}

void Placer::set_congest_feasible(bool feasible)
{

    if (_congest_feasible_chain.size() > _congest_feasible_chain_size)
    {
        if (_congest_feasible_chain.front() == true)
            --_congest_feasible_count;
        _congest_feasible_chain.pop_front();
    }
    _congest_feasible_chain.push_back(feasible);
    if (feasible)
        ++_congest_feasible_count;
    return;
}

void Placer::set_movecell_feasible(bool feasible)
{

    if (_movecell_feasible_chain.size() > _movecell_feasible_chain_size)
    {
        if (_movecell_feasible_chain.front() == true)
            --_movecell_feasible_count;
        _movecell_feasible_chain.pop_front();
    }
    _movecell_feasible_chain.push_back(feasible);
    if (feasible)
        ++_movecell_feasible_count;
    return;
}