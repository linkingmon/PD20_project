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
    // initialize the congection map
    init_supply_map();
    init_demand_map();
}

void Placer::place(){
    set_range();
    double wire;
    // keep the initial placement
    keep_best_solution();
    // calculate the normalize factor of congestion and wirelength (highly congested condition will not be present in random floorplan with high probalbility)
    calculate_norm_factor();
    recover_best_solution();
    _best_cost = calculate_total_cost(_best_congestion, _best_wire);

    // specify prameters
    int k = 10;
    double P = 0.95;
    N = _placement->_numCells * k;
    int c = 10000 ; // a small number for stage 2 in fast SA
    // random floorplanning for calculate avg cost
    double avg_cost = random_place(N);
    T1 = avg_cost / log(1 / P); // set initial temperature
    double terminate_temperature = 1e-3;
    int num_stage_1 = 2; 
    int num_stage_2 = num_stage_1 + _placement->_numCells;

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
            if (cur_cost < _best_cost)
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
    // re calculate the congestion demand map
    init_demand_map();
}

void Placer::perturb(){
    _perturb_type = rand_01() < 0.8 ? MOVE : SWAP;
    _perturb_val1 = rand() % (_placement->_numCells);
    _perturb_val2 = rand() % (_placement->_numCells);

    _temp_x = _placement->_cellArray[_perturb_val1]->getx();
    _temp_y = _placement->_cellArray[_perturb_val1]->gety();

    // cout << "TAKE CELL: " << _perturb_val1 << " " << _perturb_val2 << '\n';
    // cout << "TAKE TYPE: " << _perturb_type << endl;
    // cout << "CELL 1 COORD: " << _temp_x << " " << _temp_y << '\n';
    // cout << "CELL 2 COORD: " << _placement->_cellArray[_perturb_val2]->getx() << " " << _placement->_cellArray[_perturb_val2]->gety() << '\n';

    switch(_perturb_type){
        case MOVE:{
            // for incremental update
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            // wire length inc.
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) _netlength[pin_ary1[i]->getNetId()] = -1;
            // record modify nets
            set<int> modify_netIds;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) modify_netIds.insert(pin_ary1[i]->getNetId());
            // remove net congestion before update 2 pin nets
            congestion_incremental_update(modify_netIds, false);
            // MST inc.
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            // ass net congestion after update 2 pin nets
            congestion_incremental_update(modify_netIds, true);

            // set moving range
            int cur_x = _placement->_cellArray[_perturb_val1]->getx();
            cur_x = cur_x + rand() % x_range - x_range / 2;
            cur_x = cur_x < _placement->_leftBoundary ? _placement->_leftBoundary : cur_x;
            cur_x = cur_x > _placement->_rightBoundary ? _placement->_rightBoundary : cur_x;

            int cur_y = _placement->_cellArray[_perturb_val1]->gety();
            cur_y = cur_y + rand() % y_range - y_range / 2;
            cur_y = cur_y < _placement->_bottomBoundary ? _placement->_bottomBoundary : cur_y;
            cur_y = cur_y > _placement->_topBoundary ? _placement->_topBoundary : cur_y;

            _placement->_cellArray[_perturb_val1]->setx(cur_x);
            _placement->_cellArray[_perturb_val1]->sety(cur_y);

            }
            break;
        case SWAP:{
            // for incremental
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds1, true);

            vector<Pin*> pin_ary2 = _placement->_cellArray[_perturb_val2]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) _netlength[pin_ary2[i]->getNetId()] = -1;
            set<int> modify_netIds2;
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) modify_netIds2.insert(pin_ary2[i]->getNetId());
            congestion_incremental_update(modify_netIds2, false);
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) 
                _msts[pin_ary2[i]->getNetId()]->update(pin_ary2[i], _placement->_netArray[pin_ary2[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds2, true);

            _placement->_cellArray[_perturb_val1]->setx(_placement->_cellArray[_perturb_val2]->getx());
            _placement->_cellArray[_perturb_val1]->sety(_placement->_cellArray[_perturb_val2]->gety());
            _placement->_cellArray[_perturb_val2]->setx(_temp_x);
            _placement->_cellArray[_perturb_val2]->sety(_temp_y);
            }
            break;
    }
}

void Placer::deperturb(){

    // cout << "D TAKE CELL: " << _perturb_val1 << " " << _perturb_val2 << '\n';
    // cout << "D TAKE TYPE: " << _perturb_type << endl;
    // cout << "D CELL 1 COORD: " << _placement->_cellArray[_perturb_val1]->getx() << " " << _placement->_cellArray[_perturb_val1]->getx() << '\n';
    // cout << "D CELL 2 COORD: " << _placement->_cellArray[_perturb_val2]->getx() << " " << _placement->_cellArray[_perturb_val2]->gety() << '\n';

    switch(_perturb_type){
        case MOVE:{
            // for incremental
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds1, true);

            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);
            }
            break;
        case SWAP:{
            // for incremental
            vector<Pin*> pin_ary1 = _placement->_cellArray[_perturb_val1]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) _netlength[pin_ary1[i]->getNetId()] = -1;
            set<int> modify_netIds1;
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) modify_netIds1.insert(pin_ary1[i]->getNetId());
            congestion_incremental_update(modify_netIds1, false);
            for(int i = 0, end_i = pin_ary1.size() ; i < end_i ; ++i) 
                _msts[pin_ary1[i]->getNetId()]->update(pin_ary1[i], _placement->_netArray[pin_ary1[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds1, true);

            vector<Pin*> pin_ary2 = _placement->_cellArray[_perturb_val2]->get_master()->get_pinArray();
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) _netlength[pin_ary2[i]->getNetId()] = -1;
            set<int> modify_netIds2;
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) modify_netIds2.insert(pin_ary2[i]->getNetId());
            congestion_incremental_update(modify_netIds2, false);
            for(int i = 0, end_i = pin_ary2.size() ; i < end_i ; ++i) 
                _msts[pin_ary2[i]->getNetId()]->update(pin_ary2[i], _placement->_netArray[pin_ary2[i]->getNetId()]->getPinArray(),_placement);
            congestion_incremental_update(modify_netIds2, true);

            _placement->_cellArray[_perturb_val2]->setx(_placement->_cellArray[_perturb_val1]->getx());
            _placement->_cellArray[_perturb_val2]->sety(_placement->_cellArray[_perturb_val1]->gety());
            _placement->_cellArray[_perturb_val1]->setx(_temp_x);
            _placement->_cellArray[_perturb_val1]->sety(_temp_y);
            }
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
    _congestion_norm_factor = 1/_congestion_norm_factor;
    _wire_length_norm_factor = 1/_wire_length_norm_factor;
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
    cost_ratio = 0.5;
    // cerr << "IIIIIIIIIIIIIIII " << wire * _wire_length_norm_factor<< " " <<congestion * _congestion_norm_factor<< " " << wire * (1 - cost_ratio) * _wire_length_norm_factor + congestion * cost_ratio * _congestion_norm_factor<<endl;
        return wire * (1 - cost_ratio) * _wire_length_norm_factor + congestion * cost_ratio * _congestion_norm_factor;
}

double Placer::calculate_total_cost(double& congestion, double& wire){
    calculate_congestion_cost(congestion);
    calculate_wire_length_cost(wire);
    // calculate cost
    _beta = 0;
    double cost_ratio = 0.5 + (1 - _beta);
    cost_ratio = cost_ratio > 0.9 ? 0.9 : cost_ratio;
    cost_ratio = 0.5;
    // cerr << "IIIIIIIIIIIIIIII " << wire * _wire_length_norm_factor<< " " <<congestion * _congestion_norm_factor<< " " << wire * (1 - cost_ratio) * _wire_length_norm_factor + congestion * cost_ratio * _congestion_norm_factor<<endl;
    return wire * (1 - cost_ratio) * _wire_length_norm_factor + congestion * cost_ratio * _congestion_norm_factor;
}

void Placer::calculate_congestion_cost(double& congestion){
    // add all the overflow grid values
    congestion = 0;
    for(int i = 0 ; i < _placement->_boundary_width ; ++i){
        for(int j = 0 ; j < _placement->_boundary_height ; ++j){
            for(int k = 0 ; k < _placement->_numLayers ; ++k){
                if(_demand[i][j][k] > _supply[i][j][k])
                    congestion += _demand[i][j][k] - _supply[i][j][k];
            }
        }
    }
}

void Placer::calculate_wire_length_cost(double& wire){
    // cerr << "CAL WIRE 1" << endl;
    wire = 0;
    for(int i = 0, end_i = _placement->_numNets ; i < end_i ; ++i){
        if(_netlength[i] == -1) {
            int wire_len = 0;
            wire_len = cal_net_cost(_placement->_netArray[i]); 
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
                wire_len += abs(c1->getx() - c2->getx()) + abs(c1->gety() - c2->gety() + abs(p1->get_layer() - p2->get_layer()));
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
    _supply = vector<vector<vector<int> > >(_placement->_boundary_width, vector<vector<int> >(_placement->_boundary_height, vector<int>(_placement->_numLayers)));
    for(int i = 0 ; i < _placement->_numLayers ; ++i){ // z
        int cur_layer_supply = _placement->layers[i]->get_supply();
        for(int j = 0 ; j < _placement->_boundary_width ; ++j){ // x
            for(int k = 0 ; k < _placement->_boundary_height ; ++k){ // y
                _supply[j][k][i] = cur_layer_supply;
            }
        }
    }
    // cerr << "INIT SUPPLY " << _placement->_numNonDefault << endl;
    // non default supply
    for(int i = 0; i < _placement->_numNonDefault ; ++i){
        NonDefault* cur_grid = _placement->nondefault[i];
        // cur_grid->print();
        _supply[cur_grid->getx()-_placement->_leftBoundary][cur_grid->gety()-_placement->_bottomBoundary][cur_grid->getz()] = cur_grid->get_offset();
    }
    // cerr << "INIT SUPPLY DONE" << endl;
    return;
}

void Placer::init_demand_map(){
    // cerr << "INIT DEMAND" << endl;
    _demand = vector<vector<vector<int> > >(_placement->_boundary_width, vector<vector<int> >(_placement->_boundary_height, vector<int>(_placement->_numLayers, 0)));
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
    // cerr << "INIT DEMAND DONE" << endl;
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
                if(inc) _supply[i][j][k] += exp(log_prob);
                else _supply[i][j][k] -= exp(log_prob);
                // cerr << "FILL " << exp(log_prob) << endl;
            }
        }
    }
}

void Placer::congestion_incremental_update(set<int> netIds, bool inc){
    set<int>::iterator iter;
    for(iter = netIds.begin() ; iter != netIds.end() ; ++iter){
        vector<EDGE> two_pin_net = _msts[*iter]->get2pinnets();
        for(int j = 0, end_j = two_pin_net.size() ; j < end_j ; ++j){
            Pin* p1 = two_pin_net[j].first;
            Pin* p2 = two_pin_net[j].second;
            Cell* c1 = _placement->_cellArray[p1->getcellId()];
            Cell* c2 = _placement->_cellArray[p2->getcellId()];
            fill_demand(c1->getx()-_placement->_leftBoundary, c1->gety()-_placement->_bottomBoundary, p1->get_layer(),
                    c2->getx()-_placement->_leftBoundary, c2->gety()-_placement->_bottomBoundary, p2->get_layer(), inc);
        }
    }
}
