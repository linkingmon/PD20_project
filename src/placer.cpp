#include "placer.h"

Placer::Placer(Placement * placement)  : _placement(placement) {
    // _supply = vector<vector<double> >(_placement->_boundary_width, vector<double>(_placement->_boundary_height));
    // _demand = vector<vector<double> >(_placement->_boundary_width, vector<double>(_placement->_boundary_height));
    // _congestion = vector<vector<double> >(_placement->_boundary_width, vector<double>(_placement->_boundary_height));
}

void Placer::place(){
    cout << "Placing ..." << endl;
    // int iteration = 40;
    // int dw = 1;
    // for(int i = 0 ; i < iteration ; ++i){
    //     bd_congestion_init(dw);
    //     for(int j = 0, end_j = _placement->_numNets ; j < end_j ; ++j)
    //         cal_bd_congestion(_placement->_netArray[j]);

    // }
    for(int i = 0 ; i < _placement->_numNets ; ++i)
        cout << "Net " << i << " HPWL: " << cal_net_cost(_placement->_netArray[i]) << endl;
}

void Placer::cal_congestion_force(){

}

void Placer::cal_wirelen_force(){

}

void Placer::cal_extrademand_force(){

}

double Placer::cal_bd_congestion(Net* cur_net){

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
    int min_x = 2147483647;
    int min_y = 2147483647;
    for(int i = 0, end_i = pinarray.size() ; i < end_i ; ++i){
        Cell* cur_cell = _placement->_cellArray[pinarray[i]->getcellId()];
        if(max_x < cur_cell->getx()) max_x = cur_cell->getx();
        if(min_x > cur_cell->getx()) min_x = cur_cell->getx();
        if(max_y < cur_cell->gety()) max_y = cur_cell->gety();
        if(min_y > cur_cell->gety()) min_y = cur_cell->gety();
    }
    return (max_x - min_x) + (max_y - min_y);
}