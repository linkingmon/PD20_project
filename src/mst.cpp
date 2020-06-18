#include "mst.h"
#include <cmath>
#include <algorithm>

bool PinCompare::operator() (Pin* p1, Pin* p2){
    Cell* cell1 = _placement->_cellArray[p1->getcellId()];
    Cell* cell2 = _placement->_cellArray[p2->getcellId()];
    switch(_type){
        case direction::R1:
            return cell1->getx() + cell1->gety() < cell2->getx() + cell2->gety();
        case direction::R2:
            return cell1->getx() + cell1->gety() < cell2->getx() + cell2->gety();
        case direction::R3:
            return cell1->getx() + cell1->gety() < cell2->getx() + cell2->gety();
        case direction::R4:
            return cell1->getx() + cell1->gety() < cell2->getx() + cell2->gety();

    }
}


MST::MST(const vector<Pin*>& pin_ary, Placement* placement) : _color(0) {
    init_direction(pin_ary, placement);
    init_weight(pin_ary, placement);
}

void MST::init_direction(vector<Pin*> pin_ary, Placement* placement){
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R1, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R2, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R3, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R4, placement));
}

void MST::init_weight(const vector<Pin*>& pin_ary, Placement* placement){
    // clear the map
    _weight2edge.clear();
    _edge2weight.clear();
    // quadratic runtime but only run once
    for(int i = 0, end_i = pin_ary.size() ; i < end_i ; ++i){
        Cell* cur_cell = placement->_cellArray[pin_ary[i]->getcellId()];
        for(int j = i+1 ; j < end_i ; ++j){
            Cell* other_cell = placement->_cellArray[pin_ary[j]->getcellId()];
            EDGE cur_edge = pair<Pin*, Pin*>(minmax(pin_ary[i],pin_ary[j])); // order the edge by minmax
            int cur_wire = abs(cur_cell->getx() - other_cell->getx()) + abs(cur_cell->gety() - other_cell->gety());
            _weight2edge.insert(pair<int,EDGE>(cur_wire, cur_edge));
            _edge2weight.insert(pair<EDGE,int>(cur_edge, cur_wire));
        } 
    }
    return;
}

vector<EDGE> get2pinnets(){

}