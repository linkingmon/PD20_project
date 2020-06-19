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
    // init_direction(pin_ary, placement);
    init_weight(pin_ary, placement);
    _numPins = pin_ary.size();
    _two_pin_nets.reserve(_numPins);
    construct2pins();
}

void MST::init_direction(vector<Pin*> pin_ary, Placement* placement){
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R1, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R2, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R3, placement));
    sort(pin_ary.begin(), pin_ary.end(), PinCompare(direction::R4, placement));
}

void MST::init_weight(const vector<Pin*>& pin_ary, Placement* placement){
    // clear the map
    // cerr << "INIT" << endl;
    _weight2edge.clear();
    _edge2weight.clear();
    // quadratic runtime but only run once
    for(int i = 0, end_i = pin_ary.size() ; i < end_i ; ++i){
        Cell* cur_cell = placement->_cellArray[pin_ary[i]->getcellId()];
        for(int j = i+1 ; j < end_i ; ++j){
            Cell* other_cell = placement->_cellArray[pin_ary[j]->getcellId()];
            EDGE cur_edge = pair<Pin*, Pin*>(minmax(pin_ary[i],pin_ary[j])); // order the edge by minmax
            int cur_wire = abs(cur_cell->getx() - other_cell->getx()) + abs(cur_cell->gety() - other_cell->gety()) + abs(pin_ary[i]->get_layer() - pin_ary[j]->get_layer());
            // cerr << "W2E INSERT: " << cur_wire << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() 
            //     << "," << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ")" << endl;
            _weight2edge.insert(pair<int,EDGE>(cur_wire, cur_edge));
            // cerr << "E2W INSERT:" << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() << "," 
            //     << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ") "<< cur_wire << endl;
            _edge2weight.insert(pair<EDGE,int>(cur_edge, cur_wire));
        } 
    }
    // cerr << "INIT DONE" << endl;
    return;
}

vector<EDGE> MST::get2pinnets(){
    return _two_pin_nets;
}

void MST::update(Pin* pin, const vector<Pin*>& pin_ary, Placement* placement){
    // cerr << "UPDATE: " << pin->getcellId() << "/" << pin->get_name() << " IN NET " << pin->getNetId() << endl;
    // the color is not yet construct
    pair<multimap<int,EDGE>::iterator, multimap<int,EDGE>::iterator> res;
    multimap<int, EDGE>::iterator iter;
    for(int i = 0, end_i = pin_ary.size() ; i < end_i ; ++i){
        Cell* cur_cell = placement->_cellArray[pin->getcellId()];
        if(pin != pin_ary[i]){
            // erase the map
            EDGE cur_edge = EDGE(minmax(pin, pin_ary[i]));
            int cur_weight = _edge2weight[cur_edge];
            // cerr << "E2W ERASE" << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() 
            //     << "," << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ")" << cur_weight << endl;
            _edge2weight.erase(cur_edge);
            res = _weight2edge.equal_range(cur_weight);
            for(iter = res.first ; iter != res.second ; ++iter){
                    // cerr << "   CHECK " << iter->first << " (" << iter->second.first->getcellId() << "/" << iter->second.first->get_name() 
                    //     << "," << iter->second.second->getcellId() << "/" << iter->second.second->get_name() << ")" << endl;
                if(iter->second == cur_edge){
                    // cerr << "W2E ERASE " << cur_weight << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() 
                    //     << "," << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ")" << endl;
                    _weight2edge.erase(iter);
                    break;
                }
            }
            // insert the map
            Cell* other_cell = placement->_cellArray[pin_ary[i]->getcellId()];
            int cur_wire = abs(cur_cell->getx() - other_cell->getx()) + abs(cur_cell->gety() - other_cell->gety());
            // cerr << "W2E INSERT: " << cur_wire << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() 
            //     << "," << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ")" << endl;
            // cerr << "E2W INSERT:" << " (" << cur_edge.first->getcellId() << "/" << cur_edge.first->get_name() << "," 
            //     << cur_edge.second->getcellId() << "/" << cur_edge.second->get_name() << ") "<< cur_wire << endl;
            _weight2edge.insert(pair<int,EDGE>(cur_wire, cur_edge));
            _edge2weight.insert(pair<EDGE,int>(cur_edge, cur_wire));
        }
    }
    construct2pins();
    // cerr << "UPDATE DONE" << endl;
    return;
}

void MST::construct2pins(){
    _two_pin_nets.clear();
    multimap<int, EDGE>::iterator iter;
    // make_sets
    for(iter = _weight2edge.begin() ; iter != _weight2edge.end() ; ++iter){
        // if(find_set(iter->second.first) != find_set(iter->second.second)){ // find_set
        //     // union_set
            _two_pin_nets.push_back(iter->second);
        // }
    }
}

void MST::make_set(){

}

Pin* MST::find_set(Pin* pin){

}

void MST::unize(){

}