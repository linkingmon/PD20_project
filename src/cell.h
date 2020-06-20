#ifndef CELL_H
#define CELL_H

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include "struc.h"
using namespace std;


class Master
{
    friend class Cell;
public:
    // constructor and destructor
    Master(string name, int numPin, int numBlockage, int id) : _name(name), _numPin(numPin), _numBlockage(numBlockage) , _id(id){};
    ~Master() {};

    void add_pin(string name, int layer) {_PinName2Id[name] = _pinArray.size(); _pinArray.push_back(new Pin(name, layer));};
    void add_blockage(int layer, int demand) {_BlockLayers.push_back(layer); _BlockDemand.push_back(demand);};
    void add_adj_demand(int extraId, int layer, int demand) {_adjDemand.push_back(ExtraDemand(extraId, layer, demand));};
    void add_same_demand(int extraId, int layer, int demand) {_sameDemand.push_back(ExtraDemand(extraId, layer, demand));};
    
    // access function
    const vector<ExtraDemand>& get_adjHDemand() { return  _adjDemand;}
    const vector<ExtraDemand>& get_sameDemand() { return  _sameDemand;}
    int getId() { return _id;}

    void print() {
        cout << "Master cell name: " << _name << endl;
        cout << "> Num of pins: " << setw(11) << _numPin << " (name, layer): ";
        for(int i = 0 ; i < _numPin ; ++i)
            _pinArray[i]->print();
        cout << "\n> Num of blocks: " << setw(9) << _numBlockage << " (layer, demand): ";
        for(int i = 0 ; i < _numBlockage ; ++i)
            cout << "(" << _BlockLayers[i] << "," << _BlockDemand[i] << ")";
        cout << "\n> Num of Adj demands: " << setw(4) << _adjDemand.size() << " (MSId, layer, demand): ";
        for(int i = 0, end_i = _adjDemand.size() ; i < end_i ; ++i)
            cout << "(" << _adjDemand[i]._extraId << "," << _adjDemand[i]._layer << "," << _adjDemand[i]._demand << ")";
        cout << "\n> Num of Same demands: " << setw(3) << _sameDemand.size() << " (MSId, layer, demand): ";
        for(int i = 0, end_i = _sameDemand.size() ; i < end_i ; ++i)
            cout << "(" << _sameDemand[i]._extraId << "," << _sameDemand[i]._layer << "," << _sameDemand[i]._demand << ")";
        cout << '\n';
    }
    void set_pin_id(int cell_id) {
        for(int i = 0, end_i = _pinArray.size() ; i < end_i ; ++i)
            _pinArray[i]->setcellId(cell_id);
    }
    Pin* getPin(string pin_name) {return _pinArray[_PinName2Id[pin_name]];}
    Master* copy() {
        Master* cpy = new Master(*this); 
        for(int i = 0, end_i = _pinArray.size() ; i < end_i ; ++i)
            cpy->_pinArray[i] = new Pin(_pinArray[i]->get_name(), _pinArray[i]->get_layer());
        return cpy;
    }
    vector<Pin*> get_pinArray(){ return _pinArray;}
    int get_blockage() {return _numBlockage;};
    int get_block_layer(int idx) {return _BlockLayers[idx];}
    int get_block_demand(int idx) {return _BlockDemand[idx];}
    string get_name() {return _name;};

private:
    int _id;                             
    string _name;                       // Name of the master cell (MC)
    int _numPin;                        // Number of pin of this MC
    int _numBlockage;                   // Number of blockage of this MC
    vector<Pin*> _pinArray;             // Pin array of the MC
    vector<int> _BlockLayers;           // the layer num of each block
    vector<int> _BlockDemand;           // the demand of each block
    vector<ExtraDemand> _adjDemand;     // adjacencyH grid extra demand
    vector<ExtraDemand> _sameDemand;    // same grid extra demand
    map<string, int> _PinName2Id;   
};

class Cell
{
public:
    // Constructor and destructor
    Cell(string cellname, Master* type, int y, int x, bool isM, int cell_id) : _cellname(cellname), _x(x), _y(y), _isMovable(isM){
        _type = type->copy();
        _type->set_pin_id(cell_id);
    }
    ~Cell() { }
    void print() {
        cout << "Cell name: " << _cellname << " at (" << _x << "," << _y << ")" << " with MS " << _type->_name << " and " << (_isMovable ? "Movable" : "Fixed") << '\n';
    }
    Pin* getPin(string pin_name) {return _type->getPin(pin_name);};

    int getx() {return _x;};
    int gety() {return _y;};

    int setx(int x) {_x = x;};
    int sety(int y) {_y = y;};
    Master* get_master() {return _type;};

    bool is_movable() {return _isMovable;};
private:
    string _cellname;       // cell name
    int _x;                 // cell coordinate x
    int _y;                 // cell coordinate y
    bool _isMovable;        // whether the cell is movable
    Master* _type;          // the master cell type of this cell
};

#endif  // CELL_H
