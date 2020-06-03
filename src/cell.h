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
    Master(string name, int numPin, int numBlockage) : _name(name), _numPin(numPin), _numBlockage(numBlockage) {};
    ~Master() {};

    void add_pin(string name, int layer) {_PinName2Id[name] = _pinArray.size(); _pinArray.push_back(new Pin(name, layer));};
    void add_blockage(int layer, int demand) {_BlockLayers.push_back(layer); _BlockDemand.push_back(demand);};
    void add_adj_demand(int extraId, int layer, int demand) {_adjDemand.push_back(ExtraDemand(extraId, layer, demand));};
    void add_same_demand(int extraId, int layer, int demand) {_sameDemand.push_back(ExtraDemand(extraId, layer, demand));};

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

private:
    string _name;
    int _numPin;
    int _numBlockage;
    vector<Pin*> _pinArray;
    vector<int> _BlockLayers;
    vector<int> _BlockDemand;
    vector<ExtraDemand> _adjDemand;
    vector<ExtraDemand> _sameDemand;
    map<string, int> _PinName2Id;
};

class Cell
{
public:
    // Constructor and destructor
    Cell(string cellname, Master type, int y, int x, bool isM, int cell_id) : _cellname(cellname), _type(type), _x(x), _y(y), _isMovable(isM){
        type.set_pin_id(cell_id);
    }
    ~Cell() { }
    void print() {
        cout << "Cell name: " << _cellname << " at (" << _x << "," << _y << ")" << " with MS " << _type._name << " and " << (_isMovable ? "Movable" : "Fixed") << '\n';
    }
    Pin* getPin(string pin_name) {return _type.getPin(pin_name);};
private:
    string _cellname;
    int _x;
    int _y;
    bool _isMovable;
    Master _type;
};

#endif  // CELL_H
