#ifndef STRUC_H
#define STRUC_H
#include <iomanip>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
using namespace std;

class Layer
{
public:
    Layer(string name, bool isH, int supply) : _name(name), _isH(isH), _supply(supply) {};
    ~Layer() {};

    void print() {cout << "Layer Name " << setw(4) << _name << ", is " << (_isH ? 'H' : 'V') << ", with default supply " << _supply << '\n';};
    int get_supply() {return _supply;};
private:
    string _name;       // layer name
    bool _isH;          // is horizontal
    int _supply;        // default supply of the layer
};

class NonDefault
{
public:
    NonDefault(int x, int y, int z, int offset) : _x(x), _y(y), _z(z), _offset(offset) {};
    ~NonDefault() {};

    void print() {cout << "(" << _x << ',' << _y << ',' << _z << "), with supply offset  " << _offset << '\n';};
    
    int getx() {return _x;};
    int gety() {return _y;};
    int getz() {return _z;};
    int get_offset() {return _offset;};
private:
    int _x, _y, _z;     // coordinate of non default supply grid
    int _offset;        // non default supply offset

};

class Pin
{
public:
    // constructor and destructor
    Pin(string name, int layer) : _name(name), _layer(layer){};
    ~Pin() {};
    void print() { cout << "(" << _name << "," << _layer << ")";};
    void print_with_cell() { cout << "(" << _cellId << "/" << _name << "," << _layer << ")onNet" << _netId;};
    void setcellId(int cellid) {_cellId = cellid;};
    
    int getcellId() {return _cellId;};
    int get_layer() {return _layer;};
    string get_name() {return _name;};
    void addNetId(int Id) {_netId = Id;}
    int getNetId() {return _netId;}

private:
    int _cellId;        // the cell id in the cellArray
    int _layer;         // the layer num of the pin
    string _name;       // pin name
    int _netId;
};

class ExtraDemand{
public:
    ExtraDemand(int extraId, int layer, int demand) : _extraId(extraId), _layer(layer), _demand(demand) {};
    ~ExtraDemand() {};
    int _extraId;       // its counterpart
    int _layer;         // extra demand layer
    int _demand;        // extra demand value
};


#endif // STRUC_H