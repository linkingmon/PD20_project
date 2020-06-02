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
private:
    string _name;
    bool _isH;
    int _supply;
};

class NonDefault
{
public:
    NonDefault(int x, int y, int z, int offset) : _x(x), _y(y), _z(z), _offset(offset) {};
    ~NonDefault() {};

    void print() {cout << "(" << _x << ',' << _y << ',' << _z << "), with supply offset  " << _offset << '\n';};
private:
    int _x, _y, _z;
    int _offset;

};

class Pin
{
public:
    // constructor and destructor
    Pin(string name, int layer) : _name(name), _layer(layer){};
    ~Pin() {};
    void print() { cout << "(" << _name << "," << _layer << ")";};
    void setcellId(int cellid) {_cellId = cellid;};

private:
    int _cellId;
    int _layer;
    string _name;

};

class ExtraDemand{
public:
    ExtraDemand(int extraId, int layer, int demand) : _extraId(extraId), _layer(layer), _demand(demand) {};
    ~ExtraDemand() {};
    int _extraId;
    int _layer;
    int _demand;    
};


#endif // STRUC_H