#ifndef STRUC_H
#define STRUC_H
#include <iomanip>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
// #include "router.h"

using namespace std;

class Layer
{
    // friend class Router;
public:
    Layer(string name, bool isH, int supply) : _name(name), _isH(isH), _supply(supply) {};
    ~Layer() {};
    // access function
    int get_supply()    {return _supply;}  
    bool isH()          {return _isH;   }
    void print() {cout << "Layer Name " << setw(4) << _name << ", is " << (_isH ? 'H' : 'V') << ", with default supply " << _supply << '\n';};
private:
    string _name;       // layer name
    bool _isH;          // is horizontal
    int _supply;        // default supply of the layer
};

class NonDefault
{
    friend class Router;
public:
    NonDefault(int x, int y, int z, int offset) : _x(x), _y(y), _z(z), _offset(offset) {};
    ~NonDefault() {};

    void print() {cout << "(" << _x << ',' << _y << ',' << _z << "), with supply offset  " << _offset << '\n';};
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
    void print_with_cell() { cout << "(" << _cellId << "/" << _name << "," << _layer << ")";};
    void setcellId(int cellid) {_cellId = cellid;};
    
    int getcellId() {return _cellId;};
    int get_layer() {return _layer;};
    string get_name() {return _name;};

private:
    int _cellId;        // the cell id in the cellArray
    int _layer;         // the layer num of the pin
    string _name;       // pin name

};

class ExtraDemand{
public:
    ExtraDemand(int extraId, int layer, int demand) : _extraId(extraId), _layer(layer), _demand(demand) {};
    ~ExtraDemand() {};
    int _extraId;       // its counterpart
    int _layer;         // extra demand layer
    int _demand;        // extra demand value
};


class Bend{
public:
    Bend() {}
    Bend( int x , int y ,int z , Bend* p = NULL , Bend* n = NULL):_x(x),_y(y),_z(z),prev_bend(p),next_bend(n){}
    void set_prev( Bend* p ) { prev_bend = p ;}
    void set_next( Bend* n ) { next_bend = n ;}
    Bend* get_prev() const { return prev_bend;}
    Bend* get_next() const { return next_bend;}
    void print() { cout<<"x is: "<<setw(5)<< _x <<" y is: "<<setw(5)<< _y <<setw(5)<<" z is "<<setw(5)<< _z <<endl;}
    int _x ;
    int _y ; 
    int _z ;
    Bend* prev_bend;
    Bend* next_bend;
    // void operator = (Coordinate a){ _x = a.x ; _y = a.y ; _z = a.z; }
};
 
#endif // STRUC_H