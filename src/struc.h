#ifndef STRUC_H
#define STRUC_H
#include <iomanip>
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <cstdlib>
// #include "router.h"

using namespace std;

enum Via_Direction :int{
    Pos_x,
    Neg_x,
    Pos_y,
    Neg_y,
    Pos_z,
    Neg_z,
    End_point,
};

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
    Pin(string name, int layer) : _name(name), _layer(layer), _netId(-1) {};
    ~Pin() {};
    void print() { cout << "(" << _name << "," << _layer << ")";};
    void print_with_cell() { cout << "(" << _cellId << "/" << _name << "," << _layer << ")onNet" << _netId;};
    void setcellId(int cellid) {_cellId = cellid;};
    
    int getcellId() {return _cellId;};
    int get_layer() {return _layer;};
    string get_name() {return _name;};
    void addNetId(int Id) {_netId = Id;}
    int getNetId() {return _netId;}

    void set_deputy(Pin* deputy) {_deputy = deputy;}
    Pin* get_deputy() {return _deputy;}

private:
    int _cellId;        // the cell id in the cellArray
    int _layer;         // the layer num of the pin
    string _name;       // pin name
    int _netId;
    Pin* _deputy;       // the deputy for Kruskals MST
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
 
// template<class data>
class branch{
public:
    branch() {}
    branch(int x, int y, int z ,int id= 0, int n = 0):_x(x),_y(y),_z(z),_id(id),_n(n){}
    int _x;     //row index 
    int _y;     //column index
    int _z;     //layer index
    int  _n;     //neighbor
    int  _id;    //index
    int dist_to_pin;    //distance to the nearest pin (segment number)
    bool operator == (branch a) {return (_x == a._x && _y == a._y); }
    friend ostream& operator<<(ostream& os, const branch& a){ os<< "("<< a._x <<","<<a._y<<","<<a._z<<")   index: "<<a._id<<" neighbor: "<<a._n; return os;}
};

// template<class data>
class two_pin_net
{
    friend class Router;
public:
    two_pin_net(branch* a, branch* b) : b_source(a), b_target(b){ segment_len = 0;}
    two_pin_net() {}
    ~two_pin_net(){}
    Bend* get_source() { return source;};
    void set_source( Bend* root ) { source = root;}
    int segment_length() { if(segment_len == 0 ) compute_seglen(); return segment_len;}
    void compute_seglen() {
        segment_len = -1;
        Bend* temp = source ; 
        while(temp != NULL){
            temp = temp->get_next();
            ++segment_len;
        }
    }
    int length(){
        Bend* temp = source ; 
        Bend* n_bend = source->get_next();
        int l = 0;
        while(n_bend != NULL){
            int last_x = temp->_x;
            int last_y = temp->_y;
            int cur_x = n_bend->_x;
            int cur_y = n_bend->_y;
            if(cur_x == last_x){
                int cur_length = (cur_y - last_y);
                l += abs(cur_length);
            }
            else{
                int cur_length = (cur_x - last_x);
                l += abs(cur_length);    
            }
            temp = n_bend;
            n_bend = n_bend->get_next();
        }
        return l;
    }
    void print_bend() { 
        Bend* temp = source ; 
        while(temp != NULL){
            temp->print();
            temp = temp->get_next();
        }
    }
private:
    branch* b_source;
    branch* b_target;
    Bend* source; //staring point
    int segment_len;

};

class Grid
{
public:
    Grid() {}
    Grid(int x , int y, int num_master):_x(x),_y(y){
        // master_list.resize(num_master);
    }
    ~Grid(){}

    int getx()  {return _x;}
    int gety()  {return _y;}
    void add_Cell( int cell_id)     { cell_list.push_back(cell_id) ;}
    void add_MCell( int MCell_id ) { 
        map<int,int>::iterator iter = master_list.find(MCell_id);
        if(iter == master_list.end()){
            master_list.insert(pair<int,int>(MCell_id,1) );
        }   
        else 
            ++(iter->second);
    }
    map<int,int> get_MCell_list() const {return master_list; }
    // int& getMCell( int MCell_id) { return master_list[MCell_id] ;}
private:
    int   _x ;      //normalized x
    int   _y ;      //normalized y
    vector<int> cell_list;      //cell index list
    int cell_num;               //number of cells
    // vector<int> master_list;    //the number of each matser cell
    map<int,int> master_list;   //map from master cell index to its number 
};

class Via_Grid_cost{
public:
    Via_Grid_cost(){}
    Via_Grid_cost(double* v , int l, int index, bool available):value(v), layer(l), idx(index), avail(available), visit(false){}
    ~Via_Grid_cost(){}
    double* value;
    int  layer;  // only z index
    int  idx;
    bool visit;
    bool avail;
    void print() { cout<<"layer: "<<layer<<" index: "<<idx<<" cost value: "<< *value <<endl; }
};

//big to little
class Via_Grid_bigger_comp{
public:
    Via_Grid_bigger_comp(){}
    bool operator() (Via_Grid_cost a, Via_Grid_cost b) const {return *(a.value) > *(b.value);}
};

class Segment{
public: 
    Segment( Bend* s, Bend* t) :source(s),target(t) {}
    Bend* source;
    Bend* target;
    int distance;
    void print() {source->print();target->print();}
};
#endif // STRUC_H