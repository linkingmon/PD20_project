#ifndef MST_H
#define MST_H
#include "struc.h"
#include "placement.h"
#include <map>

using namespace std;

typedef pair<Pin*, Pin*> EDGE; // <a, b> = <b, a>

enum direction{
    R1,
    R2,
    R3,
    R4
};

class MST{
    friend class Placer;
    // for four direction linked list
    struct Node{
        Node* _next;
        Pin* _pin;
    };

    struct Direction{
        Node* R1;
        Node* R2;
        Node* R3;
        Node* R4;
    };
public:
    MST(const vector<Pin*>& pin_ary, Placement* placement);
    void init_direction(vector<Pin*> pin_ary, Placement* placement);
    void init_weight(const vector<Pin*>& pin_ary, Placement* placement);
    void construct2pins();
    vector<EDGE> get2pinnets();
    void update(Pin* pin, const vector<Pin*>& pin_ary, Placement* placement);
private:
    // if the pin array is smaller than 3, then only run basic version !!!

    // weights[minmax(pin1, pin2)] = x
    // the pin memory address will be modified if the best solution is recovered 
    // EDGE getedge();
    multimap<int, EDGE> _weight2edge;
    map<EDGE, int> _edge2weight;
    int _color;

    Node* _R1_head;
    Node* _R2_head;
    Node* _R3_head;
    Node* _R4_head;
    vector<Direction> _direction; // 1-1 to the pins
    int _numPins;
    vector<EDGE> _two_pin_nets;

    // color the original chosen edges, if the edges is chosen again, no update

    // make set related
    void make_set();
    Pin* find_set(Pin*);
    void unize();
};

class PinCompare {
public:
    PinCompare(direction type, Placement* placement) : _type(type), _placement(placement) {};
    bool operator() (Pin* p1, Pin* p2);
private:
    direction _type;
    Placement* _placement; // only temperery storage

};

#endif