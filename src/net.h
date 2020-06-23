#ifndef NET_H
#define NET_H

#include <vector>
using namespace std;

class Net
{
public:
    // constructor and destructor
    Net(string& name, int numPin, int minLayer) : _name(name), _numPin(numPin), _minLayer(minLayer) {};
    ~Net() {};

    // basic access methods
    string getName()            const { return _name;       }
    vector<Pin*> getPinArray()  const { return _pinArray;   }
    Pin* getPin(int i)              const { return _pinArray[i];}
    int  getPin_num()           const { return _numPin;     }

    // modify methods
    void addPin(Pin* new_pin)  { _pinArray.push_back(new_pin); }

    void print(){
        cout << "Net name: " << getName() << ", min layer " << _minLayer << ", (CellId/PinName,layer): ";
        for(int i = 0 ; i < _numPin ; ++i) _pinArray[i]->print_with_cell();
        cout << '\n';
    }

private:
    string          _name;          // Name of the net
    vector<Pin*>    _pinArray;      // Array of all pins
    int             _numPin;        // Number of pins of the net
    int             _minLayer;      // Minimum routing layer constraint
};

#endif  // NET_H
