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
    Pin* getPin(int i)          const { return _pinArray[i];}
    int  getPin_num()           const { return _numPin;     }
    int  getMinLayer()                { return _minLayer;   }
    int  get_2D_len()                 { return _2D_len;     }

    // modify methods
    void addPin(Pin* new_pin)   { _pinArray.push_back(new_pin); }
    void set_2D_len(int len)    {   _2D_len = len;             }

    void print(){
        cout << "Net name: " << getName() << ", min layer " << _minLayer << ", (CellId/PinName,layer): ";
        for(int i = 0 ; i < _numPin ; ++i) _pinArray[i]->print_with_cell();
        cout << '\n';
    }
    // compare function
    bool operator < (Net a)   { return double(get_2D_len()) / getPin_num() < double(a.get_2D_len()) / a.getPin_num() ; }
private:
    string          _name;          // Name of the net
    vector<Pin*>    _pinArray;      // Array of all pins
    int             _numPin;        // Number of pins of the net
    int             _minLayer;      // Minimum routing layer constraint
    int             _len;           // total length
    int             _2D_len;        // 2D length
};

#endif  // NET_H
