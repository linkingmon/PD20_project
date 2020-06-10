#ifndef CONGESTION_H
#define CONGESTION_H

#include 

class Congestion{

public:
    Congestion(size_t x, size_t y, size_t z)
    :Row(x) , Col(y) , Layer(z){ 
        cong_map.resize( x * y * z );
    }

    double& operator() (size_t x , size_t y, size_t z ){
        return cong_map.at(x + y * Row + z * Row * Col );
    }
private: 
    vector<double> cong_map;
    size_t Row;
    size_t Col;
    size_t Layer;
}
